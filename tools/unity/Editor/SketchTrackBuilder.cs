// Copy into Assets/Editor/AutoDRIVEIntegration in the AutoDRIVE Unity source project.
// Batch entry points: AutoDriveTrackTools.SketchTrackBuilder.CreateScene / BuildPlayer.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace AutoDriveTrackTools
{
    public static class SketchTrackBuilder
    {
        const string SourceScene = "Assets/Scenes/RoboRacer - Sim Racing.unity";
        public const string ScenePath = "Assets/Scenes/RoboRacer - Sketch Track.unity";
        const string AssetRoot = "Assets/Tracks/SketchTrack";
        const string BarrierName = "Sketch Track Barrier";
        // The stock scanner is at 0.156 m, above the ZIP's 0.1524 m tubes.
        // Stretch only Y so the physical XY corridor and AVLite map are unchanged.
        const float BarrierHeightScale = 2f;
        static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

        /// <summary>
        /// Read the path following a required command-line flag and make it absolute.
        /// Batch builds pass source and output locations this way; a missing value
        /// stops the build before it can use an unintended working directory.
        /// </summary>
        static string Argument(string flag)
        {
            var args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, flag);
            if (index < 0 || index + 1 == args.Length)
                throw new ArgumentException("Missing " + flag);
            return Path.GetFullPath(args[index + 1]);
        }

        /// <summary>
        /// Find components of type T beneath every root in this scene, including
        /// inactive objects. This keeps lookups within the scene being converted.
        /// </summary>
        static T[] InScene<T>(Scene scene) where T : Component => scene.GetRootGameObjects()
            .SelectMany(root => root.GetComponentsInChildren<T>(true)).ToArray();

        /// <summary>
        /// Create a generated asset, or copy new data into the existing asset.
        /// Updating in place preserves Unity references when the track is rebuilt.
        /// </summary>
        static void SaveAsset(Object value, string path)
        {
            var existing = AssetDatabase.LoadAssetAtPath<Object>(path);
            if (existing == null) AssetDatabase.CreateAsset(value, path);
            else EditorUtility.CopySerialized(value, existing);
        }

        /// <summary>
        /// Save and return a low-smoothness HDRP material for a road or barrier.
        /// The name also determines its generated asset filename; the color sets
        /// its appearance without changing the collider's physical properties.
        /// </summary>
        static Material MakeMaterial(string name, Color color)
        {
            var shader = Shader.Find("HDRP/Lit");
            if (shader == null) throw new InvalidOperationException("HDRP/Lit shader is unavailable");
            var material = new Material(shader) { name = name };
            material.SetColor("_BaseColor", color);
            material.SetFloat("_Smoothness", 0.15f);
            var path = AssetRoot + "/Generated/" + name + ".mat";
            SaveAsset(material, path);
            return AssetDatabase.LoadAssetAtPath<Material>(path);
        }

        /// <summary>
        /// Read the supplied triangulated OBJ into separate road and barrier meshes.
        /// Vertices already use Unity coordinates, so no importer scale or handedness
        /// conversion is applied. Each named object gets local vertex indices, new
        /// normals and a saved mesh asset. Unexpected objects or faces are rejected.
        /// </summary>
        static Dictionary<string, Mesh> ReadMeshes(string filename)
        {
            var vertices = new List<Vector3>();
            var faces = new Dictionary<string, List<int>>();
            List<int> current = null;
            foreach (string line in File.ReadLines(filename))
            {
                var parts = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0) continue;
                if (parts[0] == "o")
                {
                    current = new List<int>();
                    faces.Add(parts[1], current);
                }
                else if (parts[0] == "v")
                    vertices.Add(new Vector3(float.Parse(parts[1], Invariant),
                        float.Parse(parts[2], Invariant), float.Parse(parts[3], Invariant)));
                else if (parts[0] == "f")
                {
                    if (current == null || parts.Length != 4)
                        throw new InvalidDataException("Expected named, triangulated OBJ objects");
                    for (int k = 1; k < 4; k++)
                    {
                        int index = int.Parse(parts[k].Split('/')[0], Invariant) - 1;
                        if (index < 0 || index >= vertices.Count)
                            throw new InvalidDataException("Invalid OBJ vertex index");
                        current.Add(index);
                    }
                }
            }
            var result = new Dictionary<string, Mesh>();
            foreach (var item in faces)
            {
                var indices = item.Value.Distinct().ToArray();
                var remap = indices.Select((index, i) => new { index, i })
                    .ToDictionary(pair => pair.index, pair => pair.i);
                var mesh = new Mesh { name = item.Key, indexFormat = IndexFormat.UInt32 };
                mesh.vertices = indices.Select(index => vertices[index]).ToArray();
                mesh.triangles = item.Value.Select(index => remap[index]).ToArray();
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                var path = AssetRoot + "/Generated/" + item.Key + ".asset";
                SaveAsset(mesh, path);
                result.Add(item.Key, AssetDatabase.LoadAssetAtPath<Mesh>(path));
            }
            if (!result.Keys.OrderBy(n => n).SequenceEqual(
                new[] { "Inner_Barrier", "Outer_Barrier", "Track_Surface" }))
                throw new InvalidDataException("Expected road, inner barrier and outer barrier meshes");
            return result;
        }

        sealed class Route
        {
            public Vector3[] Points;
            public float[] Distance;
            public float Length;
            /// <summary>
            /// Convert AVLite's world (x, y) path into Unity (-y, 0, x) coordinates.
            /// Store cumulative distance in metres, including the closing segment,
            /// so spawn poses and gates can be placed by distance around the lap.
            /// </summary>
            public Route(JObject plan)
            {
                Points = plan["path"].Select(p => new Vector3(-(float)p[1], 0, (float)p[0])).ToArray();
                Distance = new float[Points.Length + 1];
                for (int i = 0; i < Points.Length; i++)
                    Distance[i + 1] = Distance[i] + Vector3.Distance(Points[i], Points[(i + 1) % Points.Length]);
                Length = Distance[Distance.Length - 1];
            }
            /// <summary>
            /// Return a position at the requested distance in metres along the route.
            /// Wrap distance at the finish line, then interpolate within its segment.
            /// </summary>
            public Vector3 At(float distance)
            {
                distance = Mathf.Repeat(distance, Length);
                int i = Array.BinarySearch(Distance, distance);
                if (i < 0) i = ~i - 1;
                return Vector3.Lerp(Points[i], Points[(i + 1) % Points.Length],
                    (distance - Distance[i]) / (Distance[i + 1] - Distance[i]));
            }
            /// <summary>
            /// Face along the route using a point 0.1 m ahead to estimate direction.
            /// At handles wrapping, so orientation remains defined across the finish.
            /// </summary>
            public Quaternion Rotation(float distance) => Quaternion.LookRotation(
                At(distance + 0.1f) - At(distance), Vector3.up);
        }

        /// <summary>
        /// Create a checkpoint or finish trigger across the track at a route distance.
        /// Sideways raycasts find the nearest barrier on each side, setting the gate's
        /// centre and width. The tag and matching layer connect it to the lap timer.
        /// Missing walls raise an error instead of leaving a gate across open space.
        /// </summary>
        static GameObject Gate(Transform parent, string name, string tag, Route route, float distance)
        {
            var rotation = route.Rotation(distance);
            var point = route.At(distance);
            var right = rotation * Vector3.right;
            // Cast through the middle of the tubes, restricted to physical barriers.
            var origin = point + Vector3.up * (0.0762f * BarrierHeightScale);
            var rightHits = Physics.RaycastAll(origin, right, 40, 1, QueryTriggerInteraction.Ignore);
            var leftHits = Physics.RaycastAll(origin, -right, 40, 1, QueryTriggerInteraction.Ignore);
            var rightWalls = rightHits.Where(h => h.collider.name == BarrierName).OrderBy(h => h.distance).ToArray();
            var leftWalls = leftHits.Where(h => h.collider.name == BarrierName).OrderBy(h => h.distance).ToArray();
            if (rightWalls.Length == 0 || leftWalls.Length == 0)
                throw new InvalidOperationException("Cannot locate both walls for gate " + name
                    + " at " + origin + "; right hits: " + string.Join(",", rightHits.Select(h => h.collider.name))
                    + "; left hits: " + string.Join(",", leftHits.Select(h => h.collider.name)));
            var r = rightWalls[0];
            var l = leftWalls[0];
            var gate = new GameObject(name) { tag = tag, layer = LayerMask.NameToLayer(tag) };
            gate.transform.SetParent(parent, false);
            gate.transform.SetPositionAndRotation(point + right * (r.distance - l.distance) / 2
                + Vector3.up * 0.3f, rotation);
            var collider = gate.AddComponent<BoxCollider>();
            collider.isTrigger = true;
            collider.size = new Vector3(r.distance + l.distance - 0.04f, 0.6f, 0.08f);
            return gate;
        }

        /// <summary>
        /// Build a separate sketch scene from the stock racing scene and validated plan.
        /// Replace the active track geometry, align the car's rear axle with the route,
        /// and add lap gates, reset poses and the optional smoke-test component. Road
        /// coverage and LiDAR height are checked before saving the scene and report.
        /// This prepares the scene; autonomous driving still needs its own validation.
        /// </summary>
        public static void CreateScene()
        {
            var source = Argument("-sketchSource");
            var plan = JObject.Parse(File.ReadAllText(Path.Combine(source, "config/maps/sketch.plan.json")));
            var route = new Route(plan);
            Directory.CreateDirectory(AssetRoot + "/Generated");
            AssetDatabase.Refresh();
            var scene = EditorSceneManager.OpenScene(SourceScene, OpenSceneMode.Single);
            var socket = InScene<Socket>(scene).Single();
            if (socket.VehicleControllers.Length != 1 || socket.PositioningSystems.Length != 1)
                throw new InvalidOperationException("Expected a solo RoboRacer scene");
            var controller = socket.VehicleControllers[0];
            var vehicle = controller.VehicleRigidBody.transform;
            var gps = socket.PositioningSystems[0].VehicleTransform;
            var lap = socket.LapTimers[0];
            var prefab = PrefabUtility.GetOutermostPrefabInstanceRoot(vehicle.gameObject);
            if (prefab != null)
                PrefabUtility.UnpackPrefabInstance(prefab, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
            var infrastructure = scene.GetRootGameObjects().Single(g => g.name == "Infrastructure");
            var floor = scene.GetRootGameObjects().Single(g => g.name == "Floor");
            var roadPhysics = floor.GetComponent<Collider>().sharedMaterial;
            infrastructure.SetActive(false);
            floor.SetActive(false);

            var track = new GameObject("Sketch Track");
            var meshes = ReadMeshes(Path.Combine(source, "assets/tracks/sketch_track/sketch_track.obj"));
            var roadMaterial = MakeMaterial("Sketch Asphalt", new Color(0.11f, 0.12f, 0.14f));
            var barrierMaterial = MakeMaterial("Sketch Barrier", new Color(0.72f, 0.035f, 0.025f));
            foreach (var item in meshes)
            {
                var group = new GameObject(item.Key);
                group.transform.SetParent(track.transform, false);
                bool road = item.Key == "Track_Surface";
                var geometry = new GameObject(road ? "Sketch Road" : BarrierName);
                geometry.transform.SetParent(group.transform, false);
                if (!road) geometry.transform.localScale = new Vector3(1, BarrierHeightScale, 1);
                geometry.AddComponent<MeshFilter>().sharedMesh = item.Value;
                geometry.AddComponent<MeshRenderer>().sharedMaterial = road ? roadMaterial : barrierMaterial;
                var collider = geometry.AddComponent<MeshCollider>();
                collider.sharedMesh = item.Value;
                collider.convex = false;
                collider.sharedMaterial = roadPhysics;
                geometry.isStatic = true;
            }
            Physics.SyncTransforms();
            for (float s = 0; s < route.Length; s += 0.25f)
            {
                if (!Physics.Raycast(route.At(s) + Vector3.up, Vector3.down,
                    out var hit, 1.1f, 1, QueryTriggerInteraction.Ignore) || hit.collider.name != "Sketch Road")
                    throw new InvalidOperationException("Road collider is absent below the plan at " + s);
            }

            // Preserve the prefab's axle-to-root offset and wheel/ground height.
            float initialHeight = vehicle.position.y;
            var axleToRoot = Quaternion.Inverse(vehicle.rotation) * (vehicle.position - gps.position);
            axleToRoot.y = 0;
            vehicle.SetPositionAndRotation(route.At(0) + route.Rotation(0) * axleToRoot
                + Vector3.up * initialHeight, route.Rotation(0));
            controller.DrivingMode = 0;
            controller.AutonomousThrottle = 0;
            controller.AutonomousSteering = 0;
            lap.RacetrackName = BarrierName;
            lap.Checkpoints = new Transform[16];
            var checkpoints = new GameObject("Sketch Checkpoints").transform;
            checkpoints.SetParent(track.transform, false);
            for (int i = 0; i < lap.Checkpoints.Length; i++)
            {
                float s = route.Length * i / lap.Checkpoints.Length;
                var pose = new GameObject("Respawn " + i).transform;
                pose.SetParent(checkpoints, false);
                pose.SetPositionAndRotation(route.At(s) + route.Rotation(s) * axleToRoot
                    + Vector3.up * initialHeight, route.Rotation(s));
                lap.Checkpoints[i] = pose;
                Gate(checkpoints, i.ToString(Invariant), "Checkpoint", route, s);
            }
            Gate(checkpoints, "Sketch Finish", "Finish Line A", route, route.Length - 1.1f);
            lap.CurrentCheckpoint = lap.PreviousCheckpoint = lap.CheckpointCount = 0;
            lap.LapCount = lap.CollisionCount = 0;

            var finishPose = new GameObject("Finish Test Pose").transform;
            finishPose.SetParent(checkpoints, false);
            float finishDistance = route.Length - 1.1f;
            finishPose.SetPositionAndRotation(route.At(finishDistance)
                + route.Rotation(finishDistance) * axleToRoot + Vector3.up * initialHeight,
                route.Rotation(finishDistance));
            var smoke = track.AddComponent<SketchTrackSmokeCheck>();
            smoke.Vehicle = controller;
            smoke.Position = socket.PositioningSystems[0];
            smoke.Scanner = socket.LIDARUnits.Single();
            smoke.Laps = lap;
            smoke.Reset = socket.ResetManagers.Single();
            smoke.FinishCrossingPose = finishPose;
            smoke.ExpectedRearAxle = gps.position;

            var scan = socket.LIDARUnits.Single();
            float lidarHeight = scan.Head.transform.position.y;
            if (lidarHeight <= 0 || lidarHeight >= 0.1524f * BarrierHeightScale)
                throw new InvalidOperationException("LiDAR height misses the supplied tube barriers: " + lidarHeight);

            var camera = scene.GetRootGameObjects().Single(g => g.name == "Trackcam");
            foreach (var follower in camera.GetComponentsInChildren<AbstractTargetFollower>(true))
                follower.enabled = false;
            foreach (var follower in camera.GetComponentsInChildren<FollowTarget>(true))
                follower.enabled = false;
            camera.transform.SetPositionAndRotation(new Vector3(0, 32, 0), Quaternion.Euler(90, 0, 0));
            var cameraComponent = camera.GetComponentInChildren<Camera>(true);
            cameraComponent.orthographic = true;
            cameraComponent.orthographicSize = 18.5f;
            var switcher = InScene<CameraSwitch>(scene).Single();
            switcher.Cameras = new[] { camera }.Concat(switcher.Cameras.Where(c => c != camera)).ToArray();
            foreach (var item in switcher.Cameras) item.SetActive(item == camera);
            switcher.Label.text = "Sketch Track";

            Lightmapping.Clear();
            PlayerSettings.productName = "AutoDRIVE Simulator - Sketch Track";
            PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone, ScriptingImplementation.Mono2x);
            PlayerSettings.defaultIsNativeResolution = false;
            PlayerSettings.defaultScreenWidth = 960;
            PlayerSettings.defaultScreenHeight = 540;
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            if (!EditorSceneManager.SaveScene(scene, ScenePath)) throw new IOException("Scene save failed");
            AssetDatabase.SaveAssets();
            var report = new JObject {
                ["scene"] = ScenePath, ["plan_map_sha256"] = plan["map_sha256"],
                ["route_length_m"] = route.Length, ["mesh_objects"] = meshes.Count,
                ["checkpoint_count"] = lap.Checkpoints.Length, ["lidar_height_m"] = lidarHeight,
                ["barrier_height_m"] = 0.1524f * BarrierHeightScale,
                ["barrier_vertical_scale"] = BarrierHeightScale,
                ["rear_axle_unity_position"] = JArray.FromObject(new[] {gps.position.x, gps.position.y, gps.position.z}),
                ["vehicle_unity_yaw_deg"] = vehicle.eulerAngles.y,
                ["checks"] = new JArray("literal OBJ coordinates", "non-convex static colliders",
                    "road raycasts every 0.25 m", "checkpoint wall intersections", "LiDAR height within barriers"),
                ["driving_tested"] = false
            };
            File.WriteAllText("sketch-scene-validation.json", report.ToString());
            Debug.Log("SKETCH_SCENE_READY " + report.ToString(Newtonsoft.Json.Formatting.None));
        }

        /// <summary>
        /// Batch entry point that prepares the sketch scene and then builds its player.
        /// A scene-generation failure prevents the build stage from running.
        /// </summary>
        public static void CreateAndBuild()
        {
            CreateScene();
            BuildPlayer();
        }

        /// <summary>
        /// Build the saved sketch scene as a Windows 64-bit simulator at -sketchBuild.
        /// Select the desktop HDRP profile and Direct3D 11, disable automatic XR startup,
        /// and save those project settings before building. A failed build raises an
        /// error so the PowerShell caller does not report an incomplete player as ready.
        /// </summary>
        public static void BuildPlayer()
        {
            if (!File.Exists(ScenePath)) throw new FileNotFoundException("Create the sketch scene first");
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.StandaloneWindows64, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.StandaloneWindows64,
                new[] { GraphicsDeviceType.Direct3D11 });
            // Quality levels use this profile, but upstream's global fallback
            // points at an unrelated terrain profile and adds its shader variants.
            var pipeline = AssetDatabase.LoadAssetAtPath<HDRenderPipelineAsset>(
                "Assets/Rendering Settings/HDRP Profile.asset");
            if (pipeline == null) throw new InvalidOperationException("Missing desktop HDRP profile");
            GraphicsSettings.defaultRenderPipeline = pipeline;
            var settings = pipeline.currentPlatformRenderPipelineSettings;
            settings.supportedLitShaderMode = RenderPipelineSettings.SupportedLitShaderMode.ForwardOnly;
            pipeline.currentPlatformRenderPipelineSettings = settings;
            EditorUtility.SetDirty(pipeline);
            var xr = UnityEditor.XR.Management.XRGeneralSettingsPerBuildTarget
                .XRGeneralSettingsForBuildTarget(BuildTargetGroup.Standalone);
            if (xr != null)
            {
                xr.InitManagerOnStart = false;
                EditorUtility.SetDirty(xr);
            }
            AssetDatabase.SaveAssets();
            var output = Argument("-sketchBuild");
            Directory.CreateDirectory(Path.GetDirectoryName(output));
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { ScenePath }, locationPathName = output,
                target = BuildTarget.StandaloneWindows64, options = BuildOptions.None
            });
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException("Sketch build failed: " + report.summary.result);
            Debug.Log("SKETCH_BUILD_READY " + output);
        }
    }
}
