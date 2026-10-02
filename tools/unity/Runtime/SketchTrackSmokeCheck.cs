// Only runs when the standalone simulator is launched with -sketch-smoke-test.
using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

public class SketchTrackSmokeCheck : MonoBehaviour
{
    public VehicleController Vehicle;
    public GPS Position;
    public LIDAR Scanner;
    public LapTimer Laps;
    public ResetManager Reset;
    public Transform FinishCrossingPose;
    public Vector3 ExpectedRearAxle;

    /// <summary>
    /// Run scene checks only when -sketch-smoke-test is present; otherwise do nothing.
    /// Let the car settle, check spawn position, wheel contact and LiDAR, then move it
    /// through checkpoint triggers to verify lap counting and reset behavior. These
    /// controlled movements test scene wiring, not autonomous driving. Save a screenshot
    /// and JSON report, then exit with a status the PowerShell test runner can check.
    /// </summary>
    IEnumerator Start()
    {
        string[] args = Environment.GetCommandLineArgs();
        int flag = Array.IndexOf(args, "-sketch-smoke-test");
        if (flag < 0) yield break;
        string output = flag + 1 < args.Length ? args[flag + 1] : "sketch-smoke.json";
        string failure = null;
        Vehicle.DrivingMode = 0;
        Vehicle.AutonomousThrottle = 0;
        for (int i = 0; i < 100; i++) yield return new WaitForFixedUpdate();
        Vector3 pose = Position.VehicleTransform.position;
        float planarError = Vector2.Distance(new Vector2(pose.x, pose.z),
            new Vector2(ExpectedRearAxle.x, ExpectedRearAxle.z));
        int wheelsGrounded = new[] {Vehicle.FrontLeftWheelCollider, Vehicle.FrontRightWheelCollider,
            Vehicle.RearLeftWheelCollider, Vehicle.RearRightWheelCollider}.Count(w => w.isGrounded);
        int finiteHits = Scanner.CurrentRangeArray.Count(s => float.TryParse(s, NumberStyles.Float,
            CultureInfo.InvariantCulture, out float r) && r > Scanner.MinimumLinearRange
            && r < Scanner.MaximumLinearRange && !float.IsInfinity(r));
        if (planarError > 0.25f || wheelsGrounded != 4 || finiteHits < 20)
            failure = "Spawn, ground contact or LiDAR check failed";
        ScreenCapture.CaptureScreenshot(Path.ChangeExtension(output, ".png"));
        for (int i = 0; i < 10; i++) yield return null;

        var body = Vehicle.VehicleRigidBody;
        Vehicle.enabled = false;
        Reset.enabled = false;
        var coSim = body.GetComponent<CoSimManager>();
        if (coSim != null) coSim.enabled = false;
        // Move the car directly for trigger checks, without wheel forces or driving
        // commands interfering. Physics steps below still let trigger events fire.
        body.isKinematic = true;
        Laps.LapCount = Laps.CheckpointCount = Laps.CurrentCheckpoint = Laps.PreviousCheckpoint = 0;
        foreach (Transform checkpoint in Laps.Checkpoints)
            yield return Cross(body, checkpoint);
        yield return Cross(body, FinishCrossingPose);
        int firstLap = Laps.LapCount;
        foreach (Transform checkpoint in Laps.Checkpoints)
            yield return Cross(body, checkpoint);
        yield return Cross(body, FinishCrossingPose);
        int secondLap = Laps.LapCount;
        if (firstLap != 1 || secondLap != 2) failure = "Consecutive lap trigger check failed";

        Reset.enabled = true;
        Reset.ResetFlag = true;
        // ResetManager consumes its flag in Update. Several FixedUpdates can
        // run before one rendered frame during initial shader/driver warmup.
        for (int i = 0; i < 60 && Reset.ResetFlag; i++) yield return null;
        for (int i = 0; i < 5; i++) yield return new WaitForFixedUpdate();
        Vector3 resetPose = Position.VehicleTransform.position;
        float resetError = Vector2.Distance(new Vector2(resetPose.x, resetPose.z),
            new Vector2(ExpectedRearAxle.x, ExpectedRearAxle.z));
        if (resetError > 0.05f || Laps.LapCount != 0 || Laps.CollisionCount != 0)
            failure = "Reset does not restore the sketch spawn and counters";
        string json = "{\n  \"passed\": " + (failure == null ? "true" : "false")
            + ",\n  \"spawn_error_m\": " + planarError.ToString(Invariant())
            + ",\n  \"grounded_wheels\": " + wheelsGrounded
            + ",\n  \"finite_lidar_hits\": " + finiteHits
            + ",\n  \"first_lap\": " + firstLap + ",\n  \"second_lap\": " + secondLap
            + ",\n  \"reset_error_m\": " + resetError.ToString(Invariant())
            + ",\n  \"failure\": \"" + (failure ?? "") + "\"\n}\n";
        File.WriteAllText(output, json);
        Debug.Log("SKETCH_SMOKE " + json);
        Application.Quit(failure == null ? 0 : 1);
    }

    /// <summary>
    /// Use decimal points in JSON numbers regardless of Windows regional settings.
    /// </summary>
    static CultureInfo Invariant() => CultureInfo.InvariantCulture;

    /// <summary>
    /// Move the test body from 0.6 m before a gate to 0.6 m after it, facing forward.
    /// Synchronize each position and allow physics steps so the lap timer receives
    /// trigger events instead of having the body skip over the gate in one jump.
    /// </summary>
    static IEnumerator Cross(Rigidbody body, Transform pose)
    {
        foreach (float offset in new[] {-0.6f, 0f, 0.6f})
        {
            body.position = pose.position + pose.forward * offset;
            body.rotation = pose.rotation;
            Physics.SyncTransforms();
            for (int i = 0; i < 3; i++) yield return new WaitForFixedUpdate();
        }
    }
}
