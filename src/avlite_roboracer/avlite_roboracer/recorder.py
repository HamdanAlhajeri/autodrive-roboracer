"""Record, export and replay Jetson sensor sessions.

    roboracer_record record --profile P --output DIR      # rosbag2 + session.json
    roboracer_record export DIR --profile P               # DIR/records.jsonl for reports
    roboracer_record replay DIR --profile P --mode mapping
    roboracer_record replay DIR --profile P --mode localization --pose-graph MAPDIR/posegraph

Recording never publishes commands. rosbag2 is the replay source of truth; the export is
a compact JSON copy for ROS-free analysis on the Jetson or laptop. Replays run SLAM
Toolbox on simulated time and record its output next to the session. Any map -> odom
transform already in the recording is dropped during replay, so the replayed SLAM process
is the only publisher.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

import yaml

from .profile import HardwareProfile
from .records import make_record, write_records
from .slam import SlamProcess, effective_parameters, require_save_success

STATUS_TOPICS = ("/roboracer/status", "/roboracer/actuator_status")
REPLAY_TF = "/replay/tf"
REPLAY_TF_STATIC = "/replay/tf_static"


def replay_topics(profile):
    """Replay sensor inputs only; recorded commands and SLAM outputs are never played."""
    topics = {"/tf", "/tf_static"}
    topics.update(profile.topic(k) for k in ("scan", "odom", "imu") if profile.topic(k))
    forbidden = {profile.topic(k) for k in ("drive_output", "manual_input", "slam_pose",
                                            "slam_graph")}
    forbidden.update((*STATUS_TOPICS, "/roboracer/command", "/map"))
    if topics & forbidden:
        raise ValueError("replay sensor topics overlap command or SLAM output topics")
    return sorted(topics)


def replay_environment(domain_id):
    if not 0 <= domain_id <= 101:
        raise ValueError("replay domain must be between 0 and 101")
    if domain_id == int(os.environ.get("ROS_DOMAIN_ID", "0")):
        raise ValueError("replay domain must differ from the current ROS_DOMAIN_ID")
    return {**os.environ, "ROS_DOMAIN_ID": str(domain_id), "ROS_LOCALHOST_ONLY": "1"}


def topic_kinds(profile):
    """Map recorded topic names to record kinds using the profile's interfaces."""
    kinds = {"/tf": "tf", "/tf_static": "tf_static", "/roboracer/command": "drive"}
    for name, kind in (("scan", "scan"), ("odom", "odom"), ("imu", "imu"),
                       ("slam_pose", "pose"), ("slam_graph", "graph"),
                       ("drive_output", "drive"), ("manual_input", "drive")):
        if profile.topic(name):
            kinds[profile.topic(name)] = kind
    kinds.update({topic: "status" for topic in STATUS_TOPICS})
    return kinds


def record_topics(profile, include_map=False):
    topics = sorted(topic_kinds(profile))
    return topics + (["/map"] if include_map else [])


def git_revision():
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                              cwd=Path(__file__).parent, timeout=5).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def write_session(directory, profile, topics, purpose):
    directory.mkdir(parents=True, exist_ok=False)
    session = {
        "version": 1, "purpose": purpose,
        "started_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "host": platform.node(), "ros_distro": os.environ.get("ROS_DISTRO"),
        "git_revision": git_revision(), "profile": profile.summary(),
        "profile_data": profile.data, "topics": topics,
    }
    (directory / "session.json").write_text(json.dumps(session, indent=2) + "\n")
    return session


def interrupt(process, timeout_s=10.0):
    """Stop a child process group cleanly so rosbag2 finalizes its files."""
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout_s)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def record(args, profile):
    directory = Path(args.output)
    topics = record_topics(profile, args.include_map)
    write_session(directory, profile, topics, args.purpose)
    print(f"Recording {len(topics)} topics to {directory}/bag. Ctrl+C to finish.", flush=True)
    process = subprocess.Popen(["ros2", "bag", "record", "-o", str(directory / "bag"), *topics],
                               start_new_session=True)
    try:
        return process.wait()
    except KeyboardInterrupt:
        interrupt(process)
        return 0


def _storage_id(bag):
    meta = yaml.safe_load((Path(bag) / "metadata.yaml").read_text())
    return meta["rosbag2_bagfile_information"]["storage_identifier"]


def export(args, profile):
    """Convert DIR/bag (or a replay's bag) into records.jsonl using rosbag2_py."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(args.session) / "bag"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=_storage_id(bag)),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    kinds = topic_kinds(profile)
    kinds[REPLAY_TF] = "tf"
    records, skipped = [], 0
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if topic not in kinds or topic not in types:
            continue
        try:
            records.append(make_record(kinds[topic], topic, receive_ns,
                                       deserialize_message(data, types[topic])))
        except (ValueError, AttributeError) as exc:
            skipped += 1
            if skipped <= 5:
                print(f"Skipping malformed {topic} message: {exc}", file=sys.stderr)
    output = Path(args.output or Path(args.session) / "records.jsonl")
    write_records(output, records)
    print(f"Exported {len(records)} records ({skipped} skipped) to {output}")
    return 0


def tf_relay(args):
    """Relay replayed /tf, dropping transforms owned by the live SLAM process."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    drop = {tuple(pair.split(":")) for pair in args.drop}
    rclpy.init()
    node = Node("roboracer_replay_tf_relay")

    def subscribe(source, target, qos):
        publisher = node.create_publisher(TFMessage, target, qos)

        def relay(msg):
            kept = [t for t in msg.transforms
                    if (t.header.frame_id, t.child_frame_id) not in drop]
            if kept:
                publisher.publish(TFMessage(transforms=kept))

        node.create_subscription(TFMessage, source, relay, qos)

    subscribe(REPLAY_TF, "/tf", 100)
    subscribe(REPLAY_TF_STATIC, "/tf_static",
              QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def replay(args, profile):
    """Replay a session through SLAM Toolbox and record the result in a new session."""
    env = replay_environment(args.domain_id)
    inputs = replay_topics(profile)
    session = Path(args.session).resolve()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    output = session / f"replay-{args.mode}-{stamp}"
    topics = record_topics(profile, include_map=True)
    write_session(output, profile, topics, f"replay {args.mode} of {session.name}")
    params = effective_parameters(
        args.slam_params, profile, args.mode, output / "slam_params.yaml",
        map_file=args.pose_graph, start_pose=args.start_pose, use_sim_time=True)
    drop = f"{profile.frame('map')}:{profile.frame('odom')}"
    relay = subprocess.Popen([sys.executable, "-m", "avlite_roboracer.recorder", "tf-relay",
                              "--drop", drop], start_new_session=True, env=env)
    slam = SlamProcess(output)
    recorder = None
    playback = None
    try:
        slam.start(args.mode, params, use_sim_time=True, env=env)
        recorder = subprocess.Popen(["ros2", "bag", "record", "--use-sim-time", "-o",
                                     str(output / "bag"), *topics], start_new_session=True,
                                    env=env)
        time.sleep(3.0)  # let SLAM and the recorder discover topics before playback
        playback = subprocess.Popen(
            ["ros2", "bag", "play", str(session / "bag"), "--clock",
             "--rate", str(args.rate), "--topics", *inputs, "--remap",
             f"/tf:={REPLAY_TF}", f"/tf_static:={REPLAY_TF_STATIC}"],
            start_new_session=True, env=env)
        code = playback.wait()
        if code != 0:
            raise RuntimeError(f"ros2 bag play failed with exit code {code}")
        time.sleep(2.0)
        if args.mode == "mapping":
            subprocess.run([sys.executable, "-m", "avlite_roboracer.recorder",
                            "save-replay-map", str(output)], env=env, check=True, timeout=90)
        print(f"Replay output: {output}")
        print(f"Next: roboracer_record export {output} --profile {args.profile}")
        return 0
    finally:
        if playback is not None:
            interrupt(playback)
        if recorder is not None:
            interrupt(recorder)
        slam.stop()
        interrupt(relay)


def save_replay_map(directory):
    """Run inside the replay domain and check both SLAM service result codes."""
    import rclpy
    from slam_toolbox.srv import SaveMap, SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node("roboracer_replay_save")
    directory = Path(directory).resolve()
    save = SaveMap.Request()
    save.name.data = str(directory / "map")
    requests = ((SerializePoseGraph, "serialize_map",
                 SerializePoseGraph.Request(filename=str(directory / "posegraph"))),
                (SaveMap, "save_map", save))
    try:
        for service, name, request in requests:
            client = node.create_client(service, f"/slam_toolbox/{name}")
            try:
                if not client.wait_for_service(timeout_sec=10.0):
                    raise RuntimeError(f"{name} unavailable")
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
                if not future.done() or future.result() is None:
                    raise RuntimeError(f"{name} timed out or failed")
                require_save_success(future.result(), name)
            finally:
                node.destroy_client(client)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--profile", required=True)
    rec.add_argument("--output", required=True)
    rec.add_argument("--purpose", default="manual commissioning run")
    rec.add_argument("--include-map", action="store_true", help="Also record /map")
    exp = sub.add_parser("export")
    exp.add_argument("session")
    exp.add_argument("--profile", required=True)
    exp.add_argument("--output")
    rep = sub.add_parser("replay")
    rep.add_argument("session")
    rep.add_argument("--profile", required=True)
    rep.add_argument("--mode", choices=("mapping", "localization"), required=True)
    rep.add_argument("--slam-params", default=str(
        Path(__file__).resolve().parents[3] / "config" / "roboracer" / "slam_toolbox.yaml"))
    rep.add_argument("--pose-graph", help="Serialized pose graph (no extension)")
    rep.add_argument("--start-pose", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                     metavar=("X", "Y", "YAW"), help="Replay start pose in the map frame")
    rep.add_argument("--rate", type=float, default=1.0)
    rep.add_argument("--domain-id", type=int, default=73,
                     help="Isolated replay ROS domain, different from the live car (default: 73)")
    save = sub.add_parser("save-replay-map")
    save.add_argument("directory")
    relay = sub.add_parser("tf-relay")
    relay.add_argument("--drop", nargs="+", default=["map:odom"])
    args = parser.parse_args(argv)
    if args.command == "tf-relay":
        return tf_relay(args)
    if args.command == "save-replay-map":
        return save_replay_map(args.directory)
    profile = HardwareProfile.load(args.profile)
    if args.command == "replay" and args.mode == "localization" and not args.pose_graph:
        parser.error("--pose-graph is required for localization replay")
    return {"record": record, "export": export, "replay": replay}[args.command](args, profile)


if __name__ == "__main__":
    raise SystemExit(main())
