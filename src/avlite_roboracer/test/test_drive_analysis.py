import math

from avlite_roboracer.drive_analysis import (at_minimum_radius, report, speed_and_radius,
                                              stalls)


def straight(seconds, speed, dt=0.25, t0=0.0, x0=0.0):
    return [(t0 + i * dt, x0 + speed * i * dt, 0.0, 0.0) for i in range(int(seconds / dt))]


def test_a_stall_is_found_when_the_command_is_high_and_the_car_barely_moves():
    path = straight(10, 0.05) + straight(5, 0.8, t0=10.0, x0=0.5)
    samples = speed_and_radius(path)
    found = stalls(samples, [(0.0, 0.8)])
    assert len(found) == 1
    start, end, command, mean = found[0]
    assert command == 0.8 and mean < 0.1 and end - start > 8


def test_no_stall_when_the_car_follows_and_circle_radius_is_recovered():
    path = straight(10, 0.7)
    assert stalls(speed_and_radius(path), [(0.0, 0.8)]) == []
    circle = [(i * 0.25, 1.4 * math.cos(0.5 * i * 0.25), 1.4 * math.sin(0.5 * i * 0.25),
               0.5 * i * 0.25 + math.pi / 2) for i in range(60)]   # v = 0.7, R = 1.4
    assert at_minimum_radius(speed_and_radius(circle), 1.4) > 0.95


def test_report_reads_the_csv_files(tmp_path):
    slam, probe = tmp_path / "slam.csv", tmp_path / "probe.csv"
    slam.write_text("stamp_s,x,y,yaw\n" + "".join(
        f"{100 + t:.2f},{0.01 * t:.3f},0,0\n" for t in range(0, 20)))
    probe.write_text("t_s,topic,value\n0.0,drive,0.8\n")
    assert "stall" in report(slam, probe)
