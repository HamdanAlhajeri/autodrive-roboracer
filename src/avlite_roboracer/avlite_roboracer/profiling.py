"""Small, dependency-free timing tools for finding why the supervisor falls behind its sensors.

The supervisor is one Python thread: it handles every sensor callback and the control tick. If
a tick or a callback runs long, scan and odometry ages grow although the sensors are fine (the
stale-LiDAR stop at mapping start on 23 Nov). Two tools:

- TickStats: how long each part of the control tick took, and the longest gap between ticks.
- StackSampler: a background thread that samples where the supervisor thread is, many times a
  second, so a stop leaves a count of the functions that kept it busy.

No ROS here. Both are off the hot path: TickStats is a few clock reads per tick, the sampler
sleeps between samples and only counts while it is switched on.
"""

from collections import Counter
import sys
import threading

PHASES = ("scans", "supervisor", "faults", "actions", "control", "publish")


class TickStats:
    """Per-phase and per-tick durations of the supervisor's control tick."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.ticks = 0
        self.last_start = None
        self.gap_max_s = 0.0
        self.tick_max_s = 0.0
        self.tick_total_s = 0.0
        self.phase_total_s = dict.fromkeys(PHASES, 0.0)
        self.phase_max_s = dict.fromkeys(PHASES, 0.0)

    def record(self, start, marks):
        """marks maps a phase name to the monotonic time that phase ended, in tick order."""
        if self.last_start is not None:
            self.gap_max_s = max(self.gap_max_s, start - self.last_start)
        self.last_start = start
        previous = start
        for phase in PHASES:
            end = marks.get(phase)
            if end is None:
                continue
            took = end - previous
            self.phase_total_s[phase] += took
            self.phase_max_s[phase] = max(self.phase_max_s[phase], took)
            previous = end
        self.ticks += 1
        self.tick_total_s += previous - start
        self.tick_max_s = max(self.tick_max_s, previous - start)

    def text(self):
        if not self.ticks:
            return "no control ticks recorded"
        mean = self.tick_total_s / self.ticks
        busiest = max(PHASES, key=lambda p: self.phase_total_s[p])
        return (f"{self.ticks} ticks, mean {mean * 1e3:.0f} ms, "
                f"longest {self.tick_max_s * 1e3:.0f} ms, "
                f"longest gap between ticks {self.gap_max_s * 1e3:.0f} ms; most time in "
                f"'{busiest}' ({self.phase_total_s[busiest] / self.tick_total_s:.0%}, "
                f"max {self.phase_max_s[busiest] * 1e3:.0f} ms)")


class StackSampler:
    """Counts the innermost frames of one thread (the supervisor's) while it is switched on."""

    def __init__(self, thread_id, interval_s=0.01, depth=4):
        self.thread_id = thread_id
        self.interval_s = interval_s
        self.depth = depth
        self.active = False
        self.samples = 0
        self.innermost = Counter()
        self.stacks = Counter()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="stack-sampler", daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()

    def reset(self):
        self.samples = 0
        self.innermost.clear()
        self.stacks.clear()

    def _run(self):
        while not self._stop.wait(self.interval_s):
            if self.active:
                self.sample()

    def sample(self):
        frame = sys._current_frames().get(self.thread_id)
        if frame is None:
            return
        names = []
        while frame is not None and len(names) < self.depth:
            code = frame.f_code
            module = code.co_filename.rsplit("/", 1)[-1]
            names.append(f"{module}:{code.co_name}:{frame.f_lineno}")
            frame = frame.f_back
        self.samples += 1
        self.innermost[names[0]] += 1
        self.stacks[" < ".join(names)] += 1

    def report(self, top=15):
        if not self.samples:
            return "no samples"
        lines = [f"{self.samples} samples of the supervisor thread "
                 f"(every {self.interval_s * 1e3:.0f} ms)", "innermost frame:"]
        lines += [f"  {n / self.samples:5.1%}  {name}"
                  for name, n in self.innermost.most_common(top)]
        lines.append("call stacks (innermost first):")
        lines += [f"  {n / self.samples:5.1%}  {stack}"
                  for stack, n in self.stacks.most_common(top)]
        return "\n".join(lines)
