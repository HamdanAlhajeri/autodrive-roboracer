# Documentation

Start with the root [README](../README.md) and [team contribution guide](../CONTRIBUTING.md).

## Working guides

| Task | Guide |
| --- | --- |
| Start, stop, reload, record, and inspect speed | [Windows commands](commands.md) |
| Architecture, ROS interfaces, Linux setup, telemetry and troubleshooting | [Setup and recording](avlite-setup.md) |
| Prepare maps and configure the race planner | [Planned driving](planned-driving.md) |
| Build, select, test or roll back the custom track | [Sketch track](sketch-track.md) |
| Measure acceleration and braking response | [Response measurements](response-measurements.md) |
| Record, map, localize and race the Jetson car | [Jetson car](jetson.md) |

## Plans and evidence

| Location | Contents |
| --- | --- |
| [plans/checklist-2026-09-21.md](plans/checklist-2026-09-21.md) | Ongoing implementation and validation checklist |
| [plans/milestone-2026-09-21.md](plans/milestone-2026-09-21.md) | Original milestone and acceptance criteria |
| [plans/LOCALIZATIONPLAN.md](plans/LOCALIZATIONPLAN.md) | Jetson map-one-lap-then-race plan; software implemented, hardware validation pending |
| [plans/real-car-plugin.md](plans/real-car-plugin.md) | Earlier hardware proposal (AVLite ICP); superseded by the SLAM Toolbox plan |
| [validation/](validation/) | Dated measurements, graphs and test reports |
| [validation/controller-history.md](validation/controller-history.md) | Earlier steering and actuator comparisons, moved out of the README |
| [history/avlite-integration-plan.md](history/avlite-integration-plan.md) | Original integration plan |
| [research/localization-resources.txt](research/localization-resources.txt) | Mapping/localization reading list |

Dated reports describe the settings used for those runs. Read `config/` and run
`..\avlite.ps1 speed` from this directory to inspect the current profile.
Raw recordings and locally built simulators are in ignored `log/` storage.

## Legacy documentation

[source/](source/) and [Dockerfile](Dockerfile) retain the Sphinx documentation
for the original standalone ROS controller. Its launch files are now under
`docker/legacy/`. Use the working guides above for the AVLite stack.
