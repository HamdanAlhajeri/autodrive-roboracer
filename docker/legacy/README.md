# Original standalone controller

These Compose files run `src/my_team_racer`, the pre-AVLite ROS controller, and
its Sphinx documentation. The active AVLite stack uses the two root Compose
files and `avlite.ps1`. The legacy controller is retained because it still has
regression tests and reference documentation.

Run from the repository root on Linux with the original setup prerequisites:

```bash
docker compose --project-directory . -f docker/legacy/compose.yml up simulator devkit
docker compose --project-directory . -f docker/legacy/compose.yml -f docker/legacy/compose.dev.yml run --rm devkit
bash scripts/linux/test-legacy.sh
```

`--project-directory .` is required: source mounts and documentation build paths
remain relative to the repository root. Stop AVLite before starting the legacy
controller; both publish vehicle commands.

The [Sphinx sources](../../docs/source/) describe this older workflow. Current
AVLite setup is documented in [docs/avlite-setup.md](../../docs/avlite-setup.md).
