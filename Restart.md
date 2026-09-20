# Restart the Windows simulator stack

Run these commands from the repository in PowerShell, with Docker Desktop open.

After editing settings, restart both controllers:

```powershell
docker compose -f docker-compose.avlite.yml -f docker-compose.windows.yml restart avlite actuator
```

This resumes driving when the simulator is connected in Autonomous mode. For a
recorded test, use `.\record-one-lap.ps1 -Laps 3` instead; it handles controller
reload, reset instructions, recording and stopping.

To restart the entire stack and simulator:

```powershell
.\run-windows.ps1 -Stop
.\run-windows.ps1
```

Select **Connection** (`127.0.0.1:4567`) and **Autonomous** in the simulator.
See [setup and recording](docs/avlite-setup.md) for logs and troubleshooting.
