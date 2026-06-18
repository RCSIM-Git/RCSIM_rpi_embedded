# Testing Guide - Fixed Deployment

## What was fixed?

We have introduced **3 key diagnostic improvements** to help identify the root cause of the "DEPLOYMENT FAILED (Code: 1)" error:

### 1️⃣ Improved entrypoint.sh
- Added verification tests before the application starts.
- Checks if PySide6 imports correctly.
- Provides human-readable error messages.

### 2️⃣ Dockerfile Verification
- The build process will abort immediately if PySide6 fails to install.
- Fail-fast approach to catch issues early.

### 3️⃣ Better Diagnostics in deployment_logic.py
- Automatically retrieves container logs on failure.
- Verifies if the container is actively running.

---

## How to Test?

### Step 1: Run the RCSIM Deployment Tool
```bash
cd RCSIM_deployment_tool/RCSIM_deployment_tool
python RCsimRPi5deploymentapp.py
```

### Step 2: Enter Details and Deploy
1. **RPi Tailscale IP:** (e.g., 100.x.x.x)
2. **SSH User:** (e.g., pi)
3. **SSH Password:** (the password for your RPi)
4. **Click:** "Deploy to Raspberry Pi"

### Step 3: Observe the Logs
You will now see **much more detail**:

#### ✅ Success Scenario:
```
[ENTRYPOINT] Starting RCSIM Container...
[ENTRYPOINT] Python version: Python 3.11.x
[ENTRYPOINT] Testing Python imports...
✓ PySide6 OK
[ENTRYPOINT] All checks passed. Starting supervisor...
✓ Container is RUNNING. Streaming initial logs...
--- DEPLOYMENT COMPLETED SUCCESSFULLY! ---
```

#### ❌ Error Scenario - Detailed output:
```
[ENTRYPOINT] ERROR: PySide6 import failed!
ModuleNotFoundError: No module named 'PySide6.QtCore'
```
or
```
✗ Container NOT running. Fetching error logs...
[detailed container logs - last 50 lines]
ERROR: Container failed to start!
```

---

## What to Do Next?

### A) If Deployment Completes Successfully ✅
Excellent! The system is operational. You can now:
- Launch the RCSIM PC App and connect to the robot.
- Verify the telemetry and video stream.

### B) If You Encounter an Error ❌
**Copy the ENTIRE output from the deployment logs window** and:
1. Review the troubleshooting section in `docs/README_DOCKER.md`.
2. Or send the logs to the lead developer for further debugging.

---

## Quick Manual Diagnostics (SSH)

If you need to check the status directly on the Raspberry Pi:

```bash
# Connect to the RPi
ssh username@rpi_tailscale_ip

# Check container status
docker ps -a

# View container logs
docker logs rcsim_industrial

# If the container crashed, view the last 100 lines
docker logs --tail 100 rcsim_industrial
```

---

## FAQ - Common Issues

### Q: The build succeeded, but the container doesn't start.
**A:** This is likely an issue with PySide6 or missing system Qt libraries on the host OS.  
Refer to the troubleshooting notes in `docs/README_DOCKER.md`.

### Q: "ERROR: PySide6 import failed during build!"
**A:** PySide6 was not installed correctly from pip. Possible workarounds:
1. Check your connection to `piwheels.org`.
2. Install the package system-wide: `python3-pyside6.qtcore`.

### Q: "Container rcsim_industrial not running"
**A:** The container started but exited immediately. The logs will display the reason (now fetched automatically).

---

## Modified Files

    `RCSIM_RPI/RCSIM_rpi_embedded/rpi_project_source/entrypoint.sh` - startup diagnostics  
    `RCSIM_RPI/RCSIM_rpi_embedded/rpi_project_source/Dockerfile` - PySide6 verification  
    `RCSIM_deployment_tool/RCSIM_deployment_tool/core/deployment_logic.py` - log extraction  
    `RCSIM_RPI/RCSIM_rpi_embedded/docs/README_DOCKER.md` - detailed docker deployment notes  

---

## Support / Next Steps

After running deployment:
1. **Success?** → Great! The system is ready to use.
2. **Error?** → Copy the logs and review `docs/README_DOCKER.md`.
3. **Still not working?** → Send the logs for diagnosis.

**Good luck!** 🚀
