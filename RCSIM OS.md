# 🍓 Creating a Custom RCSIM OS Image

To create your own customized operating system image (**Custom OS Image / RCSIM OS**) based on the code and configurations in the `/RCSIM_RPI` directory, you need to prepare the system on an SD card, deploy the RCSIM containerized application, and then dump and compress the filesystem into an `.img` file.

Below is a complete step-by-step guide to achieving this.

---

### STEP 1: Preparing the Base Operating System
The RCSIM deployment application for Raspberry Pi 5 runs containerized via Docker on a Debian-based Raspberry Pi OS (64-bit).

1. Download and install **Raspberry Pi Imager** on your PC.
2. Insert a clean SD card into your card reader.
3. In Raspberry Pi Imager, select:
   * **Device:** Raspberry Pi 5
   * **Operating System:** Raspberry Pi OS (64-bit) (Bookworm release or the latest version compatible with Python 3.13)
4. Click the gear icon (Advanced Settings) and configure:
   * **Host Name** (e.g., `rcsim`)
   * **Enable SSH** (use password authentication or public key authorization)
   * **User and Password** (e.g., user: `pi`)
   * **Wireless LAN (WiFi)** configuration (SSID and password so the device automatically connects to your network on boot)
5. Click **WRITE** and wait for the process to complete.

---

### STEP 2: Deploying the RCSIM Code to Raspberry Pi
Next, you need to upload and run the code from your development directory `RCSIM_RPI\RCSIM_rpi_embedded\rpi_project_source` on the running Raspberry Pi. You can do this in two ways:

#### Option A: Automatically via the RCSIM Deployment Tool (Recommended)
1. Run the **RCSIM RPi5 Deployment Tool** on your PC (launch the script `RCsimRPi5deploymentapp.py` or run the compiled `.exe` file from the `RCSIM_deployment_tool` folder).
2. In the **Source Directory** field, specify the path to the RPi source files:
   `RCSIM_RPI\RCSIM_rpi_embedded\rpi_project_source`
3. Enter the IP address assigned to your Raspberry Pi by your router and your SSH login credentials.
4. Click **START DEPLOYMENT**. The tool will automatically:
   * Install Docker and Docker Compose on the Raspberry Pi.
   * Copy the source code, systemd service files, and hardware configuration.
   * Pull and build the container based on the project's `Dockerfile`.
   * Configure the container to auto-start on system boot.

#### Option B: Manually via Terminal (SSH)
1. Connect to the Raspberry Pi over SSH:
   ```bash
   ssh pi@<RASPBERRY_PI_IP>
   ```
2. Install Docker:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh && sh get-docker.sh
   sudo usermod -aG docker $USER
   ```
3. Copy the contents of the `rpi_project_source` folder from your PC to the `~/rpi_project_source` directory on the Raspberry Pi (e.g., using SFTP / FileZilla).
4. Navigate to the directory on the Raspberry Pi and build the containers:
   ```bash
   cd ~/rpi_project_source
   docker compose up --build -d
   ```

---

### STEP 3: Enabling Hardware Overlays
RCSIM requires direct access to the I2C and UART buses, the CSI camera interface, and the Hailo-8 NPU accelerator. Ensure the correct hardware device tree overlays are enabled in `/boot/firmware/config.txt` on the Raspberry Pi:
```ini
dtparam=i2c_arm=on
dtparam=uart0=on
dtoverlay=hailo-ctl
```

---

### STEP 4: Dumping the Configured System to an `.img` File (Cloning)
Once the system on the Raspberry Pi runs stably, the containers auto-start on boot, and all diagnostics pass, you can clone the SD card to create a custom installation image.

1. Gracefully power off the Raspberry Pi:
   ```bash
   sudo poweroff
   ```
2. Remove the SD card from the Raspberry Pi and insert it back into your PC.
3. Open **Win32 Disk Imager** (on Windows):
   * Select your SD card drive letter in the **Device** dropdown.
   * Under **Image File**, select the folder icon, choose a destination directory, and enter a filename (e.g., `RCSIM_OS.img`).
   * Click the **Read** button. The program will clone the entire partition layout of the SD card into a single `.img` file on your PC.

---

### STEP 5: Shrinking the Image Size (Optional - PiShrink)
The raw image dumped from the SD card will have the exact same size as the physical card capacity (e.g., 32GB or 64GB), even if most of the disk space is empty. To reduce the `.img` file size to the minimum required data size (usually 4-6GB), use **PiShrink** (requires a Linux environment or WSL on Windows):

1. Install PiShrink on your Linux/WSL instance:
   ```bash
   wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
   chmod +x pishrink.sh
   sudo mv pishrink.sh /usr/local/bin/
   ```
2. Run the image optimization command:
   ```bash
   sudo pishrink.sh -s RCSIM_OS.img
   ```
The tool will remove unused filesystem sectors and set the auto-resize flag so that the partition automatically expands to fill the card's capacity when booted on a new Raspberry Pi.

---

### STEP 6: Distribution and Flashing
Your custom operating system image is now ready for deployment. You can flash it onto any SD card:

1. Open **Raspberry Pi Imager**.
2. Select **Use Custom** as the operating system and choose the created `RCSIM_OS.img` file.
3. Select your target SD card and click **WRITE**.
4. Once flashed, place your custom config files in the `config` directory on the boot partition.
