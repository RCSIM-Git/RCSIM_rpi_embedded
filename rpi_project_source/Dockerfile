# Base: Debian Bookworm Slim 
FROM arm64v8/debian:bookworm-slim

# Environment Variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app:/usr/lib/python3/dist-packages \
    MTX_VER=v1.9.0

# Install System Dependencies and Hailo RT
ENV CACHE_BUSTER=2024-03-24_v2
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg \
    wget \
    && wget -qO /etc/apt/trusted.gpg.d/raspi.asc http://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
    && echo "deb [trusted=yes] http://archive.raspberrypi.com/debian bookworm main" > /etc/apt/sources.list.d/raspi.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    libcap-dev \
    libcamera-tools \
    ffmpeg \
    libavcodec-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavutil-dev \
    libswresample-dev \
    libswscale-dev \
    libopus-dev \
    libvpx-dev \
    tar \
    git \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-numpy \
    python3-scipy \
    python3-opencv \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# Fix PEP 668 to allow global pip installs in Docker
RUN rm -f /usr/lib/python3.11/EXTERNALLY-MANAGED || true

# Inject manually downloaded HailoRT Python 3.11 Wheel for aarch64
# (Since the public WordPress hailo.ai download links expire, the user must provide the wheel)
COPY hailort*cp311*linux_aarch64.whl /tmp/

# Force HailoRT 4.23.0 (matches newer RPi kernels)
# First remove ghost 4.20.0 packages, install the 4.23 C++ library, then install the Python bindings
RUN dpkg -r --force-depends python3-hailort hailort || true && \
    wget -qO /tmp/hailort.deb http://archive.raspberrypi.com/debian/pool/main/h/hailort/hailort_4.23.0_arm64.deb && \
    dpkg -i /tmp/hailort.deb && rm /tmp/hailort.deb && \
    python3 -m pip install --break-system-packages /tmp/hailort*cp311*linux_aarch64.whl && \
    rm /tmp/hailort*cp311*linux_aarch64.whl

# Fix PEP 668 to allow global pip installs in Docker
RUN rm -f /usr/lib/python3.11/EXTERNALLY-MANAGED || true

RUN git clone --depth 1 https://github.com/simondlevy/BreezySLAM.git /tmp/BreezySLAM && \
    cd /tmp/BreezySLAM/python && \
    python3 -m pip install --break-system-packages --ignore-installed --no-cache-dir . && \
    rm -rf /tmp/BreezySLAM

WORKDIR /app

# Install MediaMTX
RUN wget -q https://github.com/bluenviron/mediamtx/releases/download/${MTX_VER}/mediamtx_${MTX_VER}_linux_arm64v8.tar.gz && \
    tar -xzf mediamtx_${MTX_VER}_linux_arm64v8.tar.gz && \
    mv mediamtx /usr/local/bin/ && \
    chmod +x /usr/local/bin/mediamtx && \
    rm mediamtx_${MTX_VER}_linux_arm64v8.tar.gz

# Install Python Requirements
COPY requirements.txt .
RUN python3 -m pip install --break-system-packages --ignore-installed --no-cache-dir -r requirements.txt --extra-index-url https://www.piwheels.org/simple

# Copy Application Code
COPY . .
RUN python3 -m pip install --break-system-packages pyserial pyyaml pynmeagps dacite smbus2 netifaces aiohttp aiortc psutil pymavlink
# Permissions & Line Endings
RUN sed -i 's/\r$//' entrypoint.sh tools/diagnostics/healthcheck.py && \
    chmod +x entrypoint.sh tools/diagnostics/healthcheck.py

# Entrypoint
ENTRYPOINT ["./entrypoint.sh"]
