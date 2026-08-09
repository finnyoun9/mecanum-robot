# ROS2 via Docker (Raspberry Pi OS host)

The Pi runs its stock **Raspberry Pi OS (Debian 12 Bookworm)** — not
Ubuntu — so ROS2's official apt packages aren't installable directly on
the host. Instead, ROS2 Jazzy runs inside an **Ubuntu 24.04 container**.

This is not a VM: the container shares the host's Linux kernel (just a
different userspace), and since the Pi is already arm64, the container
runs at native speed — no QEMU emulation involved.

## Why not just containerize everything, including the camera?

The Pi's camera is CSI-ribbon (libcamera / `rp1-cfe` driver), not a plain
USB webcam. libcamera's userspace stack is tightly coupled to the host
kernel driver version, which makes it a genuine pain to pass into a
container cleanly. Since none of `perception/detection/`,
`perception/camera/`, or `perception/laser_triangulation/` need to run
inside ROS2 at all (they're plain OpenCV scripts), the camera stays on
the host and those scripts run there natively. Only the pieces that
actually need ROS2 — `ros2_control`, SLAM, Nav2, the EKF, and the serial
bridge to the STM32 — run in the container.

## Usage

```bash
# First time (also builds the image automatically if missing):
./docker/run.sh --build

# Get a shell in the container (auto-builds image on first run):
./docker/run.sh

# Inside the container:
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mcr_bringup robot.launch.py

# One-off command instead of an interactive shell:
./docker/run.sh colcon build --symlink-install
```

`ros2_ws/` on the host is bind-mounted into the container at `/ros2_ws`,
so edits (e.g. `git pull` on the Pi) are picked up immediately — no image
rebuild needed unless you change the Dockerfile itself (i.e. add/remove
apt packages).

## Devices

`docker/run.sh` probes for `/dev/ttyAMA10` (Pi 5's hardware UART — the
STM32 link, once wired), `/dev/ttyUSB0` (LD06 LiDAR, once connected via
its USB-serial adapter), and `/dev/ttyACM0` (fallback if the STM32 board
uses USB-CDC serial directly). Only devices that actually exist at
startup get passed through, so the script won't fail before your
hardware is wired up — it just runs without that device until you
re-launch.

## Networking

`--network host` is used so ROS2's DDS discovery (UDP multicast) works
exactly as it would on a native install — no port mapping or bridging
needed between the container and the Pi's network.
