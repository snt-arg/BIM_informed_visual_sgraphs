# 🚀 Run ivS-Graphs using Docker

## ✅ I. Set Up NVIDIA Container Toolkit (Ubuntu)

You might need to install `Nvidia`'s container toolkit to

```bash
# Detect your Ubuntu distribution
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

# Add the NVIDIA GPG key
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add the NVIDIA container repository
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure the Docker runtime (inside docker folder)
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker
sudo systemctl restart docker
```

## ⚙️ II. Build

To build the Docker image, run the following command within this directory:

```bash
# Set your username (used inside the container)
export USERNAME=$(whoami)

# Build using Docker Compose (recommended)
docker compose build

# Or build directly
docker build -t ivsgraphs -f Jazzy.Dockerfile --build-arg USERNAME=$USERNAME .
```

## 🚀 III. Run

### III-A. Run the Docker Image

Use one of the below options:

```bash
# [Option I] using Docker
docker run -it -d --privileged --name ivsgraphs -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v /tmp/.X11-unix:/tmp/.X11-unix -v $XAUTHORITY:$XAUTHORITY ivsgraphs

# [Option II] using Docker Compose
docker compose up -d

# [Option III] using devcontainers
# in `VSCode`, select "Reopen in Container"
```

> 🛎️ Tip: If you use **Docker Compose**, make sure to properly configure `Working Directories` (maps your project folder from the host to the container to ensure code and configuration changes persist) and `Data Directories` (mount the folder containing your datasets (e.g., ROS bags) so they are accessible and runnable inside the container).

### III-B. Run the Container

```bash
docker exec -it ivsgraphs bash

# Inside the container
ros2 launch vs_graphs vs_graphs.launch.py
```
