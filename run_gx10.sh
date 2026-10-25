#!/usr/bin/env bash
# ==============================================================================
# Robust Docker Launcher for ASUS GX10 Base Station
# Ensures ALL terminals share the exact same container, memory, and ROS 2 network.
# ==============================================================================

CONTAINER_NAME="robot_base"

# Check if the container is already running
if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[GX10] Joining existing container '${CONTAINER_NAME}'..."
    sudo docker exec -it ${CONTAINER_NAME} bash
else
    echo "[GX10] Cleaning up any stopped container named '${CONTAINER_NAME}'..."
    sudo docker rm -f ${CONTAINER_NAME} 2>/dev/null || true

    echo "[GX10] Starting '${CONTAINER_NAME}' with host networking and shared IPC..."
    sudo docker run -it \
        --name ${CONTAINER_NAME} \
        --net=host \
        --ipc=host \
        -v /home/asus/calhacks:/workspace \
        -w /workspace \
        ros:humble-robot \
        bash -c "grep -q 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc; exec bash"
fi
