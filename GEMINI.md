# ROS-Edge-HUD / Watcher Context

## Project Overview

**ROS-Edge-HUD** (also referred to as **Watcher**) is a lightweight, modern system monitoring and control tool designed for edge computing devices in robotics (specifically targeting **Intel N100** running Ubuntu 20.04/ROS).

It addresses the resource constraints of running heavy GUI tools (like RQt) on edge hardware by providing:
1.  **Low-overhead TUI (Terminal UI)** for local or SSH-based monitoring.
2.  **WebUI** for remote monitoring via mobile/tablet.
3.  **ROS Integration** to monitor nodes, topics, and workspaces without sourcing environments manually.

The project has two distinct implementations:
*   **Python Version (`ros_edge_hud`)**: Quick prototyping, rich UI (Textual), Web capabilities (FastAPI).
*   **C++ Version (`cpp_watcher`)**: High performance, minimal resource footprint, direct Linux kernel (`/proc`) parsing.

## Directory Structure

*   `ros_edge_hud/`: Python implementation.
    *   `app.py`: Main TUI application using `textual`.
    *   `server.py`: Web backend using `fastapi` + `uvicorn` (WebSockets).
    *   `ros_utils.py`: Shared ROS interaction logic.
    *   `requirements.txt`: Python dependencies.
*   `cpp_watcher/`: C++ implementation (Performance focused).
    *   `src/main.cpp`: Single-file core logic (TUI + System Monitor + ROS Monitor).
    *   `CMakeLists.txt`: Build configuration (fetches FTXUI v5.0.0 automatically).
*   `ai初步分析.md`: Original requirement analysis and architectural design document.
*   `setup_ssh.py`: Utility script to generate and deploy SSH keys to the target server for passwordless `scp`/`ssh`.
*   `sync.py`: Utility to synchronize code to the remote server.

## Building and Running

### Python Version (`ros_edge_hud`)

**Prerequisites:** Python 3.8+

1.  **Install Dependencies:**
    ```bash
    cd ros_edge_hud
    pip install -r requirements.txt
    ```

2.  **Run TUI:**
    ```bash
    python app.py
    ```

3.  **Run WebUI:**
    ```bash
    python server.py
    ```
    Access at `http://localhost:8000`.

### C++ Version (`cpp_watcher`)

**Prerequisites:** CMake, Make, GCC/Clang (C++17 support).

1.  **Build:**
    ```bash
    cd cpp_watcher
    mkdir build && cd build
    cmake ..
    make -j4
    ```

2.  **Run:**
    ```bash
    ./watcher
    ```

## Development Conventions

*   **Platform:** Targeted for **Linux** (Ubuntu 20.04). Code reading `/proc` or `sysinfo` will **not run on Windows** locally.
*   **Remote Development:** The workflow assumes coding on Windows and deploying to a Linux server (`10.200.98.1`) via `scp`.
*   **C++ Style:**
    *   Uses **FTXUI** for the interface.
    *   **Explicit Initialization**: Avoid implicit `std::initializer_list` conversions for FTXUI containers (use `Elements{...}` or `Components{...}`) to ensure compiler compatibility.
    *   **Direct System Calls**: Prefers reading `/proc/stat`, `/proc/meminfo` and using `popen` for shell commands over heavy libraries.
*   **ROS Integration:**
    *   Uses `rosnode list`, `rostopic list` via shell execution.
    *   Monitors `ROS_PACKAGE_PATH` to detect workspaces.
*   **Interaction:**
    *   Arrow keys for navigation.
    *   `k` key to kill processes/nodes.
    *   `q` key to quit.

## Key Features

*   **Dashboard**: CPU (Sparkline/History), RAM, Network I/O.
*   **Process Manager**: Top processes by CPU/Mem, sortable, killable (`SIGTERM`).
*   **ROS Monitor**: Active nodes list, Workspace detection, killable nodes (`rosnode kill`).
