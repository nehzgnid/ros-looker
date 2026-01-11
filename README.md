# ROS-Edge-Commander (Watcher)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![C++](https://img.shields.io/badge/C++-17-green)](https://en.cppreference.com/w/cpp/17)

**ROS-Edge-Commander** 是一款专为机器人边缘计算设备（如 Intel N100, Jetson Nano, Raspberry Pi）设计的轻量级、高性能终端监控与管控工具。

它旨在解决低算力设备上运行图形化工具（如 RQt）资源占用过高的问题，提供纯终端（TUI）的交互体验，涵盖系统资源监控、进程管理、ROS 节点管控以及自动运维功能。

---

## 🚀 核心功能 (Key Features)

### 1. 仪表盘 (Dashboard)
- **实时监控**：CPU、内存、磁盘 (根目录)、CPU 温度。
- **数值可视化**：进度条上方直接显示具体百分比数值。
- **历史曲线**：CPU 负载的 60秒 历史波形图。
- **环境感知**：实时显示 ROS Master 在线状态。

### 2. 智能内存管理 (Auto-Free Memory)
- **自动释放**：当内存使用率超过设定阈值（默认 85%）且持续高负载时，自动清理系统缓存 (`drop_caches`)。
- **动态阈值**：用户可在运行时实时调整触发阈值。
- **权限安全**：自动检测 `sudo` 权限，防止无权操作报错。

### 3. 网络监控 (Network)
- **流量统计**：实时显示上传/下载速率 (KB/s)。
- **流量曲线**：独立的 RX (接收) 和 TX (发送) 历史波形图。

### 4. ROS 管控中心
- **工作空间侦探**：自动扫描当前系统所有 Shell 进程，识别并列出所有已 Source 的 ROS 工作空间。
- **节点管理**：列出所有活跃节点，支持一键 Kill 僵死节点。

### 5. 进程管理
- **资源排序**：按 CPU 使用率降序排列系统进程。
- **快捷操作**：选中进程一键终止 (SIGTERM)。

---

## ⌨️ 快捷键 (Shortcuts)

| 按键 | 功能 | 说明 |
| :--- | :--- | :--- |
| **Left / Right** | 切换标签页 | 在 Dashboard / Network / Processes / ROS 之间切换 |
| **Up / Down** | 列表导航 | 在进程列表或 ROS 节点列表中移动光标 |
| **k** | Kill 进程/节点 | 终止当前选中的进程或 ROS 节点 |
| **m** | 手动释放内存 | 强制执行 `sync; echo 3 > /proc/sys/vm/drop_caches` (需 sudo) |
| **[** | 降低阈值 | 将内存自动释放阈值降低 5% |
| **]** | 升高阈值 | 将内存自动释放阈值升高 5% |
| **q** | 退出 | 关闭程序 |

---

## 🛠️ 编译与运行 (Build & Run)

本项目依赖 `FTXUI` (会自动下载) 和 Linux 系统库。

### 1. 安装依赖
```bash
sudo apt-get install cmake build-essential git
```

### 2. 编译
```bash
cd cpp_watcher
mkdir -p build && cd build
cmake ..
make -j4
```

### 3. 运行
建议使用 `sudo` 运行以获得完整的内存管理和进程控制权限：

```bash
sudo ./watcher
```
*(如果不加 sudo，只能查看信息，无法释放内存或杀掉属于其他用户的进程)*

---

## 📂 目录结构

*   `cpp_watcher/`: C++ 核心源码 (高性能生产环境推荐)
    *   `src/main.cpp`: 单文件核心逻辑
*   `ros_edge_hud/`: Python 原型 (仅供参考/Web扩展)

---

## 📝 开发计划
- [x] 基础资源监控 (CPU/RAM/Temp)
- [x] ROS 节点列表与管控
- [x] 自动内存释放策略
- [ ] ROS Topic 频率监控 (`rostopic hz`)
- [ ] 电池信息集成 (`/battery_state`)