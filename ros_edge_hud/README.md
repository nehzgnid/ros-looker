# ROS-Edge-HUD

一个现代化的 Linux 与 ROS 系统实时监控工具，专为边缘计算设备（如 Intel N100）设计。

## 功能
- **命令行界面 (TUI)**: 极低资源占用，支持鼠标操作，展示 CPU/内存/网络/进程及 ROS 节点。
- **网页界面 (WebUI)**: 响应式设计，适合在手机或平板上远程监控。

## 安装依赖
确保已安装 Python 3.8+，然后在项目目录下运行：
```bash
pip install -r requirements.txt
```

## 使用方法

### 1. 启动命令行界面 (TUI)
```bash
python app.py
```
- 按 `q` 退出。
- 按 `d` 切换深色/浅色模式。

### 2. 启动网页界面 (WebUI)
```bash
python server.py
```
启动后在浏览器访问：`http://localhost:8000` (或设备 IP)。

## 注意事项
- 本工具会自动检测 ROS 环境，如果未安装 ROS，则会显示相应提示。
- 在 Linux 系统下运行时，可以获取更准确的温度和工作空间信息。
