import os
import subprocess

def get_ros_nodes():
    """获取活跃的 ROS 节点列表"""
    try:
        # 尝试调用 rosnode list
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            return result.stdout.strip().split('\n')
        return []
    except Exception:
        return ["ROS not found or master not running"]

def get_ros_topics():
    """获取活跃的 ROS 话题列表"""
    try:
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            return result.stdout.strip().split('\n')
        return []
    except Exception:
        return []

def get_workspace_info():
    """检测当前终端可能关联的 ROS 工作空间"""
    # 在 Linux 下可以通过查看环境变量 ROS_PACKAGE_PATH
    ws = os.environ.get('ROS_PACKAGE_PATH', 'Unknown')
    return ws
