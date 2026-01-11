from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Static, DataTable, Label
from textual.containers import Container, Horizontal, Vertical, Grid
from textual.reactive import reactive
import psutil
import platform
import datetime
import ros_utils

class SystemInfo(Static):
    """显示系统基础信息的组件"""
    def on_mount(self) -> None:
        self.set_interval(1, self.update_info)

    def update_info(self) -> None:
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        net = psutil.net_io_counters()
        sent = net.bytes_sent / 1024 / 1024
        recv = net.bytes_recv / 1024 / 1024
        
        self.update(
            f"[bold cyan]CPU:[/bold cyan] {cpu:4.1f}% | "
            f"[bold magenta]RAM:[/bold magenta] {mem:4.1f}% | "
            f"[bold yellow]NET UP:[/bold yellow] {sent:6.2f}MB | "
            f"[bold green]DOWN:[/bold green] {recv:6.2f}MB"
        )

class ROSInfo(Static):
    """显示 ROS 节点状态的组件"""
    def on_mount(self) -> None:
        self.set_interval(5, self.update_ros)

    def update_ros(self) -> None:
        nodes = ros_utils.get_ros_nodes()
        node_count = len(nodes) if isinstance(nodes, list) else 0
        node_list = "\n".join(nodes[:5]) + ("\n..." if len(nodes) > 5 else "")
        self.update(f"[bold]ROS Nodes ({node_count}):[/bold]\n{node_list}")

class RoseEdgeHudApp(App):
    """ROS-Edge-HUD 主程序"""
    CSS = """
    Screen {
        layout: vertical;
    }
    #top-row {
        height: 3;
        background: $boost;
        border-bottom: solid $accent;
    }
    #main-content {
        height: 1fr;
        layout: grid;
        grid-size: 2;
        grid-columns: 1fr 2fr;
    }
    .panel {
        border: round $primary;
        padding: 1;
        margin: 1;
    }
    #process-panel {
        height: 1fr;
    }
    DataTable {
        height: 1fr;
    }
    """
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("d", "toggle_dark", "Toggle dark mode"),
        ("r", "refresh", "Refresh ROS"),
    ]

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="top-row"):
            yield SystemInfo()
        with Container(id="main-content"):
            with Vertical(classes="panel"):
                yield ROSInfo(id="ros-panel")
                yield Label("\n[bold]Workspace:[/bold]")
                yield Static(ros_utils.get_workspace_info(), id="ws-info")
            with Vertical(classes="panel", id="process-panel"):
                yield Label("[bold]Top Processes[/bold]")
                yield DataTable(id="process-table")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one(DataTable)
        table.add_columns("PID", "Name", "CPU %", "MEM %")
        table.cursor_type = "row"
        self.set_interval(3, self.update_processes)

    def update_processes(self) -> None:
        table = self.query_one(DataTable)
        # 记录当前光标位置
        current_cursor = table.cursor_row
        
        processes = []
        # 限制遍历数量或仅获取必要信息虽不能显著减少 process_iter 开销，
        # 但减少 append 和后续 sort 的数据量有微小帮助。
        # 关键是减少 UI 渲染压力，只取前 15 个显示。
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                p_info = proc.info
                # 过滤掉 CPU 占用极低的进程，减少排序列表大小
                if p_info['cpu_percent'] > 0.1 or p_info['memory_percent'] > 0.1:
                     processes.append(p_info)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        processes.sort(key=lambda x: x['cpu_percent'], reverse=True)
        
        table.clear()
        # 仅显示前 15 个进程，减少 UI 绘制负担
        for proc in processes[:15]:
            table.add_row(
                str(proc['pid']),
                proc['name'],
                f"{proc['cpu_percent']:.1f}",
                f"{proc['memory_percent']:.1f}"
            )
            
        # 恢复光标位置
        if current_cursor is not None and current_cursor < table.row_count:
            table.move_cursor(row=current_cursor)

if __name__ == "__main__":
    app = RoseEdgeHudApp()
    app.run()
