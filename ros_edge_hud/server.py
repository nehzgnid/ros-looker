from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import psutil
import asyncio
import json
import ros_utils

app = FastAPI()

# HTML 模板直接嵌入，方便演示，实际可分离
html = """
<!DOCTYPE html>
<html>
    <head>
        <title>ROS-Edge-HUD Web</title>
        <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
        <style>
            body { background-color: #121212; color: #e0e0e0; }
            .card { background-color: #1e1e1e; border: 1px solid #333; margin-bottom: 20px; }
            .progress { height: 25px; background-color: #333; }
            .node-list { font-family: monospace; color: #00ff00; }
        </style>
    </head>
    <body>
        <div class="container mt-4">
            <h1 class="text-center mb-4">ROS-Edge-HUD Dashboard</h1>
            <div class="row">
                <div class="col-md-6">
                    <div class="card p-3">
                        <h3>System Status</h3>
                        <p>CPU Usage: <span id="cpu-val">0</span>%</p>
                        <div class="progress mb-3">
                            <div id="cpu-bar" class="progress-bar bg-info" role="progressbar" style="width: 0%"></div>
                        </div>
                        <p>Memory Usage: <span id="mem-val">0</span>%</p>
                        <div class="progress">
                            <div id="mem-bar" class="progress-bar bg-warning" role="progressbar" style="width: 0%"></div>
                        </div>
                    </div>
                </div>
                <div class="col-md-6">
                    <div class="card p-3">
                        <h3>ROS Nodes</h3>
                        <div id="ros-nodes" class="node-list">Wait for data...</div>
                    </div>
                </div>
            </div>
            <div class="row">
                <div class="col-12">
                    <div class="card p-3">
                        <h3>Top Processes</h3>
                        <table class="table table-dark table-striped">
                            <thead>
                                <tr><th>PID</th><th>Name</th><th>CPU %</th></tr>
                            </thead>
                            <tbody id="proc-table"></tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
        <script>
            const ws = new WebSocket(`ws://${window.location.host}/ws`);
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                
                // Update CPU
                document.getElementById('cpu-val').innerText = data.cpu;
                document.getElementById('cpu-bar').style.width = data.cpu + '%';
                
                // Update Mem
                document.getElementById('mem-val').innerText = data.mem;
                document.getElementById('mem-bar').style.width = data.mem + '%';
                
                // Update ROS Nodes
                const nodeDiv = document.getElementById('ros-nodes');
                nodeDiv.innerHTML = data.ros_nodes.map(n => `<div>> ${n}</div>`).join('');
                
                // Update Processes
                const procBody = document.getElementById('proc-table');
                procBody.innerHTML = data.processes.map(p => 
                    `<tr><td>${p.pid}</td><td>${p.name}</td><td>${p.cpu}</td></tr>`
                ).join('');
            };
        </script>
    </body>
</html>
"""

@app.get("/")
async def get():
    return HTMLResponse(html)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        # 获取系统数据
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        ros_nodes = ros_utils.get_ros_nodes()
        
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
            try:
                processes.append({'pid': proc.info['pid'], 'name': proc.info['name'], 'cpu': proc.info['cpu_percent']})
            except: pass
        
        processes.sort(key=lambda x: x['cpu'], reverse=True)
        
        data = {
            "cpu": cpu,
            "mem": mem,
            "ros_nodes": ros_nodes,
            "processes": processes[:10]
        }
        await websocket.send_text(json.dumps(data))
        await asyncio.sleep(2)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
