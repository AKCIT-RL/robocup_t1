#!/usr/bin/env python3
"""
BehaviorTree Web Monitor Node.
FastAPI + WebSocket server that bridges Groot2Publisher to web clients.
"""

import rclpy
from rclpy.node import Node
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
import uvicorn
import asyncio
import threading
import queue
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Set, Dict
from ament_index_python.packages import get_package_share_directory

from bt_webmonitor.zmq_client import Groot2Client


class BtWebMonitor(Node):
    """ROS2 node hosting web-based BehaviorTree monitor."""
    
    def __init__(self):
        super().__init__('bt_webmonitor')
        
        # Declare parameters
        self.declare_parameter('webui_port', 8080)
        self.declare_parameter('zmq_host', 'localhost')
        self.declare_parameter('zmq_publisher_port', 1666)
        self.declare_parameter('zmq_server_port', 1667)
        self.declare_parameter('update_rate', 10.0)
        
        # Get parameters
        self.webui_port = self.get_parameter('webui_port').value
        self.zmq_host = self.get_parameter('zmq_host').value
        self.zmq_publisher_port = self.get_parameter('zmq_publisher_port').value
        self.zmq_server_port = self.get_parameter('zmq_server_port').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # FastAPI app
        self.app = FastAPI(title="BT Web Monitor")
        
        # WebSocket clients
        self.websocket_clients: Set[WebSocket] = set()
        
        # Thread-safe message queue for broadcasting
        # Fixes the asyncio.run() bug by using a queue consumed by an async task
        self._message_queue: queue.Queue = queue.Queue()
        
        # Latest status batch for efficient broadcasting
        self._status_batch: Dict[int, str] = {}
        self._status_batch_lock = threading.Lock()
        
        # ZMQ client
        self.zmq_client = Groot2Client(
            zmq_host=self.zmq_host,
            zmq_publisher_port=self.zmq_publisher_port,
            zmq_server_port=self.zmq_server_port,
            logger=self.get_logger()
        )
        
        # Setup callbacks
        self.zmq_client.on_tree_update = self._on_tree_update
        self.zmq_client.on_status_update = self._on_status_update
        
        # Setup routes
        self._setup_routes()
        
        # Start ZMQ client
        if self.zmq_client.connect():
            self.zmq_client.start_monitoring()
        
        # Start web server in background thread
        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=True
        )
        self.server_thread.start()
        
        self.get_logger().info(
            f"BT Web Monitor started at http://localhost:{self.webui_port}"
        )
        self.get_logger().info(
            f"Connecting to Groot2Publisher at {self.zmq_host}:{self.zmq_server_port}"
        )
    
    def _get_static_dir(self):
        """Find static files directory (works both in development and after installation)."""
        # Try installed location first
        try:
            pkg_share = get_package_share_directory('bt_webmonitor')
            static_dir = Path(pkg_share) / 'static'
            if static_dir.exists():
                return static_dir
        except Exception:
            pass
        
        # Try development location (next to this file)
        static_dir = Path(__file__).parent / 'static'
        if static_dir.exists():
            return static_dir
        
        # Last resort: check in src directory
        src_static = Path(__file__).parent.parent.parent / 'bt_webmonitor' / 'bt_webmonitor' / 'static'
        if src_static.exists():
            return src_static
        
        return None
    
    def _setup_routes(self):
        """Setup FastAPI routes."""
        
        # Allowed static filenames (whitelist for security)
        ALLOWED_STATIC_FILES = {
            'index.html', 'styles.css', 'bt_visualizer.js'
        }
        
        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            """Serve main page."""
            static_dir = self._get_static_dir()
            
            if static_dir:
                index_file = static_dir / 'index.html'
                if index_file.exists():
                    return HTMLResponse(content=index_file.read_text(), status_code=200)
            
            return HTMLResponse(
                content="<h1>BT Monitor</h1><p>Static files not found.</p>",
                status_code=200
            )
        
        @self.app.get("/static/{filename}")
        async def serve_static(filename: str):
            """Serve static files (JS, CSS) with no-cache headers."""
            # Whitelist validation prevents path traversal (no slashes possible)
            if filename not in ALLOWED_STATIC_FILES:
                self.get_logger().warn(f"Static file not in whitelist: {filename}")
                return HTMLResponse(content="Not found", status_code=404)
            
            static_dir = self._get_static_dir()
            
            if static_dir:
                file_path = static_dir / filename
                # Follow symlinks (needed for --symlink-install)
                if file_path.exists():
                    # Set explicit media types for correct Content-Type
                    media_types = {
                        '.css': 'text/css',
                        '.js': 'application/javascript',
                        '.html': 'text/html',
                    }
                    suffix = file_path.suffix.lower()
                    media_type = media_types.get(suffix)
                    
                    self.get_logger().debug(f"Serving static: {file_path} (type={media_type})")
                    return FileResponse(
                        str(file_path),
                        media_type=media_type,
                        headers={
                            "Cache-Control": "no-cache, no-store, must-revalidate",
                            "Pragma": "no-cache",
                            "Expires": "0"
                        }
                    )
                else:
                    self.get_logger().error(f"Static file not found: {file_path}")
            else:
                self.get_logger().error("Static directory not found!")
            
            return HTMLResponse(content="Not found", status_code=404)
        
        @self.app.get("/api/blackboard")
        async def get_blackboard():
            """Return current blackboard values."""
            bb_data = self.zmq_client.get_blackboard_data()
            return {"blackboard": bb_data}
        
        @self.app.websocket("/ws/bt_monitor")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket endpoint for BT updates."""
            await websocket.accept()
            self.websocket_clients.add(websocket)
            
            try:
                # Send initial tree structure
                tree_data = self.zmq_client.get_tree_structure()
                await websocket.send_json({
                    "type": "tree_structure",
                    "data": tree_data
                })
                
                # Send initial statuses as a batch
                statuses = self.zmq_client.get_all_statuses()
                if statuses:
                    batch = [{"node_id": nid, "status": s} for nid, s in statuses.items()]
                    await websocket.send_json({
                        "type": "status_batch",
                        "data": batch
                    })
                
                # Keep connection alive and handle client messages
                while True:
                    try:
                        data = await asyncio.wait_for(
                            websocket.receive_text(),
                            timeout=1.0
                        )
                        # Handle client requests if needed
                    except asyncio.TimeoutError:
                        # Send heartbeat
                        await websocket.send_json({"type": "heartbeat"})
                    
            except WebSocketDisconnect:
                self.get_logger().info("WebSocket client disconnected")
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")
            finally:
                self.websocket_clients.discard(websocket)
    
    def _on_tree_update(self, tree_data: dict):
        """Called when tree structure is received (from ZMQ thread)."""
        self.get_logger().info("Received tree structure update")
        # Enqueue message for async broadcast (thread-safe)
        self._message_queue.put({
            "type": "tree_structure",
            "data": tree_data
        })
    
    def _on_status_update(self, status_data: dict):
        """Called when node status changes (from ZMQ thread)."""
        # Accumulate into batch for efficient broadcasting
        with self._status_batch_lock:
            node_id = status_data.get('node_id')
            status = status_data.get('status')
            if node_id is not None and status is not None:
                self._status_batch[node_id] = status
    
    def _flush_status_batch(self):
        """Flush accumulated status updates as a single batch message."""
        with self._status_batch_lock:
            if not self._status_batch:
                return
            batch = [{"node_id": nid, "status": s} for nid, s in self._status_batch.items()]
            self._status_batch.clear()
        
        self._message_queue.put({
            "type": "status_batch",
            "data": batch
        })
    
    async def _broadcast_worker(self):
        """Async worker that processes the message queue and broadcasts to WebSocket clients."""
        while True:
            # Flush status batch periodically (every 100ms = 10Hz)
            self._flush_status_batch()
            
            # Process all pending messages
            messages_to_send = []
            while True:
                try:
                    msg = self._message_queue.get_nowait()
                    messages_to_send.append(msg)
                except queue.Empty:
                    break
            
            # Broadcast messages
            for message in messages_to_send:
                if not self.websocket_clients:
                    continue
                
                disconnected = set()
                for client in self.websocket_clients.copy():
                    try:
                        await client.send_json(message)
                    except Exception:
                        disconnected.add(client)
                
                self.websocket_clients -= disconnected
            
            await asyncio.sleep(0.1)  # 10 Hz broadcast rate
    
    def _run_server(self):
        """Run FastAPI server (called in background thread)."""
        # Use lifespan context manager (replaces deprecated on_event('startup'))
        @asynccontextmanager
        async def lifespan(app):
            # Startup: launch broadcast worker
            task = asyncio.create_task(self._broadcast_worker())
            yield
            # Shutdown: cancel worker
            task.cancel()
        
        self.app.router.lifespan_context = lifespan
        
        uvicorn.run(
            self.app,
            host="0.0.0.0",
            port=self.webui_port,
            log_level="warning"
        )
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.zmq_client.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    monitor = BtWebMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
