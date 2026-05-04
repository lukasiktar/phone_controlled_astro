"""
ROS2 node for controlling the Astro robot using a phone. 

Listens on HTTP for:
  POST /go          { "linear": 1.2, "angular": 0.5 }  → start moving
  POST /stop        {}                                  → stop immediately
  POST /speed       { "linear": 1.2, "angular": 0.5 }  → update speeds while moving
  GET  /state       → current state (moving/stopped, speeds)
  GET  /health      → ok

When GO is active, publishes Twist at `publish_rate` Hz using the last
received linear/angular speeds. STOP zeroes the Twist.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

#  HTTP handler                                                                

class SliderHTTPHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        self.server.node.get_logger().debug(f"[HTTP] {fmt % args}")

    def _send_json(self, code: int, payload: dict):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type",                "application/json")
        self.send_header("Content-Length",              str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods","GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers","Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length)) if length else {}

    def do_OPTIONS(self):
        self._send_json(200, {})

    # GET 
    def do_GET(self):
        node: VelocityControllerNode = self.server.node

        if self.path == "/health":
            self._send_json(200, {"ok": True})

        elif self.path == "/state":
            with node.lock:
                self._send_json(200, {
                    "moving":   node.moving,
                    "linear":   node.linear_speed,
                    "angular":  node.angular_speed,
                    "uptime_s": round(time.time() - node.start_time, 1),
                })
        else:
            self._send_json(404, {"error": "Not found"})

    # POST 
    def do_POST(self):
        node: VelocityControllerNode = self.server.node

        # ── /go
        if self.path == "/go":
            try:
                data = self._read_json()
                lin = float(data.get("linear",  node.linear_speed))
                ang = float(data.get("angular", node.angular_speed))
                node.go(lin, ang)
                self._send_json(200, {"ok": True, "moving": True,
                                      "linear": lin, "angular": ang})
            except Exception as e:
                self._send_json(400, {"error": str(e)})

        # ── /stop 
        elif self.path == "/stop":
            node.stop()
            self._send_json(200, {"ok": True, "moving": False})

        # ── /speed  (update speeds without changing moving state)
        elif self.path == "/speed":
            try:
                data = self._read_json()
                lin = float(data.get("linear",  node.linear_speed))
                ang = float(data.get("angular", node.angular_speed))
                node.set_speed(lin, ang)
                self._send_json(200, {"ok": True,
                                      "linear": lin, "angular": ang,
                                      "moving": node.moving})
            except Exception as e:
                self._send_json(400, {"error": str(e)})

        else:
            self._send_json(404, {"error": "Not found"})


class VelocityControllerNode(Node):

    def __init__(self):
        super().__init__("velocity_controller")

        self.pub = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self.get_logger().info("Publisher created on /cmd_vel_joy")

        # State
        self.lock = threading.Lock()
        self.moving = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.start_time = time.time()

        # Publish timer
        rate = 10.0  # Hz
        self.create_timer(1.0 / rate, self.publish_velocity)

        # HTTP server in separate thread
        host = "0.0.0.0"
        port = 8080
        server = HTTPServer((host, port), SliderHTTPHandler)
        server.node = self  # type: ignore
        self._http_server = server
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"HTTP server on http://{host}:{port}  "
            f"(POST /go  POST /stop  POST /speed  GET /state)"
        )


    def go(self, linear: float, angular: float):
        """Start moving at the given speeds."""
        linear, angular = self._clamp(linear, angular)
        with self.lock:
            self.linear_speed  = linear
            self.angular_speed = angular
            self.moving        = True
        self.get_logger().info(
            f"GO  linear={linear:.3f}  angular={angular:.3f}"
        )

    def stop(self):
        """Stop moving — will publish a zero Twist on next tick."""
        with self.lock:
            self.moving = False
        msg = Twist()
        msg.linear.x  = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info("STOP")


    def set_speed(self, linear: float, angular: float):
        """Update target speeds without changing moving state."""
        linear, angular = self._clamp(linear, angular)
        with self.lock:
            self.linear_speed  = linear
            self.angular_speed = angular        
        self.get_logger().debug(
            f"Speed update  linear={linear:.3f}  angular={angular:.3f}"
        )

    def _clamp(self, linear: float, angular: float):
        ml = 2.0
        ma = 2.0
        return (
            max(-ml, min(ml, linear)),
            max(-ma, min(ma, angular)),
        )

    def publish_velocity(self):
        with self.lock:
            if self.moving:
                msg = Twist()
                msg.linear.x = self.linear_speed
                msg.angular.z = self.angular_speed
                self.pub.publish(msg)

    def destroy_node(self):
        self._http_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()