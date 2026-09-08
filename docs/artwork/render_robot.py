#!/usr/bin/env python3
"""Render documentation only; no ROS nodes, device access or robot commands."""
import argparse
import http.server
from pathlib import Path
import subprocess
import threading

import xacro

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "ros2_ws/src/xlerobot_description"
VENDOR = ROOT / ".xlerobot/readme-art/node_modules/three"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--view", choices=("readme", "zero"), default="readme")
    args = parser.parse_args()
    source = (PACKAGE / "urdf/two_wheel_reference.urdf.xacro").read_text()
    document = xacro.parse(source.replace("$(find xlerobot_description)", str(PACKAGE)))
    xacro.process_doc(document, mappings={
        "hardware_enabled": "false", "torque_enabled": "false",
        "include_right_bus_control": "false", "include_left_bus_control": "false",
    })
    urdf = document.toxml().encode()

    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/robot.urdf":
                self.send_response(200)
                self.send_header("Content-Type", "application/xml")
                self.end_headers()
                self.wfile.write(urdf)
                return
            super().do_GET()

        def translate_path(self, path):
            clean = path.split("?", 1)[0]
            # Serve only artwork, mesh and rendering-library paths, not local config.
            routes = {"/art/": ROOT / "docs/artwork", "/meshes/": PACKAGE / "meshes",
                      "/vendor/": VENDOR}
            for prefix, folder in routes.items():
                if clean.startswith(prefix):
                    candidate = (folder / clean[len(prefix):]).resolve()
                    if candidate.is_relative_to(folder.resolve()) and candidate.is_file():
                        return str(candidate)
            return str(ROOT / "docs/artwork/.not-found")

        def log_message(self, *_args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        capture = "capture_zero.mjs" if args.view == "zero" else "capture.mjs"
        subprocess.run(["node", str(ROOT / "docs/artwork" / capture),
                        f"http://127.0.0.1:{server.server_port}/art/robot.html"], check=True)
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
