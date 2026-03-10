#!/usr/bin/env python3
"""
SiliconForge Real-Time Studio — HTTP Bridge
Connects browser frontend to C++ EDA engine via PTY.

Endpoints:
  GET  /                → studio/index.html
  POST /api/command     → Send shell command, auto-exports full state, returns output + state
  POST /api/upload      → Upload Verilog file to studio/ directory
  GET  /api/state       → Return latest full JSON state
  GET  /*               → Static files
"""
import http.server
import socketserver
import json
import os
import threading
import pty
import subprocess
import select
import time
import sys
import tempfile

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8080
STATE_FILE = os.path.join(PROJECT_DIR, 'web_state.json')

# ── PTY Shell Process ─────────────────────────────────────────────────

master, slave = pty.openpty()
proc = subprocess.Popen(
    ['./build/src/siliconforge'],
    stdin=slave, stdout=slave, stderr=subprocess.STDOUT,
    cwd=PROJECT_DIR
)
os.close(slave)

output_buffer = ""
buffer_lock = threading.Lock()

def reader_thread():
    global output_buffer
    while True:
        r, _, _ = select.select([master], [], [], 1.0)
        if r:
            try:
                data = os.read(master, 8192).decode('utf-8', errors='replace')
                if data:
                    with buffer_lock:
                        output_buffer += data
                else:
                    break
            except OSError:
                break

t = threading.Thread(target=reader_thread, daemon=True)
t.start()

# Wait for initial prompt
time.sleep(0.5)
with buffer_lock:
    output_buffer = ""

# ── Helper: send command and wait for response ────────────────────────

command_lock = threading.Lock()

def send_command(cmd, timeout_sec=120.0):
    """Send a command to the C++ shell and wait for the sf> prompt."""
    with command_lock:
        global output_buffer

        # Drain any leftover output from a previous timed-out command
        drain_start = time.time()
        while time.time() - drain_start < 5.0:
            with buffer_lock:
                buf = output_buffer
            if not buf or 'sf>' in buf.split('\n')[-1]:
                break
            time.sleep(0.1)

        with buffer_lock:
            output_buffer = ""

        os.write(master, (cmd + '\n').encode('utf-8'))

        timed_out = False
        start = time.time()
        while time.time() - start < timeout_sec:
            time.sleep(0.05)
            with buffer_lock:
                buf = output_buffer
            lines = buf.split('\n')
            if lines and 'sf>' in lines[-1]:
                break
        else:
            timed_out = True

        with buffer_lock:
            result = output_buffer

        # Clean: remove the echo of the command and trailing prompt
        lines = result.split('\n')
        if lines and lines[0].strip() == cmd.strip():
            lines = lines[1:]
        if lines and 'sf>' in lines[-1]:
            lines[-1] = lines[-1].replace('sf>', '').rstrip()
            if not lines[-1].strip():
                lines.pop()

        return '\n'.join(lines), timed_out

# ── HTTP Handler ──────────────────────────────────────────────────────

class SFStudioHandler(http.server.SimpleHTTPRequestHandler):

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        if self.path == '/':
            self.path = '/studio/index.html'
            super().do_GET()
            return

        if self.path.startswith('/api/state'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            try:
                with open(STATE_FILE, 'r') as f:
                    self.wfile.write(f.read().encode('utf-8'))
            except Exception:
                self.wfile.write(b'{}')
            return

        # Legacy endpoint
        if self.path.startswith('/web_state.json'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            try:
                with open(STATE_FILE, 'r') as f:
                    self.wfile.write(f.read().encode('utf-8'))
            except Exception:
                self.wfile.write(b'{}')
            return

        super().do_GET()

    def do_POST(self):
        # ── Execute Command ───────────────────────────────────────────
        if self.path == '/api/command':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8')

            try:
                data = json.loads(body)
                cmd = data.get('command', body)
            except (json.JSONDecodeError, AttributeError):
                cmd = body.strip()

            # Execute the command
            output, timed_out = send_command(cmd)

            # Auto-export full state after every command (skip if timed out to avoid desync)
            if not timed_out and cmd.strip() not in ('export_full web_state.json', 'export_json web_state.json', 'help', 'exit', 'quit'):
                send_command('export_full web_state.json', timeout_sec=10.0)

            # Read the state file
            state = {}
            try:
                with open(STATE_FILE, 'r') as f:
                    state = json.load(f)
            except Exception:
                pass

            # Return command output + full state
            response = {
                "output": output,
                "state": state
            }

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        # ── Upload Verilog File ───────────────────────────────────────
        if self.path == '/api/upload':
            content_type = self.headers.get('Content-Type', '')
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)

            if 'multipart/form-data' in content_type:
                try:
                    # Extract boundary from Content-Type header
                    boundary = None
                    for part in content_type.split(';'):
                        part = part.strip()
                        if part.startswith('boundary='):
                            boundary = part[len('boundary='):]
                            break

                    if not boundary:
                        raise ValueError("Missing boundary in multipart header")

                    boundary_bytes = ('--' + boundary).encode('utf-8')
                    parts = body.split(boundary_bytes)

                    filename = 'uploaded.v'
                    file_content = b''

                    for part in parts:
                        if b'Content-Disposition' not in part:
                            continue
                        header_end = part.find(b'\r\n\r\n')
                        if header_end < 0:
                            continue
                        headers_raw = part[:header_end].decode('utf-8', errors='replace')
                        file_data = part[header_end + 4:]
                        # Strip trailing CRLF and closing boundary marker
                        if file_data.endswith(b'\r\n'):
                            file_data = file_data[:-2]
                        if file_data.endswith(b'--'):
                            file_data = file_data[:-2]
                        if file_data.endswith(b'\r\n'):
                            file_data = file_data[:-2]

                        for line in headers_raw.split('\r\n'):
                            if 'filename=' in line:
                                fn_start = line.find('filename="')
                                if fn_start >= 0:
                                    fn_start += len('filename="')
                                    fn_end = line.find('"', fn_start)
                                    if fn_end > fn_start:
                                        filename = os.path.basename(line[fn_start:fn_end])
                        file_content = file_data
                        break

                    filepath = os.path.join(PROJECT_DIR, 'studio', filename)
                    with open(filepath, 'wb') as f:
                        f.write(file_content)

                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        "success": True,
                        "filename": filename,
                        "path": f"studio/{filename}"
                    }).encode('utf-8'))
                    return

                except Exception as e:
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        "success": False,
                        "error": f"Multipart parse error: {str(e)}"
                    }).encode('utf-8'))
                    return

            # Simple JSON body upload
            try:
                data = json.loads(body.decode('utf-8'))
                filename = data.get('filename', 'uploaded.v')
                content = data.get('content', '')
                filepath = os.path.join(PROJECT_DIR, 'studio', filename)
                with open(filepath, 'w') as f:
                    f.write(content)

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "success": True,
                    "filename": filename,
                    "path": f"studio/{filename}"
                }).encode('utf-8'))
                return
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
                return

        self.send_error(404)

    def log_message(self, format, *args):
        # Quieter logging — only show API calls
        msg = format % args
        if '/api/' in msg or 'POST' in msg:
            sys.stderr.write(f"[Studio] {msg}\n")

# ── Start Server ──────────────────────────────────────────────────────

os.chdir(PROJECT_DIR)

# Initialize empty state
if not os.path.exists(STATE_FILE):
    with open(STATE_FILE, 'w') as f:
        f.write('{}')

class ThreadedServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True

with ThreadedServer(("0.0.0.0", PORT), SFStudioHandler) as httpd:
    print(f"\n{'='*55}")
    print(f"  SiliconForge Real-Time Studio v15.0")
    print(f"  Open http://localhost:{PORT} in your browser")
    print(f"{'='*55}")
    print(f"  Backend: C++ EDA Engine (14 Phases, 111 Tests)")
    print(f"  API:     POST /api/command  GET /api/state")
    print(f"  Upload:  POST /api/upload")
    print(f"{'='*55}\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Real-Time Studio...")
        proc.terminate()
