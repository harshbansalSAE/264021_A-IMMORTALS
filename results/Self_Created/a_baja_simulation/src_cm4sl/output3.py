import socket
import struct
import math
import time
import threading
import numpy as np
from sklearn.cluster import DBSCAN
from dataclasses import dataclass, field
from typing import List, Optional

# ── Connection Settings ──────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 2212
MATLAB_IP = "127.0.0.1"
MATLAB_PORT = 5005
UDP_PUBLISH_RATE_HZ = 50 
DEBUG_MODE = False  # Keep False during runtime to eliminate I/O latency

# Global Thread-safe State
udp_payload_data = [0.0, 0.0, 0.0]
udp_payload_lock = threading.Lock()

# ── Binary Formats ───────────────────────────────────────────────────────────
RSI_HDR_FMT  = "<hhiiifi"          # 24 bytes
RADAR_HDR_FMT  = "<iiiiii"         # 24 bytes
POINT_FMT  = "<fffff"              # 20 bytes
CM_TEXT_HDR_SIZE = 64
RSI_HDR_SIZE = struct.calcsize(RSI_HDR_FMT)
RADAR_HDR_SIZE = struct.calcsize(RADAR_HDR_FMT)
POINT_SIZE = struct.calcsize(POINT_FMT)

@dataclass
class RadarDetection:
    x: Optional[float] = None; y: Optional[float] = None; z: Optional[float] = None
    r: Optional[float] = None; phi: Optional[float] = None; theta: Optional[float] = None
    power_dBm: float = 0.0; vel_mps: float = 0.0

@dataclass
class TrackedTarget:
    x: float; y: float; vel: float; pwr: float; points_in_cluster: int

@dataclass
class RadarSensorFrame:
    sensor_id: int; output_type: int; n_det: int; detections: List[RadarDetection] = field(default_factory=list)

@dataclass
class RSIFrame:
    sim_time: float; sensors: List[RadarSensorFrame] = field(default_factory=list)

# ── Logic ────────────────────────────────────────────────────────────────────
def cluster_sensor_detections(sensor: RadarSensorFrame) -> List[TrackedTarget]:
    filtered = [d for d in sensor.detections if d.power_dBm >= -95.0 and -11.0 <= d.vel_mps <= -5.0]
    if not filtered: return []
    
    coords = np.array([[d.x if d.x is not None else d.r * math.cos(d.theta) * math.cos(d.phi),
                        d.y if d.y is not None else d.r * math.cos(d.theta) * math.sin(d.phi)] for d in filtered])
    
    clustering = DBSCAN(eps=2.5, min_samples=1).fit(coords)
    targets = []
    for cid in set(clustering.labels_):
        if cid == -1: continue
        idx = np.where(clustering.labels_ == cid)[0]
        pts = [filtered[i] for i in idx]
        c_coords = coords[idx]
        targets.append(TrackedTarget(x=round(float(np.mean(c_coords[:,0])), 3),
                                     y=round(float(np.mean(c_coords[:,1])), 3),
                                     vel=round(float(np.mean([p.vel_mps for p in pts])), 3),
                                     pwr=round(float(np.max([p.power_dBm for p in pts])), 2),
                                     points_in_cluster=len(pts)))
    return targets

def decode_detection(raw: bytes, offset: int, otype: int) -> RadarDetection:
    c0, c1, c2, pwr, vel = struct.unpack_from(POINT_FMT, raw, offset)
    d = RadarDetection(power_dBm=pwr, vel_mps=vel)
    if otype == 0: # Cartesian
        d.x, d.y, d.z = c0, c1, c2
    else: # Spherical
        d.r, d.phi, d.theta = c0, c1, c2
        d.x = c0 * math.cos(c2) * math.cos(c1)
        d.y = c0 * math.cos(c2) * math.sin(c1)
    return d

def udp_publisher_task():
    """Background thread: Always sends data to MATLAB at a fixed rate."""
    pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        with udp_payload_lock:
            payload = struct.pack('<3d', *udp_payload_data)
        try:
            pub_sock.sendto(payload, (MATLAB_IP, MATLAB_PORT))
        except: pass
        time.sleep(1.0 / UDP_PUBLISH_RATE_HZ)

def process_frame(frame: RSIFrame):
    global udp_payload_data
    for s in frame.sensors:
        targets = cluster_sensor_detections(s)
        if not targets:
            with udp_payload_lock:
                udp_payload_data = [0.0, 0.0, 0.0]
            if DEBUG_MODE:
                print("    Target:0, X:0, Y:0, vel:-0, r:0, RCS:-0, Hits:0")
        else:
            closest = min(targets, key=lambda t: (t.x**2 + t.y**2)**0.5)
            with udp_payload_lock:
                udp_payload_data = [float(len(targets)), (closest.x**2 + closest.y**2)**0.5, closest.vel]
            
            if DEBUG_MODE:
                for i, t in enumerate(targets):
                    print(f"    Target:{i}, X:{t.x:.3f}, Y:{t.y:.3f}, vel:{t.vel:.3f}, r:{(t.x**2+t.y**2)**0.5:.3f}, RCS:{t.pwr:.2f}, Hits:{t.points_in_cluster}")

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    threading.Thread(target=udp_publisher_task, daemon=True).start()
    print(f"UDP Publisher active on {MATLAB_PORT}...")

    while True:
        with udp_payload_lock:
            udp_payload_data = [0.0, 0.0, 0.0]
            
        tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_sock.settimeout(2.0)

        print(f"Connecting to CarMaker ({HOST}:{PORT})...")
        try:
            tcp_sock.connect((HOST, PORT))
            print("Connected successfully. Listening for data (Silent Mode)...")
            buffer = b""
            while True:
                try:
                    chunk = tcp_sock.recv(8192)
                    if not chunk: break
                    buffer += chunk
                    while len(buffer) >= CM_TEXT_HDR_SIZE:
                        if not buffer.startswith(b"*"):
                            idx = buffer.find(b"*", 1)
                            buffer = buffer[idx:] if idx != -1 else b""
                            continue
                        
                        header_text = buffer[:CM_TEXT_HDR_SIZE].decode('ascii', 'ignore').split()
                        if len(header_text) >= 3 and header_text[0] == "*RadarRSI":
                            sim_time, msg_bytes = float(header_text[1]), int(header_text[2])
                            total_len = CM_TEXT_HDR_SIZE + msg_bytes
                            if len(buffer) < total_len: break
                            
                            # Parse Binary
                            binary = buffer[CM_TEXT_HDR_SIZE:total_len]
                            buffer = buffer[total_len:]
                            
                            _, _, _, _, _, _, n_sens = struct.unpack_from(RSI_HDR_FMT, binary, 0)
                            frame = RSIFrame(sim_time=sim_time)
                            offset = RSI_HDR_SIZE
                            for _ in range(max(0, n_sens)):
                                sid, otype, n_det, _, _, _ = struct.unpack_from(RADAR_HDR_FMT, binary, offset)
                                offset += RADAR_HDR_SIZE
                                s_fr = RadarSensorFrame(sensor_id=sid, output_type=otype, n_det=n_det)
                                for _ in range(n_det):
                                    s_fr.detections.append(decode_detection(binary, offset, otype))
                                    offset += POINT_SIZE
                                frame.sensors.append(s_fr)
                            process_frame(frame)
                        else:
                            buffer = buffer[CM_TEXT_HDR_SIZE:]
                except socket.timeout: continue
        except Exception as e:
            print(f"Link Down: {e}. Retrying in 2s...")
            time.sleep(2.0)
        finally:
            tcp_sock.close()

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nStopped.")