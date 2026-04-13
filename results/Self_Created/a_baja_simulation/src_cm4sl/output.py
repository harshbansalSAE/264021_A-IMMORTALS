"""
CarMaker Radar RSI Decoder — Port 2212
=======================================
Decodes raw binary RSI stream from CarMaker SimNet into
structured Cartesian / Spherical radar detections.

Protocol framing (per CarMaker RSI):
  [64-byte ASCII header]  →  "*RadarRSI <simtime> <bytecount>\0   ..."
  [N binary bytes]        →  tRSIResMsg (MessageRsiResult.h structures)

Binary layout (little-endian):
  tRSIResMsg_Header          24 bytes
  └─ tRSIResMsg_Radar_Header 24 bytes  (per sensor)
     └─ tOutputPointC_R      20 bytes  (per detection, cart or spherical)
"""

import socket
import struct
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

# ── Connection ──────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 2212

# ── Struct formats (all little-endian) ──────────────────────────────────────
#
# tRSIResMsg_Header:
#   short  ID          (2)
#   short  padding     (2)
#   int    ResMsgType  (4)   0=FULL, 1=SYNC
#   int    MsgSize     (4)   total message bytes including this header
#   int    MsgBodySize (4)   body bytes only
#   float  TimeStamp   (4)   simulation time [s]
#   int    nSensors    (4)
RSI_HDR_FMT  = "<hhiiifi"          # 24 bytes
RSI_HDR_SIZE = struct.calcsize(RSI_HDR_FMT)   # must be 24

#
# tRSIResMsg_Radar_Header:
#   int SensorID   (4)
#   int OutputType (4)   0=Cartesian, 1=Spherical, 2=VRx
#   int nDetPointC (4)   # detections in coord mode
#   int nDetVRx    (4)   # detections in VRx mode  (-1 if unused)
#   int nVRx       (4)   # virtual receivers        (-1 if unused)
#   int nMaxDet    (4)   maximum possible detections
RADAR_HDR_FMT  = "<iiiiii"         # 24 bytes
RADAR_HDR_SIZE = struct.calcsize(RADAR_HDR_FMT)   # must be 24

#
# tOutputPointC_R  (Cartesian OR Spherical — same binary layout):
#   float coord[0]  (4)   x [m]   OR  r   [m]
#   float coord[1]  (4)   y [m]   OR  phi [rad]   (azimuth)
#   float coord[2]  (4)   z [m]   OR  theta [rad] (polar/elevation)
#   float PowerdB   (4)   [dBm]
#   float vel       (4)   relative radial velocity [m/s]  (neg = approaching)
POINT_FMT  = "<fffff"              # 20 bytes
POINT_SIZE = struct.calcsize(POINT_FMT)           # must be 20

# CarMaker prefixes every RSI message with a 64-byte null-padded ASCII line
CM_TEXT_HDR_SIZE = 64

OUTPUT_TYPES = {0: "Cartesian", 1: "Spherical", 2: "VRx"}
RSI_MSG_TYPES = {0: "FULL", 1: "SYNC"}


# ── Data classes ─────────────────────────────────────────────────────────────
@dataclass
class RadarDetection:
    """One radar detection point in Cartesian or Spherical mode."""
    # Cartesian (OutputType == 0)
    x: Optional[float] = None       # longitudinal [m]  (+x = forward)
    y: Optional[float] = None       # lateral [m]       (+y = left)
    z: Optional[float] = None       # vertical [m]      (+z = up)
    # Spherical (OutputType == 1)
    r:     Optional[float] = None   # radial distance [m]
    phi:   Optional[float] = None   # azimuth angle [rad]
    theta: Optional[float] = None   # polar angle [rad]
    # Common
    power_dBm: float = 0.0
    vel_mps:   float = 0.0          # negative = approaching

    # Derived (always computed)
    range_m:      float = 0.0
    azimuth_deg:  float = 0.0
    elevation_deg: float = 0.0


@dataclass
class RadarSensorFrame:
    sensor_id:   int
    output_type: int
    n_det:       int
    n_max_det:   int
    detections:  List[RadarDetection] = field(default_factory=list)


@dataclass
class RSIFrame:
    sim_time:   float
    msg_type:   int
    n_sensors:  int
    sensors:    List[RadarSensorFrame] = field(default_factory=list)


# ── Parsing ───────────────────────────────────────────────────────────────────
def recv_exactly(sock: socket.socket, n: int) -> bytes:
    """Block until exactly n bytes are received."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed by remote")
        buf.extend(chunk)
    return bytes(buf)


def decode_detection(raw: bytes, offset: int, output_type: int) -> RadarDetection:
    c0, c1, c2, power, vel = struct.unpack_from(POINT_FMT, raw, offset)
    det = RadarDetection(power_dBm=power, vel_mps=vel)

    if output_type == 0:  # Cartesian
        det.x, det.y, det.z = c0, c1, c2
        det.range_m       = math.sqrt(c0**2 + c1**2 + c2**2)
        det.azimuth_deg   = math.degrees(math.atan2(c1, c0))
        det.elevation_deg = math.degrees(math.atan2(c2, math.sqrt(c0**2 + c1**2)))
    else:                  # Spherical  (c0=r, c1=phi_az, c2=theta_polar)
        det.r, det.phi, det.theta = c0, c1, c2
        det.range_m       = c0
        det.azimuth_deg   = math.degrees(c1)
        det.elevation_deg = math.degrees(c2)
        # Back-compute Cartesian
        det.x = c0 * math.cos(c2) * math.cos(c1)
        det.y = c0 * math.cos(c2) * math.sin(c1)
        det.z = c0 * math.sin(c2)

    return det


def parse_rsi_binary(data: bytes, sim_time: float) -> Optional[RSIFrame]:
    """Parse a complete binary RSI message into an RSIFrame."""
    if len(data) < RSI_HDR_SIZE:
        return None

    (msg_id, _pad, resi_msg_type,
     msg_size, body_size,
     timestamp, n_sensors) = struct.unpack_from(RSI_HDR_FMT, data, 0)

    frame = RSIFrame(
        sim_time  = sim_time,
        msg_type  = resi_msg_type,
        n_sensors = n_sensors,
    )

    body   = data[RSI_HDR_SIZE:]
    offset = 0

    for _ in range(max(n_sensors, 0)):
        if offset + RADAR_HDR_SIZE > len(body):
            break

        (sensor_id, output_type,
         n_det_c, n_det_vrx,
         n_vrx, n_max_det) = struct.unpack_from(RADAR_HDR_FMT, body, offset)
        offset += RADAR_HDR_SIZE

        sensor = RadarSensorFrame(
            sensor_id   = sensor_id,
            output_type = output_type,
            n_det       = max(n_det_c, 0),
            n_max_det   = n_max_det,
        )

        if output_type in (0, 1):  # Cartesian or Spherical
            for _ in range(sensor.n_det):
                if offset + POINT_SIZE > len(body):
                    break
                det = decode_detection(body, offset, output_type)
                sensor.detections.append(det)
                offset += POINT_SIZE

        frame.sensors.append(sensor)

    return frame


# ── Display ───────────────────────────────────────────────────────────────────
def print_frame(frame: RSIFrame):
    mode = RSI_MSG_TYPES.get(frame.msg_type, f"?({frame.msg_type})")
    print(f"\n{'─'*70}")
    print(f" t = {frame.sim_time:10.4f} s  |  {frame.n_sensors} sensor(s)  |  [{mode}]")
    print(f"{'─'*70}")

    for s in frame.sensors:
        otype = OUTPUT_TYPES.get(s.output_type, f"?({s.output_type})")
        print(f"  Sensor {s.sensor_id}  |  Mode: {otype:10s}  |  "
              f"Detections: {s.n_det}  (max {s.n_max_det})")

        if not s.detections:
            print("    (no detections this cycle)")
            continue

        # Header row
        print(f"    {'#':>3}  {'x[m]':>8}  {'y[m]':>8}  {'z[m]':>7}  "
              f"{'range[m]':>9}  {'az[°]':>7}  {'el[°]':>7}  "
              f"{'vel[m/s]':>9}  {'pwr[dBm]':>9}")
        print(f"    {'─'*3}  {'─'*8}  {'─'*8}  {'─'*7}  "
              f"{'─'*9}  {'─'*7}  {'─'*7}  {'─'*9}  {'─'*9}")

        for i, d in enumerate(s.detections):
            x = d.x if d.x is not None else d.r * math.cos(d.theta) * math.cos(d.phi)
            y = d.y if d.y is not None else d.r * math.cos(d.theta) * math.sin(d.phi)
            z = d.z if d.z is not None else d.r * math.sin(d.theta)
            print(f"    {i:>3}  {x:>8.3f}  {y:>8.3f}  {z:>7.3f}  "
                  f"{d.range_m:>9.3f}  {d.azimuth_deg:>7.2f}  {d.elevation_deg:>7.2f}  "
                  f"{d.vel_mps:>9.3f}  {d.power_dBm:>9.2f}")


# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)

    print(f"Connecting to {HOST}:{PORT} …")
    sock.connect((HOST, PORT))
    print("Connected.  Waiting for RSI frames (Ctrl-C to stop).\n")

    frame_count  = 0
    start_wall   = time.time()
    buffer       = b""

    try:
        while True:
            # ── Step 1: accumulate data into local buffer ──────────────────
            chunk = sock.recv(8192)
            if not chunk:
                print("Remote closed the connection.")
                break
            buffer += chunk

            # ── Step 2: scan buffer for complete messages ──────────────────
            while True:
                if len(buffer) < CM_TEXT_HDR_SIZE:
                    break  # need more data

                # Expect a '*' line as the framing marker
                if buffer[0:1] != b"*":
                    # Out of sync — search for next '*'
                    idx = buffer.find(b"*", 1)
                    if idx == -1:
                        buffer = b""
                    else:
                        buffer = buffer[idx:]
                    break

                # Parse the 64-byte CarMaker text header
                text = buffer[:CM_TEXT_HDR_SIZE].rstrip(b"\x00 ").decode("ascii", errors="replace")
                parts = text.split()

                if not parts:
                    buffer = buffer[CM_TEXT_HDR_SIZE:]
                    continue

                tag = parts[0]

                # ── Session banner  (*MovieNX …) ──────────────────────────
                if tag == "*MovieNX":
                    print(f"[CarMaker session]  {' '.join(parts[1:])}")
                    buffer = buffer[CM_TEXT_HDR_SIZE:]
                    continue

                # ── Radar RSI frame  (*RadarRSI <time> <bytes>) ───────────
                if tag == "*RadarRSI" and len(parts) >= 3:
                    try:
                        sim_time = float(parts[1])
                        msg_bytes = int(parts[2])
                    except ValueError:
                        buffer = buffer[CM_TEXT_HDR_SIZE:]
                        continue

                    total_needed = CM_TEXT_HDR_SIZE + msg_bytes
                    if len(buffer) < total_needed:
                        break  # wait for the rest to arrive

                    binary = buffer[CM_TEXT_HDR_SIZE:total_needed]
                    buffer = buffer[total_needed:]

                    frame = parse_rsi_binary(binary, sim_time)
                    if frame:
                        print_frame(frame)
                        frame_count += 1
                    continue

                # ── Unknown tag — skip ────────────────────────────────────
                buffer = buffer[CM_TEXT_HDR_SIZE:]

    except KeyboardInterrupt:
        pass
    except ConnectionError as exc:
        print(f"Connection error: {exc}")
    finally:
        sock.close()
        elapsed = time.time() - start_wall
        print(f"\nSocket closed.  Decoded {frame_count} RSI frames in {elapsed:.1f} s.")


if __name__ == "__main__":
    main()