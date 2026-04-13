#!/usr/bin/env python3
import socket
import struct
import math

HOST = "localhost"
PORT = 2212
TEXT_HDR_SIZE = 64

FMT_MAIN_HEADER = '<h h i i i f i'  
SIZE_MAIN_HEADER = struct.calcsize(FMT_MAIN_HEADER)

FMT_RADAR_HEADER = '<i i i i i i'
SIZE_RADAR_HEADER = struct.calcsize(FMT_RADAR_HEADER)

# Cartesian Point: x, y, z, PowerdB, vel (5 floats, 20 bytes)
FMT_CARTESIAN_POINT = '<f f f f f'
SIZE_CARTESIAN_POINT = struct.calcsize(FMT_CARTESIAN_POINT)

def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("Connection closed by server.")
        buf.extend(chunk)
    return bytes(buf)

def extract_full_cartesian(sock):
    frame = 0
    while True:
        # 1. Clear the Text Header
        raw_text = recv_exact(sock, TEXT_HDR_SIZE)
        if not raw_text.startswith(b'*RadarRSI'):
            continue 

        # 2. Main Header
        header_bytes = recv_exact(sock, SIZE_MAIN_HEADER)
        msg_id, padding, msg_type, msg_size, body_size, timestamp, n_sensors = struct.unpack(FMT_MAIN_HEADER, header_bytes)
        
        if body_size <= 0:
            continue

        # 3. Read Body
        body_bytes = recv_exact(sock, body_size)
        offset = 0

        frame += 1
        print(f"\n{'='*75}")
        print(f" Frame: {frame} | Time: {timestamp:.4f}s | Sensors: {n_sensors}")
        print(f"{'='*75}")

        for _ in range(n_sensors):
            if offset + SIZE_RADAR_HEADER > len(body_bytes): break
            
            sensor_header = body_bytes[offset : offset + SIZE_RADAR_HEADER]
            sensor_id, out_type, n_det_c, n_det_vrx, n_vrx, n_max_det = struct.unpack(FMT_RADAR_HEADER, sensor_header)
            offset += SIZE_RADAR_HEADER

            print(f" [Sensor {sensor_id}] Mode: Cartesian (0) | Detections: {n_det_c}/{n_max_det}")

            if out_type != 0:
                print("  -> Sensor is not in Cartesian Mode. Skipping...")
                continue

            if n_det_c == 0:
                print("  -> No detections.")
                continue

            header_row = f"  {'ID':>4} | {'X (Long)[m]':>12} | {'Y (Lat)[m]':>12} | {'Z[m]':>8} | {'Power[dBm]':>10} | {'Vel[m/s]':>8}"
            print(header_row)
            print(f"  {'-' * len(header_row)}")

            for d in range(n_det_c):
                point_bytes = body_bytes[offset : offset + SIZE_CARTESIAN_POINT]
                offset += SIZE_CARTESIAN_POINT
                
                x, y, z, power, vel = struct.unpack(FMT_CARTESIAN_POINT, point_bytes)
                print(f"  {d:4d} | {x:12.3f} | {y:12.3f} | {z:8.3f} | {power:10.2f} | {vel:8.3f}")

def main():
    print(f"Connecting to CarMaker at {HOST}:{PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
        print("Connected! Reading Cartesian Radar Stream...\n")
        extract_full_cartesian(sock)
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
