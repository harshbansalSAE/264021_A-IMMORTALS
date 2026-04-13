# carmaker_tcp_reader.py
import socket
import cv2
import numpy as np
import time
import os
import threading
import queue
from collections import deque

# --- CONFIGURATION ---
CAM_IP   = "127.0.0.1"
CAM_PORT = 2211
SAVE_DIR = "frames"
RECONNECT_DELAY = 2

WIDTH       = 1920
HEIGHT      = 1080
N_CH        = 3
FRAME_BYTES = WIDTH * HEIGHT * N_CH
TARGET_FPS  = 30

os.makedirs(SAVE_DIR, exist_ok=True)

# --- SHARED STATE ---
# We use deques with maxlen=1 for Display to ensure ZERO latency.
# The receiver pushes, and the display always grabs the "freshest" frame.
display_deque = deque(maxlen=1)
record_queue  = queue.Queue(maxsize=60) # Buffer for disk writing
stop_event    = threading.Event()
fps_buffer    = deque(maxlen=30)

# ═════════════════════════════════════════════════════════════════════
# THREAD 1 — Receiver (Optimized for Speed)
# ═════════════════════════════════════════════════════════════════════
def recv_into_memory(sock, buffer):
    view = memoryview(buffer)
    pos = 0
    n = len(buffer)
    while pos < n:
        received = sock.recv_into(view[pos:], n - pos)
        if not received: raise ConnectionError("Socket closed")
        pos += received

def receiver_thread():
    # Pre-allocate memory to avoid garbage collection spikes
    header_buffer = bytearray(64)
    pixel_buffer  = bytearray(FRAME_BYTES)

    while not stop_event.is_set():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Reduce latency
            sock.settimeout(5)
            sock.connect((CAM_IP, CAM_PORT))
            sock.settimeout(None)
            print(f"[RECV] Connected at {CAM_IP}:{CAM_PORT}")

            while not stop_event.is_set():
                t0 = time.perf_counter()
                
                recv_into_memory(sock, header_buffer)
                recv_into_memory(sock, pixel_buffer)

                # Fast conversion to numpy (BGR format)
                img = np.frombuffer(pixel_buffer, dtype=np.uint8).reshape((HEIGHT, WIDTH, N_CH)).copy()

                # Non-blocking hand-off
                display_deque.append(img)
                
                try:
                    record_queue.put_nowait(img)
                except queue.Full:
                    pass # Prioritize live stream over recording if disk is slow

                fps_buffer.append(time.perf_counter() - t0)

        except Exception as e:
            print(f"[RECV] Error: {e}. Reconnecting...")
            time.sleep(RECONNECT_DELAY)
        finally:
            if sock: sock.close()

# ═════════════════════════════════════════════════════════════════════
# THREAD 2 — Display (Optimized for Low Latency)
# ═════════════════════════════════════════════════════════════════════
def display_thread():
    WINDOW = "CarMaker Live Stream"
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720) 

    while not stop_event.is_set():
        if not display_deque:
            time.sleep(0.01)
            continue
        
        img = display_deque.pop() # Get the freshest frame available
        
        # Calculate FPS
        fps = 1.0 / (sum(fps_buffer)/len(fps_buffer)) if fps_buffer else 0
        
        # Simple HUD
        cv2.putText(img, f"FPS: {fps:.1f}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow(WINDOW, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()

    cv2.destroyAllWindows()

# ═════════════════════════════════════════════════════════════════════
# THREAD 3 — Disk Writer (Offloads the heaviest tasks)
# ═════════════════════════════════════════════════════════════════════
def writer_thread():
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(os.path.join(SAVE_DIR, "output.mp4"), fourcc, TARGET_FPS, (WIDTH, HEIGHT))
    
    last_save_time = 0
    save_interval = 0.1 # Save 'latest_frame.jpg' only 10 times per sec to save IO

    while not stop_event.is_set() or not record_queue.empty():
        try:
            img = record_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        video.write(img)

        # Save JPEG for Simulink at a capped rate (every 100ms)
        now = time.time()
        if now - last_save_time > save_interval:
            cv2.imwrite(os.path.join(SAVE_DIR, "latest_frame.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            last_save_time = now

    video.release()
    print("[DISK] Video and images saved.")

# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════
def run():
    threads = [
        threading.Thread(target=receiver_thread, daemon=True),
        threading.Thread(target=display_thread,  daemon=False),
        threading.Thread(target=writer_thread,   daemon=True),
    ]

    for t in threads: t.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()

    print("\nShutting down...")

if __name__ == "__main__":
    run()