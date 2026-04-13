import socket

HOST = "127.0.0.1"
PORT = 2212

def to_hex(data):
    # Convert bytes to hex string (space separated)
    return ' '.join(f'{b:02X}' for b in data)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    print(f"Connecting to {HOST}:{PORT}...")
    sock.connect((HOST, PORT))
    print("Connected! Receiving radar data in HEX:\n")

    try:
        while True:
            data = sock.recv(4096)

            if not data:
                print("Connection closed.")
                break

            # Print HEX
            print(to_hex(data))

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        sock.close()
        print("Socket closed.")

if __name__ == "__main__":
    main()