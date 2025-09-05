#!/usr/bin/env python3

import socket
from struct import unpack

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 65432

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((SERVER_HOST, SERVER_PORT))
        print(f"Connected to {SERVER_HOST}:{SERVER_PORT}")
        print("Enter command (RPOS, OBSF, RBID) or 'quit' to exit")

        while True:
            cmd = input("> ").strip().upper()
            if cmd == "QUIT":
                break
            if cmd not in ["RPOS", "OBSF", "RBID"]:
                print("Invalid command. Use RPOS, OBSF, RBID or quit")
                continue

            # Send 4-byte command
            s.sendall(cmd.encode("utf-8"))

            # Expect 16-byte response
            data = s.recv(16)
            if len(data) != 16:
                print("Invalid response size")
                continue

            if cmd == "RPOS":
                x, y, yaw, _ = unpack("<fffI", data)
                print(f"Position -> X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}")
            elif cmd == "OBSF":
                val, = unpack("<I", data[:4])
                print(f"Obstacle detected: {bool(val)}")
            elif cmd == "RBID":
                rid, = unpack("<I", data[:4])
                print(f"Robot ID: {rid}")

if __name__ == "__main__":
    main()
