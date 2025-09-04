#!/usr/bin/env python3

import socket
from struct import unpack

"""
NOTES:

- This process MUST listen to a different port than the RemoteRequest client;
- A socket MUST be closed BEFORE exiting the process.
"""


def main():
    # TODO: Implement the PositionBroadcast client here.
    listen_addr = ("", 65431)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(listen_addr)

    try:
        while True:
            data, addr = s.recvfrom(1024)  # buffer max 1024 bytes
            if len(data) == 16:
                x, y, yaw, id = unpack("<fffi", data)
                print(f"Received from {addr}: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}, id={id}")
            else:
                print(f"Received unexpected packet size {len(data)} from {addr}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        s.close()


if __name__ == "__main__":
    main()
