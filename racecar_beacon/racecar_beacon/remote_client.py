#!/usr/bin/env python3

import socket
from struct import unpack

"""
NOTES:

- This process MUST listen to a different port than the PositionBroadcast client;
- A socket MUST be closed BEFORE exiting the process.
"""

server_port = 65432
server_ip = "127.0.0.1"

def main():

    s_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_TCP.connect((server_ip,server_port))
    print("Hawktually connected")


    command = input("Input your command here: ")
    command_bytes = command.encode("ASCII")

    data = s_TCP.recv(1024)

    decoded = data.decode('utf-8')

    print(decoded)


if __name__ == "__main__":
    main()