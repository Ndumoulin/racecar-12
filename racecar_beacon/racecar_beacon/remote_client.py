#!/usr/bin/env python3

import socket

"""
NOTES:
- This client connects to a TCP server and sends a command entered by the user.
- A socket MUST be closed BEFORE exiting the process (handled automatically with 'with').
"""

def main():
    host = "127.0.0.1"
    port = 65432

    # Create and connect the client socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        print(f"Connected to {host}:{port}")

        command = input("Write the command here: ")
        command_bytes = command.encode("ascii") 

        # Send the command to the server
        sock.sendall(command_bytes)
        print("Command sent!")

        # Receive a reply
        reply = sock.recv(1024)
        if reply:
            print("Reply from server:", reply.decode("ascii"))

if __name__ == "__main__":
    main()