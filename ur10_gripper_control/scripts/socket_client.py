import socket
import sys, time


def main():
    HOST = "192.168.244.164"  # Standard loopback interface address (localhost)
    PORT = 65500  # Port to listen on (non-privileged ports are > 1023)

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error:
        print('Could not create a socket')
        time.sleep(1)
        sys.exit()

    try:
        client.connect((HOST, PORT))
    except socket.error:
        print('Could not connect to server')
        time.sleep(1)
        sys.exit()

    online = True
    while online:
        data = input()
        client.sendall(data.encode())
        while True:
            message = client.recv(4096)
            if 'q^' in message.decode():
                client.close()
                online = False
                break

            print('[+] Received: ' + message.decode())
            break  # stop receiving


# start client
main()
