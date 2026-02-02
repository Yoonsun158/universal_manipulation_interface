import socket
import threading


def client(host, port):
    cli_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli_socket.connect((host, port))
    while True:
        try:
            # send_data = "客户端连接".encode("utf-8")
            send_data = "client try to connect ...\n".encode("utf-8")
            cli_socket.send(send_data)
        except KeyboardInterrupt:
            print("\n[INFO] 捕获到 Ctrl+C，准备关闭客户端…")
            break
    
    cli_socket.close()



if __name__ == "__main__":
    host = "127.0.0.1"
    port = 5000
    client(host, port)