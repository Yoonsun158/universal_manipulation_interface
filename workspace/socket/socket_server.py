import socket
import threading


def handle_client(conn: socket.socket, addr) -> None:
    with conn:
        while True:
            try:
                data = conn.recv(1024)
                if data:
                    conn.sendall("xxxx".encode("utf-8"))  # Echo back
                    print(f"[RECV] {addr}: {data.decode('utf-8')}")
                else:
                    print(f"[INFO] 客户端已断开: {addr}")
                    break
            except KeyboardInterrupt:
                print(f"\n[INFO] 捕获到 Ctrl+C，关闭连接: {addr}")
                break
            except ConnectionResetError:
                print(f"[INFO] 客户端强制关闭连接: {addr}")
                break
            except Exception as e:
                print(f"[ERROR] 处理客户端 {addr} 时发生错误: {e}")
                break


def server(host, port):
    srv_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 端口复用，便于快速重启
    srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_socket.bind((host, port))
    srv_socket.listen()
    print(f"[INFO] 服务器已启动，监听 {host}:{port}")

    # 使用超时的 accept 循环，便于检查 stop_event
    while True:
        try:
            conn, addr = srv_socket.accept()
            print(f"[INFO] 新客户端连接: {addr}")
            client_thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            client_thread.start()
        except KeyboardInterrupt:
            print("\n[INFO] 捕获到 Ctrl+C，准备关闭服务器…")
            break

    srv_socket.close()
    print("[INFO] 服务器已关闭")



if __name__ == "__main__":
    host = "127.0.0.1"
    port = 5000
    server(host, port)