"""
简洁的 TCP Socket 服务器示例

功能要求：
1) 循环接收多个客户端发来的数据（支持并发连接）。
2) 某个客户端断开后，服务端继续等待新的客户端连接，不退出。

用法：
  作为服务运行：
	python test2.py --host 0.0.0.0 --port 5000

  运行内置演示（自动启动服务、连接两次并退出）：
	python test2.py --demo

说明：
- 为了保持简洁，采用“每连接一个线程”的模型；若需要更高并发可改为 selectors/asyncio。
- 服务器会回显收到的数据（echo），便于调试与联通性验证。
"""

from __future__ import annotations

import argparse
import socket
import threading
from typing import Callable, Optional


def handle_client(conn: socket.socket, addr, on_message: Optional[Callable[[bytes, tuple], Optional[bytes]]] = None) -> None:
	"""
	处理单个客户端连接：循环接收数据，打印并回显；客户端断开后退出函数。

	参数：
	- conn: 已建立连接的 socket
	- addr: 客户端地址 (ip, port)
	- on_message: 可选的消息处理回调，入参(raw_bytes, addr)，返回要回发的 bytes；
				  若返回 None 则不回发。
	"""
	with conn:
		conn.settimeout(10.0)  # 防止永久卡死，可按需调整或去掉
		try:
			while True:
				data = conn.recv(4096)
				if not data:
					print(f"[INFO] 客户端已断开: {addr}")
					break

				print(f"[RECV] {addr}: {data!r}")
				if on_message is not None:
					try:
						reply = on_message(data, addr)
					except Exception as e:  # 回调不应影响连接线程稳定性
						print(f"[WARN] on_message 处理异常: {e}")
						reply = None
				else:
					# 默认回显
					reply = data

				if reply:
					conn.sendall(reply)
		except socket.timeout:
			print(f"[INFO] 连接超时，关闭: {addr}")
		except ConnectionResetError:
			print(f"[INFO] 客户端强制关闭连接: {addr}")
		except Exception as e:
			print(f"[ERROR] 连接处理异常 {addr}: {e}")


def serve(host: str = "0.0.0.0", port: int = 5000,
		  on_message: Optional[Callable[[bytes, tuple], Optional[bytes]]] = None,
		  stop_event: Optional[threading.Event] = None) -> None:
	"""
	启动 TCP 服务端，循环接受新连接；每个连接交由独立线程处理。

	- host/port: 监听地址与端口
	- on_message: 收到消息时的自定义处理回调；若为 None 则默认回显
	- stop_event: 外部传入的停止事件；若为 None，则永久运行直到 KeyboardInterrupt
	"""
	srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# 端口复用，便于快速重启
	srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	srv_sock.bind((host, port))
	srv_sock.listen()

	print(f"[START] TCP 服务已启动，监听 {host}:{port}")
	try:
		# 使用超时的 accept 循环，便于检查 stop_event
		srv_sock.settimeout(1.0)
		while True:
			if stop_event is not None and stop_event.is_set():
				print("[STOP] 收到停止信号，退出服务循环…")
				break
			try:
				conn, addr = srv_sock.accept()
			except socket.timeout:
				continue

			print(f"[ACCEPT] 新连接: {addr}")
			t = threading.Thread(target=handle_client, args=(conn, addr, on_message), daemon=True)
			t.start()
	except KeyboardInterrupt:
		print("\n[STOP] 捕获到 Ctrl+C，准备关闭服务…")
	finally:
		try:
			srv_sock.close()
		except Exception:
			pass
		print("[CLOSE] 服务端 socket 已关闭")


def _echo_upper(data: bytes, _addr) -> bytes:
	"""示例回调：将消息转为大写后回发。"""
	return data.upper()


def run_demo():
	"""在本进程内自启动服务，进行两次客户端连接演示，最后优雅退出。"""
	host, port = "127.0.0.1", 5055
	stop_event = threading.Event()

	# 启动服务线程
	srv_thread = threading.Thread(target=serve, args=(host, port, _echo_upper, stop_event), daemon=True)
	srv_thread.start()

	# 简单客户端函数
	def client_send(msg: bytes) -> bytes:
		with socket.create_connection((host, port), timeout=3.0) as s:
			s.sendall(msg)
			return s.recv(4096)

	# 连续两次连接，验证“客户端断开后服务仍可用”
	rep1 = client_send(b"hello")
	print(f"[DEMO] 回声1: {rep1!r}")
	rep2 = client_send(b"world")
	print(f"[DEMO] 回声2: {rep2!r}")

	# 结束服务
	stop_event.set()
	srv_thread.join(timeout=3.0)
	print("[DEMO] 演示结束")


def main():
	parser = argparse.ArgumentParser(description="简洁 TCP Socket 服务器")
	parser.add_argument("--host", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
	parser.add_argument("--port", type=int, default=5000, help="监听端口，默认 5000")
	parser.add_argument("--demo", action="store_true", help="运行内置演示并退出")
	args = parser.parse_args()

	if args.demo:
		run_demo()
	else:
		serve(args.host, args.port, _echo_upper)


if __name__ == "__main__":
	main()

