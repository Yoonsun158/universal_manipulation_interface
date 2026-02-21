# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import time
import numpy as np
import zerorpc

# %%
def calibrate_latency():
    client = zerorpc.Client()
    client.connect("tcp://192.168.0.5:4242")
    
    latencies = []
    print("开始测量系统间延迟 (Server -> Client)...")
    
    for _ in range(100):

        t_client_send = time.perf_counter()  # 客户端记录发送请求的时间戳 (t_client_send)
        # 1. 远程调用并记录数据
        res = client.get_gripper_state()
        # 2. 客户端立即记录接收时间戳 (t_recv)
        t_client_recv = time.perf_counter()
        
        t_srv = res['latency_RobotToServer']  # 服务器记录的机器人到服务器的时间戳
        
        # 3. 计算传输延迟
        # 由于时钟已同步，这个差值就是纯粹的网络传输 + 序列化延迟
        latency = (t_client_recv - t_client_send) / 2  # 来回平均延迟
        # latency = t_srv
        latencies.append(latency)
        time.sleep(0.01)
    
    # Polymetis 获取数据的内部固定硬件延迟通常约为 1ms (1kHz 循环)
    avg_network_latency = np.mean(latencies)
    total_estimated_latency = avg_network_latency + 0.001 # 补偿机器人到服务器的 1ms
    
    print(f"平均传输延迟 (RPC): {avg_network_latency * 1000:.2f} ms")
    print(f"预估总本体感知延迟: {total_estimated_latency * 1000:.2f} ms")

if __name__ == "__main__":
    calibrate_latency()