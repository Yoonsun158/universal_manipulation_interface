# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import numpy as np
import time
import zerorpc

# %%
client = zerorpc.Client()
client.connect("tcp://192.168.0.5:4242")

def check_offset():
    offsets = []
    for _ in range(10):
        t0 = time.time()
        # 调用服务器的一个简单函数获取其系统时间
        # 需要你在服务器 FrankaInterface 里添加一个 get_time() 方法
        t_server = client.get_server_time() 
        t1 = time.time()
        
        # 估算传输延迟 (RTT / 2)
        delay = (t1 - t0) / 2
        # 计算客户端相对于服务器的偏移
        # offset = 客户端时间 - (服务器时间 + 延迟)
        offset = t1 - (t_server + delay)
        offsets.append(offset)
    
    print(f"平均时钟偏移 (Client - Server): {np.mean(offsets)*1000:.2f} ms")

if __name__ == "__main__":
    check_offset()
# 注意：如果偏移量 > 1ms，你的延迟测量结果 (t_recv - t_robot) 就是不准确的。