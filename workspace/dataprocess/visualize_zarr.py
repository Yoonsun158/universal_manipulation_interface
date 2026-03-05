# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%

# import zarr
# import numcodecs
# import imagecodecs.numcodecs

# # 显式注册编解码器
# imagecodecs.numcodecs.register_codecs()

# # 加载整个数据集组
# root = zarr.open('assembly_session/dataset.zarr', mode='r')

# # 访问 meta/episode_ends 数组
# episode_ends1 = root['data/robot0_demo_end_pose'][411]
# episode_ends2 = root['data/robot0_demo_start_pose'][411]
# episode_ends3 = root['data/robot0_eef_pos'][411]
# episode_ends4 = root['data/robot0_eef_rot_axis_angle'][411]

# print(f"episode_ends1数据形状: {episode_ends1.shape}")
# print(f"数据内容: {episode_ends1}")
# print(f"数据类型: {episode_ends1.dtype}")
# print(f"episode_ends2数据形状: {episode_ends2.shape}")
# print(f"数据内容: {episode_ends2}")
# print(f"数据类型: {episode_ends2.dtype}")
# print(f"episode_ends3数据形状: {episode_ends3.shape}")
# print(f"数据内容: {episode_ends3}")
# print(f"数据类型: {episode_ends3.dtype}")
# print(f"episode_ends4数据形状: {episode_ends4.shape}")
# print(f"数据内容: {episode_ends4}")
# print(f"数据类型: {episode_ends4.dtype}")

# %%
'''
episode_ends 数组内容:
[  411   803  1235  1684  2052  2473  2904  3417  3981  4560  4967  5538
  5978  6544  7011  7422  7813  8189  8501  8996  9393  9796 10224 10628
 11012 11375 11687 12068 12432 12808 13157 13505 13872 14248 14685 15153
 15603 15989 16321 16666 16994 17347 17764 18149 18498 18934 19270 19617
 19949 20283 20678 21085 21453 21828 22231 22684 23020 23405 23765 24113
 24509 24925 25329 25694 26053 26411 26871 27220 27528 27881 28216 28579
 28938 29308 29635 30019 30390 30730 31197 31516 31865 32181 32529 32851
 33189 33631 33980 34337 34704 35155]

'''

import open3d as o3d
import zarr
from scipy.spatial.transform import Rotation as R
import numpy as np # 确保导入了 numpy

root = zarr.open('assembly_session/dataset.zarr', mode='r')
episode_ends = root['meta/episode_ends']

# 获取前两个 episode 的数据（作为示例）
start_idx = 0
end_idx = episode_ends[0] 

eef_pos = root['data/robot0_eef_pos'][start_idx:end_idx]
eef_rot = root['data/robot0_eef_rot_axis_angle'][start_idx:end_idx]

# 将位置(N,3)和姿态(N,3)拼接成(N,6)
data = np.concatenate([eef_pos, eef_rot], axis=-1)

print(f"数据形状: {data.shape}")
print(f"坐标范围: X[{data[:,0].min():.3f}, {data[:,0].max():.3f}], Y[{data[:,1].min():.3f}, {data[:,1].max():.3f}], Z[{data[:,2].min():.3f}, {data[:,2].max():.3f}]")

geometries = []

# 创建轨迹点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(data[:, :3])  
pcd.paint_uniform_color([1, 0, 0])  # 轨迹点为红色
geometries.append(pcd)

# 标记初始位姿 (绿色球体) 和 最终位姿 (蓝色球体)
start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
start_sphere.paint_uniform_color([0, 1, 0]) # 绿色
start_sphere.translate(data[0, :3])
geometries.append(start_sphere)

end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
end_sphere.paint_uniform_color([0, 0, 1]) # 蓝色
end_sphere.translate(data[-1, :3])
geometries.append(end_sphere)

# 添加坐标轴表示位姿
for i in range(0, len(data), 20):
    is_edge = (i == 0 or i >= len(data) - 20)
    frame_size = 0.1 if is_edge else 0.03
    
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
    rot_mat = R.from_rotvec(data[i, 3:]).as_matrix()
    coord.rotate(rot_mat, center=(0, 0, 0))
    coord.translate(data[i, :3])
    geometries.append(coord)

print("可视化说明:")
print("1. 红色点序列: 机器人末端轨迹")
print("2. 绿色球体: 初始位姿 (Start Pose)")
print("3. 蓝色球体: 最终位姿 (End Pose)")
print("4. 坐标轴颜色: 红色=X轴, 绿色=Y轴, 蓝色=Z轴")

# 修改可视化配置以增大点显示大小
vis = o3d.visualization.Visualizer()
vis.create_window()
for g in geometries:
    vis.add_geometry(g)
opt = vis.get_render_option()
opt.point_size = 5.0  # 增大点的大小，防止看不见
vis.run()
vis.destroy_window()
