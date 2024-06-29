"""
    DEPRECATED: DO NOT USE, USE THE NOTEBOOK FOR NOW
"""

# import g2o
# import scipy
# import numpy as np
# import matplotlib.pyplot as plt
#
# from optimizer import PoseGraphOptimization
# from parse import get_dataset
# from icp import icp
#
# if __name__ == "__main__":
#     thres = 0.1
#     lc_num = 0
#     name = 'aces'
#
#     max_x = -float('inf')
#     max_y = -float('inf')
#     min_x = float('inf')
#     min_y = float('inf')
#
#     odoms_laser_file = 'data/' + name + "_odoms_laser.npy"
#     lasers_file = 'data/' + name + "_lasers.npy"
#     ipc_ts_file = 'data/' + name + "_ipc_ts.npy"
#     logger_ts_file = 'data/' + name + "_logger_ts.npy"
#     odoms_laser = np.load(odoms_laser_file)
#     lasers = np.load(lasers_file)
#     ipc_ts = np.load(ipc_ts_file)
#     logger_ts = np.load(logger_ts_file)
#
#     graph_optimizer = PoseGraphOptimization()
#     pose = np.eye(3)
#     id = 0
#     graph_optimizer.add_vertex(id, g2o.SE2(g2o.Isometry2d(pose)), fixed=True)
#
#     init_pose = np.eye(3)
#     vertex_idx = 1
#     registered_lasers = []
#     registered_idxs = []
#     registered_lasers.append(lasers[0])
#     registered_idxs.append(0)
#     vertex_id_odom_idx = []
#
#     # add odom to graph
#     id_list = []
#     for odom_idx, odom in enumerate(odoms_laser):
#         if odom_idx==0:
#             prev_odom = odom[odom_idx].copy()
#             prev_idx = 0
#             continue
#
#         do = odom - prev_odom
#         if np.linalg.norm(do[:2])>0.4 or abs(do[2])>0.2:
#
#             # (2, 180)
#             A = lasers[prev_idx]
#             B = lasers[odom_idx]
#             registered_lasers.append(B)
#             registered_idxs.append(odom_idx)
#             dx, dy, dtheta = do[0], do[1], do[2]
#             init_pose = np.array([[np.cos(dtheta), -np.sin(dtheta), dx], [np.sin(dtheta), np.cos(dtheta), dy],[0, 0, 1]])
#             with np.errstate(all='raise'):
#                 try:
#                     T, distances, iterations,information = icp(B.T, A.T, init_pose, max_iterations=20, tolerance=0.0001)
#
#
#                 except Exception as e:
#                     print(odom_idx, e, A.shape, B.shape)
#                     assert 1==0
#                     continue
#
#             pose = np.matmul(pose, T)
#             graph_optimizer.add_vertex(vertex_idx, g2o.SE2(g2o.Isometry2d(pose)))
#             vertex_id_odom_idx.append(odom_idx)
#
#             rk = g2o.RobustKernelDCS()
#
#             graph_optimizer.add_edge([vertex_idx-1, vertex_idx],
#                                      g2o.SE2(g2o.Isometry2d(T)),
#                                      information, robust_kernel=rk)
#
#             prev_odom = odom
#             prev_idx = odom_idx
#
#             # loop closure
#             if vertex_idx > 1:
#                 poses = [graph_optimizer.get_pose(idx).to_vector()[:2] for idx in range(vertex_idx-1)]
#
#                 kd = scipy.spatial.cKDTree(poses)
#                 x, y, theta = graph_optimizer.get_pose(vertex_idx).to_vector()
#                 direction = np.array([np.cos(theta), np.sin(theta)])
#                 idxs = kd.query_ball_point(np.array([x,y]), r=4.25)
#                 for idx in idxs:
#                     A = registered_lasers[idx]
#                     with np.errstate(all='raise'):
#                         try:
#                             T, distances, iterations, information = icp(A.T, B.T, np.eye(3), max_iterations=80, tolerance=0.0001)
#
#                         except Exception as e:
#                             print(odom_idx, e, A.shape, B.shape)
#                             continue
#
#                     if np.mean(distances) < thres:
#                         dist = np.linalg.norm(T[:2,2])
#                         print(odom_idx, vertex_idx, lc_num, dist, "added an edge")
#                         lc_num+=1
#
#                         # information = np.eye(3)
#                         rk = g2o.RobustKernelDCS()
#                         graph_optimizer.add_edge([vertex_idx, idx], g2o.SE2(g2o.Isometry2d(T)), information, robust_kernel=rk)
#
#                 graph_optimizer.optimize()
#                 pose = graph_optimizer.get_pose(vertex_idx).to_isometry().matrix()
#
#             vertex_idx+=1
#
#     graph_optimizer.plot(registered_idxs)
