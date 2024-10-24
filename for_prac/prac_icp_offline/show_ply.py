import open3d as o3d

# .ply 파일을 읽기
pcd = o3d.io.read_point_cloud("icp_source_transformed_step_1724857696217586114.ply")
pcd2 = o3d.io.read_point_cloud("icp_target_step_1724857696217586114.ply")
# 포인트 클라우드 시각화
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd2])

