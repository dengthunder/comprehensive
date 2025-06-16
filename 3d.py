import open3d as o3d

# 加载 obj 文件
mesh = o3d.io.read_triangle_mesh("C:\\Users\\24816\\Desktop\\comprehensive project\\Environment map.obj")
mesh.compute_vertex_normals()

# 显示模型
o3d.visualization.draw_geometries([mesh])
