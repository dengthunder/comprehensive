import trimesh
import numpy as np
import cv2


def obj_to_2d_image(obj_path, image_size=(512, 512), output_path="map.png"):
    # 加载obj文件
    scene_or_mesh = trimesh.load(obj_path)

    # 如果是Scene则合并所有Mesh
    if isinstance(scene_or_mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(scene_or_mesh.dump())
    else:
        mesh = scene_or_mesh

    vertices = mesh.vertices  # 顶点坐标 Nx3
    faces = mesh.faces  # 面索引 Mx3

    # 打印坐标范围，方便调试
    x_min, y_min, z_min = vertices.min(axis=0)
    x_max, y_max, z_max = vertices.max(axis=0)
    print(f"x范围: {x_min} 到 {x_max}")
    print(f"y范围: {y_min} 到 {y_max}")
    print(f"z范围: {z_min} 到 {z_max}")

    # 取XY坐标
    xy = vertices[:, :2]

    # 坐标归一化处理：先平移到(0,0)
    xy -= xy.min(axis=0)
    max_range = xy.max(axis=0)
    max_range[max_range == 0] = 1.0  # 防止除零

    # 计算缩放比例（保持长宽比）
    scale_x = (image_size[0] - 1) / max_range[0]
    scale_y = (image_size[1] - 1) / max_range[1]
    scale = min(scale_x, scale_y)

    # 坐标放缩并转为整数像素坐标
    xy_img = xy * scale
    # y轴反转处理（图像坐标y = 高度 - y）
    xy_img[:, 1] = (image_size[1] - 1) - xy_img[:, 1]
    xy_img = xy_img.astype(np.int32)

    # 创建白色背景图像
    img = 255 * np.ones(image_size, dtype=np.uint8)

    # 内部填充为黑色线条
    for face in faces:
        pts = xy_img[face]  # 3x2数组
        cv2.fillPoly(img, [pts], color=0)


    # 保存图像
    cv2.imwrite(output_path, img)
    print(f"二维地图已保存到: {output_path}")


if __name__ == "__main__":
    obj_file_path = r"C:\Users\24816\Desktop\comprehensive project\Environment map.obj"  # 修改为你的obj路径
    output_img_path = "map.png"  # 保存文件名
    obj_to_2d_image(obj_file_path, image_size=(512, 512), output_path=output_img_path)
