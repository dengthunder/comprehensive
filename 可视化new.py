import trimesh
import numpy as np
import cv2

def obj_to_2d_image(obj_path, image_size=(512, 512), output_path="map.png", margin=10):
    # 加载obj文件
    scene_or_mesh = trimesh.load(obj_path)

    # 如果是Scene则合并所有Mesh
    if isinstance(scene_or_mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(scene_or_mesh.dump())
    else:
        mesh = scene_or_mesh

    vertices = mesh.vertices  # Nx3
    faces = mesh.faces  # Mx3

    # 打印坐标范围
    x_min, y_min, z_min = vertices.min(axis=0)
    x_max, y_max, z_max = vertices.max(axis=0)
    print(f"x范围: {x_min} 到 {x_max}")
    print(f"y范围: {y_min} 到 {y_max}")
    print(f"z范围: {z_min} 到 {z_max}")

    # 取XY坐标
    xy = vertices[:, :2]

    # 计算模型宽高
    width = x_max - x_min
    height = y_max - y_min

    # 计算缩放比例，使模型在画布内，留margin边距
    scale_x = (image_size[0] - 2 * margin) / width if width != 0 else 1.0
    scale_y = (image_size[1] - 2 * margin) / height if height != 0 else 1.0
    scale = min(scale_x, scale_y)  # 保持比例，适应画布

    # 归一化坐标：先减去最小值，然后缩放
    xy_normalized = (xy - np.array([x_min, y_min])) * scale

    # 计算偏移，保证图形居中
    new_width = width * scale
    new_height = height * scale
    offset_x = (image_size[0] - new_width) / 2
    offset_y = (image_size[1] - new_height) / 2

    xy_img = xy_normalized + np.array([offset_x, offset_y])

    # 图像坐标y轴向下，需要翻转y
    xy_img[:, 1] = image_size[1] - xy_img[:, 1]

    # 四舍五入为整数
    xy_img = np.round(xy_img).astype(np.int32)

    print("映射到图像后坐标范围:")
    print(f"x: {xy_img[:,0].min()} 到 {xy_img[:,0].max()}")
    print(f"y: {xy_img[:,1].min()} 到 {xy_img[:,1].max()}")

    # 创建白色背景图
    img = 255 * np.ones(image_size, dtype=np.uint8)

    # 绘制多边形填充黑色
    for face in faces:
        pts = xy_img[face]
        cv2.fillPoly(img, [pts], color=0)

    # 保存图像
    cv2.imwrite(output_path, img)
    print(f"二维地图已保存到: {output_path}")


if __name__ == "__main__":
    obj_file_path = r"C:\Users\24816\Desktop\comprehensive project\Environment map.obj"  # 修改成你的路径
    output_img_path = "map.png"
    obj_to_2d_image(obj_file_path, image_size=(512, 512), output_path=output_img_path)