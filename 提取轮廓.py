import cv2
import numpy as np

def extract_and_crop_inner_region(input_image_path, output_image_path):
    # 读灰度图
    img = cv2.imread(input_image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("读取图片失败")
        return

    # 反色，方便找黑色边界线轮廓
    img_inv = cv2.bitwise_not(img)

    # 找外部轮廓（假设只有一层闭合边界线）
    contours, _ = cv2.findContours(img_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("未找到轮廓")
        return

    largest_contour = max(contours, key=cv2.contourArea)
    print("最大轮廓面积:", cv2.contourArea(largest_contour))

    # 创建掩码，填充轮廓内部区域为白色
    mask = np.zeros_like(img, dtype=np.uint8)
    cv2.drawContours(mask, [largest_contour], -1, 255, thickness=-1)

    # 用掩码提取内部区域像素，外部全黑（或全白也行，这里先黑色方便裁剪）
    extracted = cv2.bitwise_and(img, mask)

    # 找掩码中非零像素的边界框
    coords = cv2.findNonZero(mask)
    x, y, w, h = cv2.boundingRect(coords)

    # 裁剪到最大轮廓区域的最小矩形框
    cropped = extracted[y:y+h, x:x+w]

    # 保存裁剪结果
    cv2.imwrite(output_image_path, cropped)
    print(f"提取并裁剪内部区域已保存到: {output_image_path}")

if __name__ == "__main__":
    input_path = "map.png"  # 输入你的图像路径
    output_path = "map_cropped_inner_region.png"
    extract_and_crop_inner_region(input_path, output_path)
