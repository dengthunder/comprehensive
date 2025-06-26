import cv2
import numpy as np
import matplotlib.pyplot as plt

def image_to_occupancy_grid(image_path, threshold=127):
    """
    读取黑白地图图片，黑色为障碍(1)，白色为通行(0)
    """
    img_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img_gray is None:
        raise FileNotFoundError(f"无法读取图片: {image_path}")
    # 普通二值化，黑色(0)会变成0，白色(255)变成255
    _, binary_img = cv2.threshold(img_gray, threshold, 255, cv2.THRESH_BINARY)

    # 把黑色（像素值0）映射为1（障碍），白色（255）映射为0（空闲）
    occupancy_grid = np.where(binary_img == 0, 1, 0).astype(np.uint8)

    return occupancy_grid

if __name__ == "__main__":
    img_path = r'C:\Users\24816\Desktop\project\shange\map_cropped_inner_region.png'  # 你的图片路径
    occupancy_grid = image_to_occupancy_grid(img_path)
    print("障碍栅格大小:", occupancy_grid.shape)
    print(occupancy_grid)

    # 显示栅格图，黑色为障碍，白色为空闲
    plt.imshow(occupancy_grid, cmap='gray_r') #gray映射和实际映射相反，所以要进行反转
    plt.title('Occupancy Grid (1=Obstacle, 0=Free)')
    plt.show()
    np.savetxt("occupancy_grid.txt", occupancy_grid, fmt='%d', delimiter=' ')