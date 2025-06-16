def load_grid_from_txt(filename):
    grid = []
    with open(filename, 'r') as f:
        for line in f:
            # 每行用空格分割，转换成int数组
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return grid

def snake_coverage_path(grid):
    path = []
    rows = len(grid)
    cols = len(grid[0])

    for i in range(rows):
        if i % 2 == 0:
            # 偶数行，从左到右
            for j in range(cols):
                if grid[i][j] == 0:  # 可清扫区域
                    path.append((i, j))
        else:
            # 奇数行，从右到左
            for j in reversed(range(cols)):
                if grid[i][j] == 0:
                    path.append((i, j))
    return path

if __name__ == "__main__":
    filename = r"C:\Users\24816\Desktop\comprehensive project\occupancy_grid.txt"  # 修改为你的路径
    grid_map = load_grid_from_txt(filename)
    coverage_path = snake_coverage_path(grid_map)

    print("生成的蛇形覆盖路径点：")
    for p in coverage_path:
        print(p)
