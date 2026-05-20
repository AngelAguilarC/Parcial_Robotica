# 导入所需的库
import cv2
import numpy as np

# 读取彩色图像文件
color = cv2.imread("color.jpg")

# 读取深度图像文件
depth = cv2.imread("depth.jpg", cv2.IMREAD_UNCHANGED)

# 对彩色图像进行高斯滤波，以去除噪声
color = cv2.GaussianBlur(color, (5, 5), 0)

# 对深度图像进行中值滤波，以去除噪声
depth = cv2.medianBlur(depth, 5)

# 对彩色图像进行灰度化，以便于分割
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

# 对灰度图像进行二值化，以提取方块的轮廓
_, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

# 对二值图像进行开运算，以去除小的噪点
kernel = np.ones((3, 3), np.uint8)
thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

# 对二值图像进行轮廓检测，以提取方块的轮廓
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 假设只有一个方块，取第一个轮廓为方块的轮廓
cnt = contours[0]

# 对方块的轮廓进行多边形拟合，以得到方块的四个角点
epsilon = 0.01 * cv2.arcLength(cnt, True)
approx = cv2.approxPolyDP(cnt, epsilon, True)

# 对方块的四个角点进行排序，以得到左上、右上、右下、左下的顺序
approx = approx.reshape(4, 2)
rect = np.zeros((4, 2), dtype="float32")
s = approx.sum(axis=1)
rect[0] = approx[np.argmin(s)]
rect[2] = approx[np.argmax(s)]
diff = np.diff(approx, axis=1)
rect[1] = approx[np.argmin(diff)]
rect[3] = approx[np.argmax(diff)]

# 计算方块的长和宽（像素值）
(tl, tr, br, bl) = rect
widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
maxWidth = max(int(widthA), int(widthB))
maxHeight = max(int(heightA), int(heightB))

# 计算方块的正视图的四个角点
dst = np.array([
    [0, 0],
    [maxWidth - 1, 0],
    [maxWidth - 1, maxHeight - 1],
    [0, maxHeight - 1]], dtype="float32")

# 计算方块的透视变换矩阵
M = cv2.getPerspectiveTransform(rect, dst)

# 对彩色图像进行透视变换，以得到方块的正视图
warped = cv2.warpPerspective(color, M, (maxWidth, maxHeight))

# 对深度图像进行透视变换，以得到方块的正视图
warped_depth = cv2.warpPerspective(depth, M, (maxWidth, maxHeight))

# 计算方块的平均深度值（像素值）
mean_depth = cv2.mean(warped_depth)[0]

# 假设深度相机的内参已知，根据像素值计算方块的实际尺寸（毫米值）
# 这里使用的是Intel Realsense D435i的内参，您需要根据您的深度相机的内参进行修改
fx = 616.344 # 焦距
cx = 321.886 # 主点横坐标
cy = 238.183 # 主点纵坐标
# 计算方块的实际长和宽（毫米值）
real_width = maxWidth * mean_depth / fx
real_height = maxHeight * mean_depth / fx
# 计算方块的实际深度（毫米值）
real_depth = mean_depth
# 计算方块的体积（立方厘米值）
volume = real_width * real_height * real_depth / 1000

# 显示结果
print("The width of the cube is {:.2f} mm".format(real_width))
print("The height of the cube is {:.2f} mm".format(real_height))
print("The depth of the cube is {:.2f} mm".format(real_depth))
print("The volume of the cube is {:.2f} cm^3".format(volume))

# 显示彩色图像和深