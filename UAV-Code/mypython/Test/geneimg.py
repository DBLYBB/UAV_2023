from PIL import Image, ImageDraw

# 创建 960x800 的白色底图
img = Image.new('RGB', (960,800), 'white')

# 获取 ImageDraw 对象
draw = ImageDraw.Draw(img)

# 在 x 和 y 方向分别每隔 10 个像素绘制一条天蓝色线条
for x in range(0, 961, 20):
    if x % 160 == 0:
        draw.line((x, 0, x, 800), fill='skyblue', width=3)
    else:
        draw.line((x, 0, x, 800), fill='skyblue', width=1)

for y in range(0, 801, 20):
    if y % 160 == 0:
        draw.line((0, y, 960, y), fill='skyblue', width=3)
    else:
        draw.line((0, y, 960, y), fill='skyblue', width=1)

# 保存图片
img.save('base.png')
