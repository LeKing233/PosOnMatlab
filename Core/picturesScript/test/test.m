% 读取背景图片
backgroundImage = imread('操场地图.png'); % 替换成你想用作背景的图片路径

% 创建图窗
figure;

% 显示背景图片并调整坐标轴
imshow(backgroundImage, 'XData', [0 1], 'YData', [0 1]); % 调整坐标范围以适配
axis on; % 显示坐标轴
hold on; % 保持图窗，允许叠加其他元素

% 叠加绘制曲线或数据
x = linspace(0, 1, 100); % 假设 x 轴数据范围为 [0, 1]
y = sin(2 * pi * x); % 示例数据
plot(x, y, 'r-', 'LineWidth', 2); % 在背景图上绘制红色的正弦曲线

% 显示文字或坐标标签
xlabel('X-axis');
ylabel('Y-axis');
title('Overlay Plot with Background Image');

% 保存带有背景图的最终图像
saveas(gcf, 'output_with_background.png');
