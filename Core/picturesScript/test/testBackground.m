% === Script: plotTrajectoryWithBackgroundScript.m ===
% close all;
% clc;
% clear;
% % 参数设置
% imageFile = '操场卫星图-正-小尺寸.png';  % 图片文件路径
% rotationAngle = 0;            % 图片旋转角度（度数）
% xOffset = 10;                  % 图片在 x 方向的偏移
% yOffset = 20;                  % 图片在 y 方向的偏移
% axisLimits = [0 100 0 100];    % 坐标系范围 [xmin xmax ymin ymax]
% scale = 1;                   % 图片缩放系数
% 
% % 读取图像文件
% img = imread(imageFile);
% 
% % 旋转图片
% rotatedImg = imrotate(img, rotationAngle, 'bilinear', 'crop');
% 
% % 获取图片大小
% [imgHeight, imgWidth, ~] = size(rotatedImg);
% 
% % 缩放图片
% scaledImg = imresize(rotatedImg, scale);
% [scaledHeight, scaledWidth, ~] = size(scaledImg);
% 
% % 创建图窗
% figure;
% hold on;
% 
% % 设置固定坐标系范围
% axis(axisLimits);
% axis equal;
% 
% % 计算图片在坐标系中的位置
% xImage = [axisLimits(1) + xOffset, axisLimits(1) + xOffset + scaledWidth];
% yImage = [axisLimits(3) + yOffset, axisLimits(3) + yOffset + scaledHeight];
% 
% % 显示图片作为背景
% imagesc(xImage, yImage, flipud(scaledImg));
% set(gca, 'YDir', 'normal'); % 保持 y 轴方向一致
% 
% % 设置透明度
% alpha(0.5); % 可调整透明度
% 
% % 示例轨迹（替换为实际数据）
% t = linspace(0, 2*pi, 100);
% xTrajectory = (axisLimits(2) - axisLimits(1)) / 2 * cos(t) + (axisLimits(1) + axisLimits(2)) / 2;
% yTrajectory = (axisLimits(4) - axisLimits(3)) / 2 * sin(t) + (axisLimits(3) + axisLimits(4)) / 2;
% 
% % 绘制轨迹
% plot(xTrajectory, yTrajectory, 'r', 'LineWidth', 2);
% 
% hold off;






% 参数设置
% imageFile = 'background.png';  % 图片文件路径
% rotationAngle = 45;            % 图片旋转角度（度数）
% xOffset = 10;                  % 图片在 x 方向的偏移
% yOffset = 20;                  % 图片在 y 方向的偏移
% axisLimits = [0 100 0 100];    % 坐标系范围 [xmin xmax ymin ymax]
% scale = 0.8;                   % 图片缩放系数
% 
% % 读取和旋转图片
% img = imread(imageFile);
% rotatedImg = imrotate(img, rotationAngle, 'bilinear', 'crop');
% 
% % 缩放图片
% scaledImg = imresize(rotatedImg, scale);
% 
% % 创建图窗并设置坐标范围
% figure;
% hold on;
% axis(axisLimits);
% axis equal;
% 
% % 使用 imshow 将图像叠加为背景
% xRange = [axisLimits(1) + xOffset, axisLimits(1) + xOffset + size(scaledImg, 2)];
% yRange = [axisLimits(3) + yOffset, axisLimits(3) + yOffset + size(scaledImg, 1)];
% imshow(flipud(scaledImg), 'XData', xRange, 'YData', yRange);
% set(gca, 'YDir', 'normal'); % 保持 y 轴方向一致
% 
% % 画出示例轨迹
% t = linspace(0, 2*pi, 100);
% xTrajectory = (axisLimits(2) - axisLimits(1)) / 2 * cos(t) + (axisLimits(1) + axisLimits(2)) / 2;
% yTrajectory = (axisLimits(4) - axisLimits(3)) / 2 * sin(t) + (axisLimits(3) + axisLimits(4)) / 2;
% plot(xTrajectory, yTrajectory, 'r', 'LineWidth', 2);
% 
% hold off;




imageFile = '操场卫星图-正-小尺寸.png';  % 图片文件路径
rotationAngle = 45;            % 图片旋转角度（度数）
xOffset = 10;                  % 图片在 x 方向的偏移
yOffset = 20;                  % 图片在 y 方向的偏移
axisLimits = [0 100 0 100];    % 坐标系范围 [xmin xmax ymin ymax]
scale = 1;                   % 图片缩放系数
% 读取和旋转图片
img = imread(imageFile);
rotatedImg = imrotate(img, rotationAngle, 'bilinear', 'crop');
scaledImg = imresize(rotatedImg, scale);

% 设置图窗和坐标范围
figure;
hold on;
axis(axisLimits);
axis equal;

% 使用 image 叠加背景图片
xImage = [axisLimits(1) + xOffset, axisLimits(1) + xOffset + size(scaledImg, 2)];
yImage = [axisLimits(3) + yOffset, axisLimits(3) + yOffset + size(scaledImg, 1)];
image(xImage, yImage, flipud(scaledImg));
set(gca, 'YDir', 'normal');

% 画出示例轨迹
plot(xTrajectory, yTrajectory, 'r', 'LineWidth', 2);

hold off;




% === End of Script ===
