function plotTrajectoryWithBackground(imageFile, rotationAngle, xOffset, yOffset, axisLimits, scale)
    % imageFile: 图像文件的路径
    % rotationAngle: 图片旋转角度（度数）
    % xOffset, yOffset: 图片在xy方向上的偏移
    % axisLimits: 坐标系范围, 格式为 [xmin xmax ymin ymax]
    % scale: 图片缩放系数

    % 读取图像文件
    img = imread(imageFile);
    
    % 旋转图片
    rotatedImg = imrotate(img, rotationAngle, 'bilinear', 'crop');
    
    % 获取图片大小
    [imgHeight, imgWidth, ~] = size(rotatedImg);
    
    % 缩放图片
    scaledImg = imresize(rotatedImg, scale);
    [scaledHeight, scaledWidth, ~] = size(scaledImg);
    
    % 创建图窗
    figure;
    hold on;
    
    % 设置固定坐标系范围
    axis(axisLimits);
    axis equal;
    
    % 计算图片在坐标系中的位置
    xImage = [axisLimits(1) + xOffset, axisLimits(1) + xOffset + scaledWidth];
    yImage = [axisLimits(3) + yOffset, axisLimits(3) + yOffset + scaledHeight];
    
    % 显示图片作为背景
    imagesc(xImage, yImage, flipud(scaledImg));
    set(gca, 'YDir', 'normal'); % 保持 y 轴方向一致

    % 设置透明度
    alpha(0.5); % 如果需要透明效果可以调整此值

    % 示例轨迹（可以替换为实际数据）
    t = linspace(0, 2*pi, 100);
    xTrajectory = (axisLimits(2) - axisLimits(1)) / 2 * cos(t) + (axisLimits(1) + axisLimits(2)) / 2;
    yTrajectory = (axisLimits(4) - axisLimits(3)) / 2 * sin(t) + (axisLimits(3) + axisLimits(4)) / 2;
    
    % 绘制轨迹
    plot(xTrajectory, yTrajectory, 'r', 'LineWidth', 2);
    
    hold off;
end
