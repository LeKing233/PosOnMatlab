clc;
clear;

% 定义操场尺寸参数
straight_length = 84.39; % 直道长度（米）
curve_radius = 36.5; % 弯道半径（米）

% 创建新图形
figure;
hold on;
axis equal;
grid on;
title('400米标准操场外轮廓');
xlabel('米');
ylabel('米');

% 绘制操场外圈的左半圆和右半圆
theta_left = linspace(pi/2, 3*pi/2, 100);  % 左半圆角度范围
theta_right = linspace(-pi/2, pi/2, 100);  % 右半圆角度范围

% 计算左半圆的坐标
x_outer_left = curve_radius * cos(theta_left) - straight_length / 2;
y_outer_left = curve_radius * sin(theta_left);

% 计算右半圆的坐标
x_outer_right = curve_radius * cos(theta_right) + straight_length / 2;
y_outer_right = curve_radius * sin(theta_right);

% 绘制左半圆
plot(x_outer_left, y_outer_left, 'r--', 'LineWidth', 2);

% 绘制右半圆
plot(x_outer_right, y_outer_right, 'p--', 'LineWidth', 2);

% 绘制两条直道
plot([-straight_length/2, straight_length/2], [curve_radius, curve_radius], 'r-', 'LineWidth', 2);
plot([-straight_length/2, straight_length/2], [-curve_radius, -curve_radius], 'b-', 'LineWidth', 2);

% 设置显示范围
xlim([-60 60]);
ylim([-50 50]);
legend('外圈');
hold off;





%%
clc;
clear;

% 定义操场尺寸参数
straight_length = 84.39; % 直道长度（米）
curve_radius = 36.5; % 弯道半径（米）

% 设置角度范围
theta_left = linspace(pi/2, 3*pi/2, 100);  % 左半圆角度范围
theta_right = linspace(-pi/2, pi/2, 100);  % 右半圆角度范围

% 计算左半圆的坐标
x_outer_left = curve_radius * cos(theta_left) - straight_length / 2;
y_outer_left = curve_radius * sin(theta_left);

% 计算右半圆的坐标
x_outer_right = curve_radius * cos(theta_right) + straight_length / 2;
y_outer_right = curve_radius * sin(theta_right);

% 计算两条直道的坐标
x_straight_top = linspace(-straight_length/2, straight_length/2, 50);
y_straight_top = curve_radius * ones(size(x_straight_top));

x_straight_bottom = linspace(straight_length/2, -straight_length/2, 50);
y_straight_bottom = -curve_radius * ones(size(x_straight_bottom));

% 合并所有坐标
x_coords = [x_outer_left, x_straight_bottom, x_outer_right, x_straight_top];
y_coords = [y_outer_left, y_straight_bottom, y_outer_right, y_straight_top];

% 将起点平移到 (0, 0)
x_coords = x_coords - x_coords(1);
y_coords = y_coords - y_coords(1);

% 生成轨迹数据矩阵
track_data = [x_coords', y_coords'];

% 保存数据到 .mat 文件
save('track_data.mat', 'track_data');

% 或者将数据保存到 .csv 文件
csvwrite('track_data.csv', track_data);

% 绘制操场外轮廓以确认
figure;
hold on;
plot(track_data(:,1), track_data(:,2), 'b-', 'LineWidth', 2);
axis equal;
grid on;
title('400米标准操场外轮廓');
xlabel('米');
ylabel('米');
legend('外圈');
hold off;




%%

function track_data = generateTrackData(straight_length, curve_radius)
    % generateTrackData 生成400米标准操场的轨迹数据
    %   track_data = generateTrackData(straight_length, curve_radius)
    %   返回操场的轨迹矩阵数据，参数：
    %   straight_length - 直道长度（米），默认值为84.39米
    %   curve_radius - 弯道半径（米），默认值为36.5米
    
    % 设置默认参数
    if nargin < 2
        straight_length = 84.39; % 默认直道长度
        curve_radius = 36.5;      % 默认弯道半径
    end

    % 设置角度范围
    theta_left = linspace(pi/2, 3*pi/2, 100);  % 左半圆角度范围
    theta_right = linspace(-pi/2, pi/2, 100);  % 右半圆角度范围

    % 计算左半圆的坐标
    x_outer_left = curve_radius * cos(theta_left) - straight_length / 2;
    y_outer_left = curve_radius * sin(theta_left);

    % 计算右半圆的坐标
    x_outer_right = curve_radius * cos(theta_right) + straight_length / 2;
    y_outer_right = curve_radius * sin(theta_right);

    % 计算两条直道的坐标
    x_straight_top = linspace(-straight_length / 2, straight_length / 2, 50);
    y_straight_top = curve_radius * ones(size(x_straight_top));

    x_straight_bottom = linspace(straight_length / 2, -straight_length / 2, 50);
    y_straight_bottom = -curve_radius * ones(size(x_straight_bottom));

    % 合并所有坐标
    x_coords = [x_outer_left, x_straight_bottom, x_outer_right, x_straight_top];
    y_coords = [y_outer_left, y_straight_bottom, y_outer_right, y_straight_top];

    % 将起点平移到 (0, 0)
    x_coords = x_coords - x_coords(1);
    y_coords = y_coords - y_coords(1);

    % 生成轨迹数据矩阵
    track_data = [x_coords', y_coords'];

    % 可选：保存数据到 .mat 文件
    % save('track_data.mat', 'track_data');

    % 可选：将数据保存到 .csv 文件
    % csvwrite('track_data.csv', track_data);

    % 绘制操场外轮廓以确认
    figure;
    hold on;
    plot(track_data(:, 1), track_data(:, 2), 'b-', 'LineWidth', 2);
    axis equal;
    grid on;
    title('400米标准操场外轮廓');
    xlabel('米');
    ylabel('米');
    legend('外圈');
    hold off;
end





