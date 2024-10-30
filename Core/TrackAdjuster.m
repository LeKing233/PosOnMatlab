classdef TrackAdjuster
    %TRACKADJUSTER 轨迹调整器

    properties(Constant)

    end

    properties

    end

    methods
        function obj = TrackAdjuster()

        end
    end


    methods(Static)

        function P = translate2D(x, y, tx, ty)
            % TRANSLATE2D 对二维轨迹进行平移
            % 输入参数:
            %   - x, y: 轨迹的原始 x 和 y 坐标，向量形式
            %   - tx: x 方向的平移量
            %   - ty: y 方向的平移量
            % 输出参数:
            %   - P: 平移后的坐标矩阵，第一行是 x，第二行是 y

            % 进行平移
            x_translated = x + tx;
            y_translated = y + ty;

            % 返回平移后的坐标矩阵
            P = [x_translated; y_translated];
        end

        function P = rotate2D(x, y, angle, cx, cy)
            % ROTATE2D 对二维轨迹进行旋转
            % 输入参数:
            %   - x, y: 轨迹的原始 x 和 y 坐标，向量形式
            %   - angle: 旋转角度，单位为角度，逆时针
            %   - cx, cy: 旋转中心的 x 和 y 坐标（默认为 [0, 0]）
            % 输出参数:
            %   - P: 旋转后的坐标矩阵，第一行是 x，第二行是 y

            % 如果未指定旋转中心，则默认使用 (0, 0)
            if nargin < 4
                cx = 0;
                cy = 0;
            end

            % 创建旋转矩阵
            angle = angle / 180 * pi;%输入为角度，改成弧度
            rotation_matrix = [cos(angle), -sin(angle); sin(angle), cos(angle)];

            % 将 x 和 y 组合成 2xN 矩阵
            coordinates = [x - cx; y - cy];  % 平移到原点

            % 进行矩阵乘法，旋转坐标
            rotated_coordinates = rotation_matrix * coordinates;

            % 恢复到原来的位置
            x_rotated = rotated_coordinates(1, :) + cx;
            y_rotated = rotated_coordinates(2, :) + cy;
            P = [x_rotated; y_rotated];
        end


        function plotTrajectory(points, style)
            % PLOTTRAJECTORY 绘制连成线的轨迹图
            % 输入参数:
            %   - points: 2xN 矩阵，第一行是 x 坐标，第二行是 y 坐标
            %   - style: 字符串，指定绘图样式（例如 'r-', 'g--' 等）

            % 确保 points 是 2xN 矩阵
            if size(points, 1) ~= 2
                error('points 必须是 2xN 矩阵');
            end

            % 提取 x 和 y 坐标
            x = points(1, :);
            y = points(2, :);

            % 绘制轨迹
            plot(x, y, style, 'LineWidth', 2);
            xlabel('X方向');
            ylabel('Y方向');
            title('轨迹图');
            grid on; % 显示网格
            axis equal; % 保持坐标轴比例
        end





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

            % 可选：绘制操场外轮廓以确认
%             figure;
%             hold on;
%             plot(track_data(:, 1), track_data(:, 2), 'b-', 'LineWidth', 2);
%             axis equal;
%             grid on;
%             title('400米标准操场外轮廓');
%             xlabel('米');
%             ylabel('米');
%             legend('外圈');
%             hold off;
        end



    end

end

