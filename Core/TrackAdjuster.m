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


    end

end

