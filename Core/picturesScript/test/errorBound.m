% 示例数据
x = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4];  % 自变量
y = [10, 12, 11, 20, 19, 21, 15, 16, 14, 25, 24, 26]; % 因变量

% 获取每个唯一的 x 值
unique_x = unique(x);

% 初始化均值、标准误差、误差带上下限
mean_y = zeros(size(unique_x));
SE_y = zeros(size(unique_x));
upper_bound = zeros(size(unique_x));
lower_bound = zeros(size(unique_x));

% 计算每个 x 值对应的 y 的均值和标准误差
for i = 1:length(unique_x)
    % 找到当前 x 值对应的所有 y 值
    y_values = y(x == unique_x(i));
    
    % 计算均值和标准误差
    mean_y(i) = mean(y_values);
    SE_y(i) = std(y_values) / sqrt(length(y_values));
    
    % 计算误差带（95% 置信区间）
    upper_bound(i) = mean_y(i) + 1.96 * SE_y(i);
    lower_bound(i) = mean_y(i) - 1.96 * SE_y(i);
end

% 绘制图形
figure;
hold on;
fill([unique_x; flipud(unique_x)], [upper_bound; flipud(lower_bound)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % 绘制误差带
plot(unique_x, mean_y, '-o', 'LineWidth', 1.5, 'Color', 'b'); % 绘制均值曲线
hold off;
xlabel('X');
ylabel('Y');
title('Error Band Plot with 95% Confidence Interval');
legend('95% Confidence Band', 'Mean');
