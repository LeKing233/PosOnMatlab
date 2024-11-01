% 数据准备
x = 1:4; % X 轴数据
y1 = [3, 5, 2, 8]; % 柱状图的 Y 值
y2 = [4, 6, 3, 7]; % 折线图的 Y 值
error1 = [0.5, 0.7, 0.4, 0.6]; % 柱状图的误差
error2 = [0.3, 0.5, 0.2, 0.4]; % 折线图的误差

% 创建图形
figure;

% 第一个 Y 轴（柱状图）
yyaxis left
bar(x, y1, 'FaceColor', 'b', 'FaceAlpha', 0.5); % 绘制柱状图
hold on;
errorbar(x, y1, error1, 'Color', 'k', 'LineStyle', '--', 'Marker', 'o', 'MarkerSize', 8); % 修改误差棒样式

% 设置左侧 Y 轴标签
ylabel('Proposed Method Values');
title('Double Y-Axis Plot with Customized Error Bars');

% 第二个 Y 轴（折线图）
yyaxis right
plot(x, y2, '-o', 'Color', 'r', 'MarkerSize', 8, 'LineWidth', 2); % 绘制折线图
hold on;
errorbar(x, y2, error2, 'Color', 'r', 'LineStyle', ':', 'Marker', 'x', 'MarkerSize', 10); % 修改误差棒样式

% 设置右侧 Y 轴标签
ylabel('Traditional Method Values');

% 设置 X 轴标签和图例
xlabel('Data Points');
legend('Proposed Method', 'Proposed Method Error', 'Traditional Method', 'Traditional Method Error');

% 显示网格
grid on;

% 调整坐标轴
axis tight;
