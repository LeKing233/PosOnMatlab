% 创建数据
x = 0:0.1:10;
y = sin(x);

% 绘制图形
figure;
plot(x, y, 'LineWidth', 2);
title('Sine Wave');
xlabel('X-axis');
ylabel('Y-axis');

% 设置背景为透明
set(gca, 'Color', 'none'); % 设置坐标轴背景透明
set(gcf, 'Color', 'none'); % 设置图形背景透明

% 导出为透明背景的 PNG 文件
print('myFigure', '-dpng', '-r300');  % 导出为 PNG 文件，300 DPI
