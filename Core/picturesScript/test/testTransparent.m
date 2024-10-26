% 创建绘图
fig = figure;
ax = axes('Parent', fig);
plot(ax, rand(10, 1), 'LineWidth', 2);
hold on;
plot(ax, rand(10, 1), 'r--', 'LineWidth', 2);
xlabel(ax, 'X 轴');
ylabel(ax, 'Y 轴');
title(ax, '仅坐标区域透明的示例图');

% 设置图窗和坐标区域背景为透明
set(fig, 'Color', 'none'); % 图窗背景透明
set(ax, 'Color', 'none');  % 坐标轴背景透明

% 使用 export_fig 导出带透明背景的 PNG 图片
export_fig('output2.png', '-png', '-transparent'); % 导出为透明背景的 PNG 文件