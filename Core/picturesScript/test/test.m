%% 1
% 假设有一些轨迹数据
figure;
hold on;

% 绘制几条轨迹（示例数据）
plot(rand(10,1), rand(10,1), '-o', 'DisplayName', '轨迹1');
plot(rand(10,1), rand(10,1), '-x', 'DisplayName', '轨迹2');
plot(rand(10,1), rand(10,1), '-s', 'DisplayName', '轨迹3');

% 添加图例
legend show;

% 设置坐标区背景颜色为透明
set(gca, 'Color', 'none');

% 导出图像
set(gcf, 'Color', 'none'); % 设置整个图像背景透明
exportgraphics(gcf, 'transparent_trajectory.png', 'BackgroundColor', 'none');


%% 向量图 
% 创建一个新图形窗口
figure;

% 绘制几条随机轨迹
hold on;

% 绘制轨迹1
plot(rand(10,1), rand(10,1), '-o', 'DisplayName', '轨迹1', 'LineWidth', 2);
% 绘制轨迹2
plot(rand(10,1), rand(10,1), '-x', 'DisplayName', '轨迹2', 'LineWidth', 2);
% 绘制轨迹3
plot(rand(10,1), rand(10,1), '-s', 'DisplayName', '轨迹3', 'LineWidth', 2);

% 添加图例
legend show;

% 设置坐标轴背景为透明
set(gca, 'Color', 'none');
set(gcf, 'Color', 'none'); % 设置整个图形背景透明

% 导出为 PDF 向量图
print(gcf, 'trajectory_plot.pdf', '-dpdf');


%% png向量图
% 创建一个新图形窗口
figure;

% 绘制几条随机轨迹
hold on;

% 绘制轨迹1
plot(rand(10,1), rand(10,1), '-o', 'DisplayName', '轨迹1', 'LineWidth', 2);
% 绘制轨迹2
plot(rand(10,1), rand(10,1), '-x', 'DisplayName', '轨迹2', 'LineWidth', 2);
% 绘制轨迹3
plot(rand(10,1), rand(10,1), '-s', 'DisplayName', '轨迹3', 'LineWidth', 2);

% 添加图例
legend show;

% 设置坐标轴背景为透明
set(gca, 'Color', 'none');  % 设置坐标轴背景颜色为透明
set(gcf, 'Color', 'none');  % 设置整个图形背景颜色为透明

% 导出为 PNG 带透明背景
print(gcf, 'trajectory_plot.png', '-dpng', '-r300');  % 导出为 PNG 格式




%% svg

% 创建一个新图形窗口
figure;

% 绘制几条随机轨迹
hold on;

% 绘制轨迹1
plot(rand(10,1), rand(10,1), '-o', 'DisplayName', '轨迹1', 'LineWidth', 2);
% 绘制轨迹2
plot(rand(10,1), rand(10,1), '-x', 'DisplayName', '轨迹2', 'LineWidth', 2);
% 绘制轨迹3
plot(rand(10,1), rand(10,1), '-s', 'DisplayName', '轨迹3', 'LineWidth', 2);

% 添加图例
legend show;

% 设置坐标轴和图形背景为透明
% set(gca, 'Color', 'none');  % 设置坐标轴背景颜色为透明
% set(gcf, 'Color', 'none');  % 设置整个图形背景颜色为透明

% 导出为 SVG
print(gcf, 'trajectory_plot1.svg', '-dsvg');  % 导出为 SVG 格式
%% emf

% 创建一个新图形窗口
figure;

% 绘制几条随机轨迹
hold on;

% 绘制轨迹1
plot(rand(10,1), rand(10,1), '-o', 'DisplayName', '轨迹1', 'LineWidth', 2);
% 绘制轨迹2
plot(rand(10,1), rand(10,1), '-x', 'DisplayName', '轨迹2', 'LineWidth', 2);
% 绘制轨迹3
plot(rand(10,1), rand(10,1), '-s', 'DisplayName', '轨迹3', 'LineWidth', 2);

% 添加图例
legend show;

% 设置坐标轴和图形背景颜色
set(gca, 'Color', 'none');  % 设置坐标轴背景颜色为透明
set(gcf, 'Color', 'none');  % 设置整个图形背景颜色为透明

% 导出为 EMF
print(gcf, 'trajectory_plot.emf', '-dmeta');  % 导出为 EMF 格式

 
%% pdf
% 图形数量
numPlots = 3;

% 创建一个 PDF 文件的名称
pdfFilename = 'trajectory_plots.pdf';

% 循环绘制每个图形
for i = 1:numPlots
    % 创建一个新的图形窗口
    figure;
    
    % 绘制轨迹
    plot(rand(10, 1), rand(10, 1), 'DisplayName', sprintf('轨迹 %d', i));
    legend show;
    title(sprintf('轨迹图 %d', i));
    
    % 设置坐标轴背景为透明
    set(gca, 'Color', 'none');
    
    % 导出为 PDF，每次都添加到同一个文件中
    print(gcf, pdfFilename, '-dpdf', '-painters', '-bestfit');  % 导出为 PDF 格式

    % 关闭当前图形窗口
    close(gcf);
end

% 最终的 PDF 文件将包含 numPlots 页，每页一幅图。



