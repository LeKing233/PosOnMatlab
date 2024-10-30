% 定义变量 t 的范围
t = 0:0.05:10*pi;  % 0 到 2π，步长为 0.1

% 计算 y 值
y1 = sin(t);  % y1 为 sin(t)
y2 = cos(t);  % y2 为 cos(t)

% 将数据组合成一个矩阵
data = [t' y1' y2'];  % 转置为列向量并组合

% 保存数据到 CSV 文件
filename = 'sin_cos_data.csv';  % 文件名
writematrix(data, filename);  % 保存为 CSV 文件
