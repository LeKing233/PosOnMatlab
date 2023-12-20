% 清除工作区
clear;
close all;
clc;

% 指定 CSV 文件所在的目录
directoryPath = '../RawData/new1218测试时间同步/';

% 获取目录的父目录
parentDirectory = fileparts(directoryPath);

% 指定输出文件名
outputFileName = fullfile(parentDirectory, 'file_paths_output.m');

% 获取指定目录中所有 CSV 文件的列表
csvFiles = dir(fullfile(directoryPath, '*.csv'));

% 按照文件创建日期对文件列表进行排序
[~, sortedIndices] = sort([csvFiles.datenum]);
csvFiles = csvFiles(sortedIndices);

% 打开输出文件进行写入
fid = fopen(outputFileName, 'w');

% 检查文件是否成功打开
if fid == -1
    error('打开输出文件时出现错误。');
end

% 循环遍历每个 CSV 文件
for i = 1:length(csvFiles)
    % 提取文件名（不包括扩展名）
    fileName = csvFiles(i).name;
    [~, name, ~] = fileparts(fileName);
    
    % 构建文件路径
    ImuFilePath = fullfile(directoryPath, [name, '.csv']);
    PlantarFilePath = fullfile(directoryPath, [name, '.csv']);

     % 将文件路径添加到对应变量
    if startsWith(name, 'Imu_')
        fprintf(fid, 'ImuFilePath = ''%s'';\n', ImuFilePath);
    elseif startsWith(name, 'Plantar_')
        fprintf(fid, 'PlantarFilePath = ''%s'';\n', PlantarFilePath);
        % 为了更好的可读性，添加一个空行
        fprintf(fid, '\n');
    end
    
end

% 关闭输出文件
fclose(fid);

disp(['文件路径已写入: ', outputFileName]);


