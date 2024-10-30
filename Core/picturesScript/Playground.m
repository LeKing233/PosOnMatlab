


%% 计算所有实例
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/1015操场测试/合并/';




%操场————所有数据集
stateCalrArray_raw = [
StateCalculator(ImuHandler([rootPath 'Imu_操场2.csv']),PlantarHandler([rootPath 'Plantar_操场2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场3.csv']),PlantarHandler([rootPath 'Plantar_操场3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场4a.csv']),PlantarHandler([rootPath 'Plantar_操场4a.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场4h.csv']),PlantarHandler([rootPath 'Plantar_操场4h.csv'])), 
];

for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_raw);






%操场————修改参数
stateCalrArray_modify = [


];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end




Plotter.plot_Tracks_2D(stateCalrArray_raw);
% Plotter.plot_Tracks_2D(stateCalrArray_modify);










beep; % 播放系统提示音 程序执行完毕时播放系统提示音


%% 绘制图
addpath('..');%添加路径
% 清除工作区
% clear;
% close all;
% clc;

rootPath = '../../RawData/1015操场测试/合并/';




%操场————所有数据集
stateCalrArray_slow_walk = [
StateCalculator(ImuHandler([rootPath 'Imu_操场2.csv']),PlantarHandler([rootPath 'Plantar_操场2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场3.csv']),PlantarHandler([rootPath 'Plantar_操场3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场4a.csv']),PlantarHandler([rootPath 'Plantar_操场4a.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_操场4h.csv']),PlantarHandler([rootPath 'Plantar_操场4h.csv'])), 
];

for i = 1:length(stateCalrArray_slow_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_walk(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_slow_walk);



Plotter.plot_Tracks_Playground(stateCalrArray_slow_walk,'P1_angle',30,'P2_angle',48);


%%

% 调用plot_Tracks_Playground函数，获取图窗句柄gcf
fig = Plotter.plot_Tracks_Playground(stateCalrArray_slow_walk, 'P1_angle', 50, 'P2_angle', 50);

% 获取当前坐标轴句柄
ax = gca;

% 设置坐标轴和图窗背景为透明
set(ax, 'Color', 'none');
set(fig, 'Color', 'none');

% 设置保存路径
savePath = './output/';
% 自定义纸张大小
fig.PaperUnits = 'inches';            % 设置单位为英寸
fig.PaperSize = [8, 6];               % 设置纸张大小为 8x6 英寸
fig.PaperPosition = [0, 0, 8, 6];     % 设置图形在纸张上的位置和大小

% 导出图形为PDF格式
print(fig, fullfile(savePath, 'trajectory_plot.pdf'), '-dpdf');










%% 1029测试结果





addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/1029晚上操场测试/';





%操场————所有数据集
stateCalrArray_raw = [
StateCalculator(ImuHandler([rootPath 'Imu_操场11.csv'],'gyroFilePath',[rootPath 'Imu_静止1.csv'],'magFilePath',[rootPath 'Imu_八字1.csv']),PlantarHandler([rootPath 'Plantar_操场11.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 
StateCalculator(ImuHandler([rootPath 'Imu_操场12.csv'],'gyroFilePath',[rootPath 'Imu_静止1.csv'],'magFilePath',[rootPath 'Imu_八字1.csv']),PlantarHandler([rootPath 'Plantar_操场12.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 
StateCalculator(ImuHandler([rootPath 'Imu_操场13.csv'],'gyroFilePath',[rootPath 'Imu_静止1.csv'],'magFilePath',[rootPath 'Imu_八字1.csv']),PlantarHandler([rootPath 'Plantar_操场13.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3.csv'],'magFilePath',[rootPath 'Imu_八字3.csv']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 
StateCalculator(ImuHandler([rootPath 'Imu_操场32.csv'],'gyroFilePath',[rootPath 'Imu_静止3.csv'],'magFilePath',[rootPath 'Imu_八字3.csv']),PlantarHandler([rootPath 'Plantar_操场32.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 


StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3.csv'],'magFilePath',[rootPath 'Imu_八字3.csv']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450), 

];

for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_raw);


meanWseq = mean(stateCalrArray_raw(4).iHandler.mWSeq(:,1:500)')

%操场————修改参数
stateCalrArray_modify = [



StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3.csv'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 10,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 20,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 30,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 40,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 60,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 5,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 100,'EKF_noiseW', 0.4), 



StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 600,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 700,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_操场31.csv'],'gyroFilePath',[rootPath 'Imu_静止3'],'magFilePath',[rootPath 'Imu_八字3']),PlantarHandler([rootPath 'Plantar_操场31.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 800,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end




Plotter.plot_Tracks_2D(stateCalrArray_modify);






