


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
Plotter.plot_Tracks_2D(stateCalrArray_modify);










beep; % 播放系统提示音 程序执行完毕时播放系统提示音


%% 绘制图

addpath('..');%添加路径
% 清除工作区
rootPath = '../../RawData/1015操场测试/合并/';


%走路
stateCalrArray_walk = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a5.csv']),PlantarHandler([rootPath 'Plantar_方形a5.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a5.csv']),PlantarHandler([rootPath 'Plantar_方形a5.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 800,'ZUP_noiseG', 300), 

];


for i = 1:length(stateCalrArray_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_walk(i).solveState(str);
end

gcf_slow_walk = Plotter.plot_Tracks_Square(stateCalrArray_walk,'P1_angle',33,'P2_angle',33);



%跑步
stateCalrArray_run = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a3.csv']),PlantarHandler([rootPath 'Plantar_方形a3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h3.csv']),PlantarHandler([rootPath 'Plantar_方形h3.csv'])), 
];


for i = 1:length(stateCalrArray_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_run(i).solveState(str);
end

gcf_fast_walk = Plotter.plot_Tracks_Square(stateCalrArray_run,'P1_angle',40,'P2_angle',50);


%混合
stateCalrArray_multiple_motion = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a2.csv']),PlantarHandler([rootPath 'Plantar_方形h2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h2.csv']),PlantarHandler([rootPath 'Plantar_方形h2.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 100), 
];


for i = 1:length(stateCalrArray_multiple_motion)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_multiple_motion(i).solveState(str);
end

gcf_slow_run = Plotter.plot_Tracks_Square(stateCalrArray_multiple_motion,'P1_angle',33,'P2_angle',70);




beep;










