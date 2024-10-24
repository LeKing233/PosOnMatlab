


%% 计算所有实例
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/1015操场测试/合并/';




%方形————所有数据集
stateCalrArray_raw = [
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv'])),    
StateCalculator(ImuHandler([rootPath 'Imu_方形a2.csv']),PlantarHandler([rootPath 'Plantar_方形a2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a3.csv']),PlantarHandler([rootPath 'Plantar_方形a3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a4.csv']),PlantarHandler([rootPath 'Plantar_方形a4.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a5.csv']),PlantarHandler([rootPath 'Plantar_方形a5.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a6.csv']),PlantarHandler([rootPath 'Plantar_方形a6.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h1.csv']),PlantarHandler([rootPath 'Plantar_方形h1.csv'])),    
StateCalculator(ImuHandler([rootPath 'Imu_方形h2.csv']),PlantarHandler([rootPath 'Plantar_方形h2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h3.csv']),PlantarHandler([rootPath 'Plantar_方形h3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h4.csv']),PlantarHandler([rootPath 'Plantar_方形h4.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h5.csv']),PlantarHandler([rootPath 'Plantar_方形h5.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h6.csv']),PlantarHandler([rootPath 'Plantar_方形h6.csv'])), 
];


for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end


%方形————修改参数
stateCalrArray_modify = [
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 100,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 200,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 300,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 400,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 800,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 1000,'ZUP_noiseG', 300), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 120,'ZUP_noiseG', 300), 

StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 50),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 80),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 100),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 300),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1000),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1200),
StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1500),

];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end




Plotter.plot_Tracks_2D(stateCalrArray_raw);
Plotter.plot_Tracks_2D(stateCalrArray_modify);










beep; % 播放系统提示音 程序执行完毕时播放系统提示音


%% 测试图片旋转模块
P = stateCalrArray_raw(3).mStateSeq.P';

P_adjust = TrackAdjuster.rotate2D(P(1,:),P(2,:),40);

plot(P(1,:),P(2,:),'LineWidth',3);hold on;
plot(P_adjust(1,:),P_adjust(2,:),'LineWidth',2);hold on;
TrackAdjuster.plotTrajectory([0,0,-16,-16,0;0,19,19,0,0],'g--');

xlabel('X方向/米','FontSize', 16); % x轴注解
ylabel('Y方向/米','FontSize', 16); % y轴注解
title('二维轨迹图'); % 图形标题
grid on; % 显示格线
axis equal;


%% 测试绘制参考轨迹


TrackAdjuster.plotTrajectory([0,0,-16,-16,0;0,19,19,0,0],'g--');



%% 绘制图

addpath('..');%添加路径
% 清除工作区
rootPath = '../../RawData/1015操场测试/合并/';


%慢走
stateCalrArray_slow_walk = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a5.csv']),PlantarHandler([rootPath 'Plantar_方形a5.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a5.csv']),PlantarHandler([rootPath 'Plantar_方形a5.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 800,'ZUP_noiseG', 300), 

];


for i = 1:length(stateCalrArray_slow_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_walk(i).solveState(str);
end

gcf_slow_walk = Plotter.plot_Tracks_Square(stateCalrArray_slow_walk,'P1_angle',33,'P2_angle',33);



%快走
stateCalrArray_fast_walk = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a3.csv']),PlantarHandler([rootPath 'Plantar_方形a3.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h3.csv']),PlantarHandler([rootPath 'Plantar_方形h3.csv'])), 
];


for i = 1:length(stateCalrArray_fast_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_fast_walk(i).solveState(str);
end

gcf_fast_walk = Plotter.plot_Tracks_Square(stateCalrArray_fast_walk,'P1_angle',40,'P2_angle',50);


%慢跑
stateCalrArray_slow_run = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a2.csv']),PlantarHandler([rootPath 'Plantar_方形h2.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形h2.csv']),PlantarHandler([rootPath 'Plantar_方形h2.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 100), 
];


for i = 1:length(stateCalrArray_slow_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_run(i).solveState(str);
end

gcf_slow_run = Plotter.plot_Tracks_Square(stateCalrArray_slow_run,'P1_angle',33,'P2_angle',70);


%快跑
stateCalrArray_fast_run = [
%方形————所有数据集
StateCalculator(ImuHandler([rootPath 'Imu_方形a4.csv']),PlantarHandler([rootPath 'Plantar_方形a4.csv'])), 
StateCalculator(ImuHandler([rootPath 'Imu_方形a4.csv']),PlantarHandler([rootPath 'Plantar_方形a4.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 300,'ZUP_noiseG', 50), 
];


for i = 1:length(stateCalrArray_fast_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_fast_run(i).solveState(str);
end

gcf_fast_run = Plotter.plot_Tracks_Square(stateCalrArray_fast_run,'P1_angle',40,'P2_angle',40);



beep;


% 保存函数

% 设置保存路径
savePath = './output/'; % 你可以根据需求更改保存的路径
% 
% % 如果输出目录不存在，创建它
% if ~exist(savePath, 'dir')
%     mkdir(savePath);
% end

% 保存慢走图形
exportgraphics(gcf_slow_walk, fullfile(savePath, 'slow_walk.png'), 'Resolution', 300);

% 保存快走图形
exportgraphics(gcf_fast_walk, fullfile(savePath, 'fast_walk.png'), 'Resolution', 300);

% 保存慢跑图形
exportgraphics(gcf_slow_run, fullfile(savePath, 'slow_run.png'), 'Resolution', 300);

% 保存快跑图形
exportgraphics(gcf_fast_run, fullfile(savePath, 'fast_run.png'), 'Resolution', 300);







