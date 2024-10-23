


%% 计算所有实例
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/老数据/机械楼/';




%操场————所有数据集
stateCalrArray_raw = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第三次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第四次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


% StateCalculator(ImuHandler([rootPath 'Imu_机械楼7层1圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% 
% StateCalculator(ImuHandler([rootPath 'Imu_szp_7ceng_l.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_szp_7ceng_l.csv']), ...
% 'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% 
% StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
% 'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];

for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_raw);


%操场————修改参数
stateCalrArray_modify = [
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 400, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 300, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 100, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 400,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 500,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_机械楼2层3圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼2层3圈.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 600,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_modify);



%% 绘图
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;
rootPath = '../../RawData/老数据/机械楼/';


%操场————所有数据集
stateCalrArray_slow_walk = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_机械楼7层1圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


];

for i = 1:length(stateCalrArray_slow_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_walk(i).solveState(str);
end

Plotter.plot_Tracks_JiXieLou(stateCalrArray_slow_walk,'P1_angle',5,'P2_angle',20);



