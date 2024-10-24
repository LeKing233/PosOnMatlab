


%% 计算所有实例
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/老数据/机械楼/';




%操场————所有数据集
stateCalrArray_raw = [

% 可用
StateCalculator(ImuHandler([rootPath 'Imu_szp_7ceng_l.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_szp_7ceng_l.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


%没那么好看，但是可用
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第三次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第四次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

%反向了，不可用，而且缺个角
StateCalculator(ImuHandler([rootPath 'Imu_机械楼7层1圈.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


];

for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_raw);



%% 改参数调试第一组数据，尝试
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/老数据/机械楼/';

stateCalrArray_modify = [

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 100, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 700, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 800, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 900, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 1000, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 50, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 100, 'ZUP_noiseV', 100,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 150,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 200,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 250,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 350,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 700, 'ZUP_noiseV', 400,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 800, 'ZUP_noiseV', 450,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 900, 'ZUP_noiseV', 500,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 1000, 'ZUP_noiseV', 600,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 50, 'ZUP_noiseV', 700,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


];

for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_modify);
selectArray = [stateCalrArray_modify(6),stateCalrArray_modify(7),stateCalrArray_modify(14),stateCalrArray_modify(16)];
Plotter.plot_Tracks_2D(selectArray);




%% 修改参数得到针对第一次的四组可用参数————不好的图从这里面出
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/老数据/机械楼/';
stateCalrArray_modify = [

StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 700, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 200,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 600,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_modify);

%% 针对四组参数调试————
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPath = '../../RawData/老数据/机械楼/';


stateCalrArray_modify = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第三次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第四次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_modify);





%% 找快跑的图

% 
% rootPath = '../../RawData/老数据/机械楼/';
% stateCalrArray_modify = [
% 
% % 9 个
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 100, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 200, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 400, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 500, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 600, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 700, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 800, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 900, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% 
% % 10个
% %炸开型
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 20,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 40,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 60,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 80,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 100,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 120,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 140,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 160,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 180,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 200,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% 
% %波浪类型
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 50,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 80,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 100,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 130,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 160,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 200,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 300,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 350,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 380,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 400,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% 
% 
% ];




rootPath = '../../RawData/老数据/机械楼/';
stateCalrArray_modify = [

% 10个
%炸开型
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 20,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 40,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 60,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

%波浪类型
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 10,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 20,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 30,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 40,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 50,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];


for i = 1:length(stateCalrArray_modify)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_modify(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_modify);




%% 绘图————慢走

addpath('..');%添加路径
% 清除工作区
% clear;
close all;
% clc;
rootPath = '../../RawData/老数据/机械楼/';


stateCalrArray_slow_walk = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 320, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
];

for i = 1:length(stateCalrArray_slow_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_walk(i).solveState(str);
end


Plotter.plot_Tracks_JiXieLou(stateCalrArray_slow_walk,'P1_angle',5,'P2_angle',5);


%% 绘图————快走
addpath('..');%添加路径
% 清除工作区
% clear;
close all;
% clc;
rootPath = '../../RawData/老数据/机械楼/';


stateCalrArray_fast_walk = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 50,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),


% StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第二次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 85,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_chl_0620_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
% 'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 250,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];

for i = 1:length(stateCalrArray_fast_walk)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_fast_walk(i).solveState(str);
end


Plotter.plot_Tracks_JiXieLou(stateCalrArray_fast_walk,'P1_angle',18,'P2_angle',19);

%% 绘图————慢跑
addpath('..');%添加路径
% 清除工作区
% clear;
close all;
% clc;
rootPath = '../../RawData/老数据/机械楼/';


%操场————所有数据集
stateCalrArray_slow_run = [
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 240,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
% StateCalculator(ImuHandler([rootPath 'Imu_szp_7ceng_l.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_szp_7ceng_l.csv']), ...
% 'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 30,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
];

for i = 1:length(stateCalrArray_slow_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_slow_run(i).solveState(str);
end

Plotter.plot_Tracks_JiXieLou(stateCalrArray_slow_run,'P1_angle',5,'P2_angle',5,'refTrackIndex',1);
% Plotter.plot_Tracks_JiXieLou(stateCalrArray_slow_run,'P1_angle',-18,'P2_angle',5,'refTrackIndex',2);

%% 绘图————快跑
addpath('..');%添加路径
% 清除工作区
% clear;
close all;
% clc;
rootPath = '../../RawData/老数据/机械楼/';


%操场————所有数据集
stateCalrArray_fast_run = [
StateCalculator(ImuHandler([rootPath 'Imu_szp_7ceng_l.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_szp_7ceng_l.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 100,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPath 'Imu_chl_0621_七楼一圈第一次.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_机械楼7层1圈.csv']), ...
'ZUP_noiseF', 300, 'ZUP_noiseV', 95,'ZUP_noiseG', 20,'ZUP_noiseW', 0.6,'ZUP_noiseM', 170,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

];

for i = 1:length(stateCalrArray_fast_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_fast_run(i).solveState(str);
end

Plotter.plot_Tracks_JiXieLou(stateCalrArray_fast_run,'P1_angle',-18,'P2_angle',0,'refTrackIndex',2);
Plotter.plot_Tracks_JiXieLou_Array(stateCalrArray_fast_run,'P1_angle',-18);


beep;






%% 绘图————快跑_调试可用图
addpath('..');%添加路径
% 清除工作区
% clear;
close all;
% clc;
rootPath = '../../RawData/老数据/机械楼/';


%操场————所有数据集
stateCalrArray_fast_run = [
StateCalculator(ImuHandler([rootPath 'Imu_szp_7ceng_l.csv'],'w_origin_deviation',[0,0,0]),PlantarHandler([rootPath 'Plantar_szp_7ceng_l.csv']), ...
'AHRS', Utils.AHRS_MADGWICK, 'Aligner', Utils.AHRS_EKF,'ZUP_noiseF', 600, 'ZUP_noiseV', 100,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
];

for i = 1:length(stateCalrArray_fast_run)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_fast_run(i).solveState(str);
end

% Plotter.plot_Tracks_JiXieLou(stateCalrArray_fast_run,'P1_angle',-18,'P2_angle',0,'refTrackIndex',2);
Plotter.plot_Tracks_JiXieLou_Array(stateCalrArray_fast_run,'P1_angle',-18);