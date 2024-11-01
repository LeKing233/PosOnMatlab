
addpath('..');%添加路径
% 清除工作区
clear;
close all;
clc;

rootPathOld = '../../RawData/1015操场测试/合并/';
rootPath = '../../RawData/1029晚上操场测试/';


stateCalrArray_raw = [
% V:1-300无明显变化，1000开始出现波浪，
% F:600为合适，100,1000都会出现锯齿,1000的锯齿较为真实
% G:300为合适，100,1000都是飘，1000会小范围飘
% W：0.02合适

%快走
%Old
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a31.csv']),PlantarHandler([rootPathOld 'Plantar_直线a31.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a32.csv']),PlantarHandler([rootPathOld 'Plantar_直线a32.csv']), ...
'ZUP_noiseF', 550, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a33.csv']),PlantarHandler([rootPathOld 'Plantar_直线a33.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a34.csv']),PlantarHandler([rootPathOld 'Plantar_直线a34.csv']), ...
'ZUP_noiseF', 550, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a41.csv']),PlantarHandler([rootPathOld 'Plantar_直线a41.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a42.csv']),PlantarHandler([rootPathOld 'Plantar_直线a42.csv']), ...
'ZUP_noiseF', 550, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a43.csv']),PlantarHandler([rootPathOld 'Plantar_直线a43.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a44.csv']),PlantarHandler([rootPathOld 'Plantar_直线a44.csv']), ...
'ZUP_noiseF', 550, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPathOld 'Imu_直线a31.csv']),PlantarHandler([rootPathOld 'Plantar_直线a31.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a32.csv']),PlantarHandler([rootPathOld 'Plantar_直线a32.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a33.csv']),PlantarHandler([rootPathOld 'Plantar_直线a33.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a34.csv']),PlantarHandler([rootPathOld 'Plantar_直线a34.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a41.csv']),PlantarHandler([rootPathOld 'Plantar_直线a41.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a42.csv']),PlantarHandler([rootPathOld 'Plantar_直线a42.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a43.csv']),PlantarHandler([rootPathOld 'Plantar_直线a43.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线a44.csv']),PlantarHandler([rootPathOld 'Plantar_直线a44.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),

StateCalculator(ImuHandler([rootPathOld 'Imu_直线h31.csv']),PlantarHandler([rootPathOld 'Plantar_直线h31.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 

StateCalculator(ImuHandler([rootPathOld 'Imu_直线h33.csv']),PlantarHandler([rootPathOld 'Plantar_直线h33.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h34.csv']),PlantarHandler([rootPathOld 'Plantar_直线h34.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h41.csv']),PlantarHandler([rootPathOld 'Plantar_直线h41.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h42.csv']),PlantarHandler([rootPathOld 'Plantar_直线h42.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h43.csv']),PlantarHandler([rootPathOld 'Plantar_直线h43.csv']), ...
'ZUP_noiseF', 500, 'ZUP_noiseV', 300,'ZUP_noiseG', 200,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 

StateCalculator(ImuHandler([rootPathOld 'Imu_直线h31.csv']),PlantarHandler([rootPathOld 'Plantar_直线h31.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h33.csv']),PlantarHandler([rootPathOld 'Plantar_直线h33.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h34.csv']),PlantarHandler([rootPathOld 'Plantar_直线h34.csv']), ...
'ZUP_noiseF', 550, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4),
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h41.csv']),PlantarHandler([rootPathOld 'Plantar_直线h41.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPathOld 'Imu_直线h43.csv']),PlantarHandler([rootPathOld 'Plantar_直线h43.csv']), ...
'ZUP_noiseF', 200, 'ZUP_noiseV', 300,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 


%1号板
StateCalculator(ImuHandler([rootPath 'Imu_直线1d10.csv']),PlantarHandler([rootPath 'Plantar_直线1d10.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d11.csv']),PlantarHandler([rootPath 'Plantar_直线1d11.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d12.csv']),PlantarHandler([rootPath 'Plantar_直线1d12.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d14.csv']),PlantarHandler([rootPath 'Plantar_直线1d14.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d15.csv']),PlantarHandler([rootPath 'Plantar_直线1d15.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d16.csv']),PlantarHandler([rootPath 'Plantar_直线1d16.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d17.csv']),PlantarHandler([rootPath 'Plantar_直线1d17.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d18.csv']),PlantarHandler([rootPath 'Plantar_直线1d18.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d19.csv']),PlantarHandler([rootPath 'Plantar_直线1d19.csv']), ...
'ZUP_noiseF', 400, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d20.csv']),PlantarHandler([rootPath 'Plantar_直线1d20.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 200,'ZUP_noiseG', 300,'ZUP_noiseW', 0.04,'ZUP_noiseM', 450,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 


%添加
StateCalculator(ImuHandler([rootPath 'Imu_直线1d15.csv']),PlantarHandler([rootPath 'Plantar_直线1d15.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 1500,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 10,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线1d16.csv']),PlantarHandler([rootPath 'Plantar_直线1d16.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 1500,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 10,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 
StateCalculator(ImuHandler([rootPath 'Imu_直线3d17.csv']),PlantarHandler([rootPath 'Plantar_直线3d17.csv']), ...
'ZUP_noiseF', 600, 'ZUP_noiseV', 1500,'ZUP_noiseG', 300,'ZUP_noiseW', 0.02,'ZUP_noiseM', 10,'EKF_noiseF', 30,'EKF_noiseM', 50,'EKF_noiseW', 0.4), 





];

for i = 1:length(stateCalrArray_raw)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray_raw(i).solveState(str);
end

Plotter.plot_Tracks_2D(stateCalrArray_raw);


resMat = GraphCalculator.getLineErrorResult(stateCalrArray_raw,stateCalrArray_raw);

Plotter.plot_Tracks_Line(stateCalrArray_raw);


% 保存为 CSV 文件
writematrix(resMat, 'resMat.csv');






beep;






