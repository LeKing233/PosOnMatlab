% 清除工作区
clear;
close all;
clc;

%% 读取数据

profile clear % 清除之前的分析信息
profile on % 启用性能分析



% % 文件路径
% imuFilePaths = {
%     '..\RawData\new1214_健身房测试\Imu_0kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_1kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_2kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_3kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_4kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_5kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_6kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_7kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_8kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_9kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_10kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_11kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_12kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_1kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_2kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_3kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_4kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_4.2kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_4.4kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_4.6kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_4.8kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_5kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_5.2kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_5.4kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_5.6kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_5.8kmh.csv',
%     '..\RawData\new1214_健身房测试\Imu_6kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_7kmh_2_走路.csv',
%     '..\RawData\new1214_健身房测试\Imu_7kmh_2_跑步.csv',
%     '..\RawData\new1214_健身房测试\Imu_8kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_10kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Imu_9kmh_2.csv'
% };
% 
% plantarFilePaths = {
%     '..\RawData\new1214_健身房测试\Plantar_0kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_1kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_2kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_3kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_6kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_7kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_8kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_9kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_10kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_11kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_12kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_1kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_2kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_3kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4.2kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4.4kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4.6kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_4.8kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5.2kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5.4kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5.6kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_5.8kmh.csv',
%     '..\RawData\new1214_健身房测试\Plantar_6kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_7kmh_2_走路.csv',
%     '..\RawData\new1214_健身房测试\Plantar_7kmh_2_跑步.csv',
%     '..\RawData\new1214_健身房测试\Plantar_8kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_10kmh_2.csv',
%     '..\RawData\new1214_健身房测试\Plantar_9kmh_2.csv'
% };
% 
% % 创建存储 iHandler 和 pHandler 的单元格数组
% iHandlers = cell(size(imuFilePaths));
% pHandlers = cell(size(plantarFilePaths));
% 
% 
% % 导入数据
% for i = 1:length(imuFilePaths)
%     % 创建 ImuHandler 对象
%     iHandlers{i} = ImuHandler(imuFilePaths{i});
%     
%     % 从文件名中提取速度
%     [~, filename, ~] = fileparts(plantarFilePaths{i});
%     speedMatch = regexp(filename, '(\d*\.?\d*)kmh(_\d)?', 'tokens');
%     speedStr = speedMatch{1}{1};
%     speed = str2double(speedStr);
%     
%     % 创建 PlantarHandler 对象
%     pHandlers{i} = PlantarHandler(plantarFilePaths{i}, iHandlers{i},"WalkSpeed",speed);
% end
% 
% 
% % 创建一个空数组以容纳 PlantarHandler 对象
% plantarHandlersArray = PlantarHandler.empty(0, length(pHandlers));
% 
% % 将 PlantarHandler 对象添加到数组中
% for i = 1:length(pHandlers)
%     plantarHandlersArray(i) = pHandlers{i};
% end





profile off % 关闭性能分析

%% 进行运算
profile clear % 清除之前的分析信息
profile on % 启用性能分析
% 
stateCalr1 = StateCalculator(iHandler,pHandler);
% stateCalr2 = StateCalculator(iHandler,pHandler);
% stateCalr3 = StateCalculator(iHandler,pHandler);
% 
% stateCalr1.solveState("Both");
% stateCalr2.solveState("Imu");
% stateCalr3.solveState("Plantar");



% stateCalr1 = StateCalculator(iHandlers{1},pHandlers{1});
% stateCalr1.solveState("Imu");
% Plotter.plot_Tracks_2D_with_Plantar(stateCalr1);

profile off % 关闭性能分析

             
%% 绘图
% Plotter.plot_Tracks_2D([stateCalr1,stateCalr2,stateCalr3]);
% Plotter.plot_Tracks_3D([stateCalr1,stateCalr2,stateCalr3]);

% Plotter.plot_Gait_AccENU_AngularVel(stateCalr1);
% Plotter.plot_AngularVel_Sum(stateCalr1);
% Plotter.plot_V_COPVel(stateCalr1);

% Plotter.plot_PeakPressure_Speed(plantarHandlersArray);
Plotter.plot_AccRaw_Sum(stateCalr1);

