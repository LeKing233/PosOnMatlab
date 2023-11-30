% 清除工作区
clear;
close all;
clc;

%% 读取数据

profile clear % 清除之前的分析信息
profile on % 启用性能分析

ImuFilePath = '../RawData/new1130/imu_22-10-yiquan_manzou2.csv';
PlantarFilePath = '../RawData/new1130/plantar_22-10-yiquan_manzou2.csv';


iHandler = ImuHandler(ImuFilePath);
pHandler = PlantarHandler(PlantarFilePath,iHandler);


%% 进行运算
profile clear % 清除之前的分析信息
profile on % 启用性能分析

stateCalr1 = StateCalculator(iHandler,pHandler);
stateCalr2 = StateCalculator(iHandler,pHandler);
stateCalr3 = StateCalculator(iHandler,pHandler);

stateCalr1.solveState("Both");
stateCalr2.solveState("Imu");
stateCalr3.solveState("Plantar");

profile off % 关闭性能分析

             
%% 绘图
close all;
Plotter.plot_Tracks_2D([stateCalr1,stateCalr2,stateCalr3]);
Plotter.plot_Tracks_3D([stateCalr1,stateCalr2,stateCalr3]);
% Plotter.plot_Gait(stateCalr1);
Plotter.plot_Gait_AccRaw(stateCalr1);
Plotter.plot_Gait_AccENU(stateCalr1);




















% pHandler.drawHeatMapInSection(1,size(pHandler.mRawData.valueMat,1),2);
% pHandler.drawHeatMap(2);
% pHandler.plot_COPVelYSeq();
% 
% pHandler.plot_COPSeqInSection("Y",1,2000);

% pHandler.plot_SumSeq_Original_Insert();

% stateCalculator.plot_IMUGait_AND_Plantar();



% stateCalculator.plot_IMUGait_PlantarGait_Both();
% stateCalculator.plot_Track_3D();



% pHandler.drawHeatMapOfAll(10);
% pHandler.drawHeatMapOfAllSaveToMp4(2);



