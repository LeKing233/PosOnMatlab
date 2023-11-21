% 清除工作区
clear;
close all;
clc;

%% 读取数据
% dataImu = readtable('./RawData/zhixian_122steps_60.csv');
% dataImu = readtable('./RawData/new1019/square_636steps_60.csv');
% dataImu = readtable('./RawData/new1019/zhixian_sun_133steps_60.csv');
% dataImu = readtable('./RawData/test_doudong.csv');
% dataImu = readtable('./RawData/test_wave.csv');
% dataImu = readtable('./RawData/test_height.csv');
% dataImu = readtable('./RawData/test_3Swing_4Stance.csv');
% dataImu = readtable('./RawData/test_10_swing.csv');
% dataImu = readtable('./RawData/DataSet919/12_12_1quan_60.csv');
% dataImu = readtable('./RawData/DataSet919/12_12_3quan_60.csv');
% dataImu = readtable('./RawData/DataSet919/upStairs_2-7Floors.csv');
% dataImu = readtable('./RawData/DataSet919/upStairs_1-2Floors.csv');
% dataImu = readtable('./RawData/DataSet919/12_12_1quan_60_slow.csv');
% dataImu = readtable('./RawData/DataSet919/100_1_zhixian.csv');
% dataImu = readtable('./RawData/DataSet919/12_12_3quan_60_run.csv');
% dataImu = readtable('./RawData/DataSet919/30_4_zhixian.csv');
% dataImu = readtable('./RawData/DataSet919/yuan.csv');
% dataImu = readtable('./RawData/new1025/zhixian_single_line.csv');
% dataImu = readtable('./RawData/new1025/whole_square.csv');
% dataImu = readtable('./RawData/new1101/chen_caochang.csv');
% dataImu = readtable('./RawData/new1101/sun_caochang.csv');
% dataImu = readtable('./RawData/new1101/louti_zhixian.csv');
% dataImu = readtable('./RawData/new1101/louti_bihuan.csv');



% dataImu = readtable('./RawData/new1113/test_imu.csv');

% dataPlantar = extractPlantarData('./RawData/new1113/plantar_whole2.csv'); 
% dataPlantar = extractPlantarData('./RawData/new1113/plantar_baidong.csv'); 
% dataPlantar = extractPlantarData('./RawData/new1113/plantar_70.csv'); 
% dataPlantar = extractPlantarData('./RawData/new1113/plantar_normal.csv'); 
% dataPlantar = extractPlantarData('./RawData/new1113/plantar_stance.csv'); 
% ImuFilePath = '../RawData/DataSet919/100_1_zhixian.csv';
% PlantarFilePath = '../RawData/new1113/plantar_walk_100.csv';

ImuFilePath = '../RawData/new1118/imu_walk_100.csv';
PlantarFilePath = '../RawData/new1118/plantar_walk_100.csv';





%% 构建
iHandler = ImuHandler(ImuFilePath);
pHandler = PlantarHandler(PlantarFilePath);
stateCalculator = StateCalculator(iHandler,pHandler);
%进行运算
stateCalculator.solveState();


             
%% 绘图
% pHandler.drawHeatMapInSection(1,size(pHandler.mRawData.valueMat,1),2);
% pHandler.drawHeatMap(2);
% pHandler.plotSumSeq();
% pHandler.plotCOPNorm();
pHandler.plotCOPSeq();
pHandler.plotPressureCenterSeq();
stateCalculator.plotHorizontalVelNorm();
stateCalculator.plotHorizontalVelXY();





% pHandler.drawHeatMapOfAll(1);
% pHandler.drawHeatMapOfAllSaveToMp4(1);
















%----------------------绘图控制--------------------- 
Track_Figure        = false;  
Velocity_Figure     = false;
Phi_Figure          = false;
Gait_Figure         = false;
KF_X_Figure         = false;
RawData_Figure      = false;
% 
% P = stateCalculator.StateSeq.P.toMat();
% GaitPhase = stateCalculator.StateSeq.GaitPhase.toMat();

% 
% trackLength = norm(P(:,end)-P(:,1))
% deltaZ = norm(max(P(3,:))-min(P(3,:)));



%轨迹图
if Track_Figure
    figure(1)    
%     [0 0.4470 0.7410]
    plot(P(1,:),P(2,:),'LineWidth',2,'color',[0 0.4470 0.7410]);
    xlabel('X方向/米','FontSize', 16); % x轴注解
    ylabel('Y方向/米','FontSize', 16); % y轴注解
    title('二维轨迹图','FontSize', 16); % 图形标题
    axis equal
    grid on; % 显示格线
    set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
    % 设置底色为透明    
%   colordef none; %2D/3D图背景透明色（能透过看到窗口背景的颜色）
%     set(gca,'color','none'); % gca = get current axis
%     set(gcf,'color','none'); % gcf = get current figure
    
        
    
%     plot3(P(1,:),P(2,:),P(3,:),'LineWidth',2);
%     xlabel('X方向/米','FontSize', 12); % x轴注解
%     ylabel('Y方向/米','FontSize', 12); % y轴注解
%     zlabel('Z方向/米','FontSize', 12); % z轴注解
%     title('三维轨迹图','FontSize', 16); % 图形标题
%     set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
%     axis equal
%     grid on; % 显示格线



%     set(gcf, 'Position', get(0, 'Screensize'));
%     subplot(1,2,1);
%     P = stateCalculator.StateSeq.P.toMat();
%     plot3(P(1,:),P(2,:),P(3,:));
%     xlabel('X方向'); % x轴注解
%     ylabel('Y方向'); % y轴注解
%     zlabel('Z方向'); % z轴注解
%     title('三维轨迹图'); % 图形标题
%     axis equal
%     grid on; % 显示格线
% 
%     subplot(1,2,2);
%     plot(P(1,:),P(2,:));
%     xlabel('X方向'); % x轴注解
%     ylabel('Y方向'); % y轴注解
%     title('二维轨迹图'); % 图形标题
%     axis equal
%     grid on; % 显示格线
end

%速度图
if Velocity_Figure
    figure(2)
    plot(dataImu.Timestamp,result.V(1,:));hold on
    plot(dataImu.Timestamp,result.V(2,:));hold on
    plot(dataImu.Timestamp,result.V(3,:));
    xlabel('时间戳'); % x轴注解
    ylabel('速度'); % y轴注解
    title('三轴速度曲线'); % 图形标题
    legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
    grid on; % 显示格线
end

%姿态角图
if Phi_Figure
    figure(3)
    plot(dataImu.Timestamp,result.Phi(1,:));hold on
    plot(dataImu.Timestamp,result.Phi(2,:));hold on
    plot(dataImu.Timestamp,result.Phi(3,:));hold on
%     plot(data.Timestamp,data.AngleRoll);hold on
%     plot(data.Timestamp,data.AnglePitch);hold on
%     plot(data.Timestamp,data.AngleYaw);hold on
    xlabel('时间戳'); % x轴注解
    ylabel('欧拉角'); % y轴注解
    title('欧拉角曲线'); % 图形标题
    legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线
    
end

%步态检测结果图
if Gait_Figure
    figure(4)
    plot(dataImu.Timestamp,GaitPhase);
    xlabel('时间戳'); % x轴注解
    ylabel('步态检测结果'); % y轴注解
    title('步态检测结果曲线'); % 图形标题
    legend('0——支撑相，1——摆动相，-1——无结果'); % 图形注解
    grid on; % 显示格线
end

%零速修正过程结果图
if KF_X_Figure
    figure(5)
    plot(dataImu.Timestamp,result.KF_X(1,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(2,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(3,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_phi'); % y轴注解
    title('欧拉角误差曲线'); % 图形标题
    legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线

    figure(6)
    plot(dataImu.Timestamp,result.KF_X(4,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(5,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(6,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_v'); % y轴注解
    title('速度误差曲线'); % 图形标题
    legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
    grid on; % 显示格线

    figure(7)
    plot(dataImu.Timestamp,result.KF_X(7,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(8,:));hold on;
    plot(dataImu.Timestamp,result.KF_X(9,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_p'); % y轴注解
    title('位置误差曲线'); % 图形标题
%     legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线
    
end


if RawData_Figure
    figure(8)
    plot(dataImu.Timestamp,dataImu.AccX);hold on;
    plot(dataImu.Timestamp,dataImu.AccY);hold on;
    plot(dataImu.Timestamp,dataImu.AccZ);hold on;
%     plot(data.Timestamp,result.A(1,:));hold on;
%     plot(data.Timestamp,result.A(2,:));hold on;
%     plot(data.Timestamp,result.A(3,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('acc'); % y轴注解
    title('加速度曲线'); % 图形标题
%     legend('AccX','AccY','AccZ','AccX-G','AccY-G','AccZ-G'); % 图形注解
    legend('AccX','AccY','AccZ'); % 图形注解
    grid on; % 显示格线




    figure(9)
    plot(dataImu.Timestamp,dataImu.AngularVelX);hold on;
    plot(dataImu.Timestamp,dataImu.AngularVelY);hold on;
    plot(dataImu.Timestamp,dataImu.AngularVelZ);
    xlabel('时间戳'); % x轴注解
    ylabel('AngularVel'); % y轴注解
    title('角速度曲线'); % 图形标题
    legend('X','Y','Z'); % 图形注解
    grid on; % 显示格线


%     figure(10)
%     plot(data.Timestamp,result.AccENU(1,:));hold on;
%     plot(data.Timestamp,result.AccENU(2,:));hold on;
%     plot(data.Timestamp,result.AccENU(3,:));hold on;
%     xlabel('时间戳'); % x轴注解
%     ylabel('加速度'); % y轴注解
%     title('ENU坐标系下的加速度曲线,Z轴减去重力加速度'); % 图形标题
%     legend('X','Y','Z'); % 图形注解
%     grid on; % 显示格线


 
    
end

% 
% Phi = stateCalculator.StateSeq.Phi.toMat();
% figure(9)
%     plot(dataImu.Timestamp,Phi(1,:));hold on;
%     plot(dataImu.Timestamp,dataImu.AngleRoll)
%     xlabel('时间戳'); % x轴注解
%     ylabel('欧拉角/°'); % y轴注解
%     title('欧拉角曲线'); % 图形标题
%     legend('自解算','维特智能'); % 图形注解
%     grid on; % 显示格线

