% 清除工作区
clear all;
close all;
clc;

%% 读取数据
data = readtable('./RawData/test_trulyStatic.csv');

%% 进行运算
result = calculateState( ...
               [data.AngularVelX';data.AngularVelY';data.AngularVelZ'], ...
               [data.AccX';data.AccY';data.AccZ'] ...
               );

%% 绘图
%----------------------绘图控制--------------------- 
Track_Figure        = true;  
Velocity_Figure     = true;
Phi_Figure          = true;
Gait_Figure         = true;
KF_X_Figure         = true;


%轨迹图
if Track_Figure
    figure(1)
    plot(result.P(1,:),result.P(2,:),"LineWidth",2);
    xlabel('X方向'); % x轴注解
    ylabel('Y方向'); % y轴注解
    title('轨迹图'); % 图形标题
    legend('二维轨迹'); % 图形注解
    grid on; % 显示格线
end

%速度图
if Velocity_Figure
    figure(2)
    plot(data.Timestamp,result.V(1,:),"LineWidth",2);hold on
    plot(data.Timestamp,result.V(2,:),"LineWidth",2);hold on
    plot(data.Timestamp,result.V(3,:),"LineWidth",2);
    xlabel('时间戳'); % x轴注解
    ylabel('速度'); % y轴注解
    title('三轴速度曲线'); % 图形标题
    legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
    grid on; % 显示格线
end

%姿态角图
if Phi_Figure
    figure(3)
    plot(data.Timestamp,result.Phi(1,:));hold on
    plot(data.Timestamp,result.Phi(2,:));hold on
    plot(data.Timestamp,result.Phi(3,:));
    xlabel('时间戳'); % x轴注解
    ylabel('欧拉角'); % y轴注解
    title('欧拉角曲线'); % 图形标题
    legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线
end

%步态检测结果图
if Gait_Figure
    figure(4)
    plot(data.Timestamp,result.GaitPhase);
    xlabel('时间戳'); % x轴注解
    ylabel('步态检测结果'); % y轴注解
    title('步态检测结果曲线'); % 图形标题
    legend('0——支撑相，1——摆动相，-1——无结果'); % 图形注解
    grid on; % 显示格线
end

%零速修正过程结果图
if KF_X_Figure
    figure(5)
    plot(data.Timestamp,result.KF_X(1,:));hold on;
    plot(data.Timestamp,result.KF_X(2,:));hold on;
    plot(data.Timestamp,result.KF_X(3,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_phi'); % y轴注解
    title('欧拉角误差曲线'); % 图形标题
    legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线

    figure(6)
    plot(data.Timestamp,result.KF_X(4,:));hold on;
    plot(data.Timestamp,result.KF_X(5,:));hold on;
    plot(data.Timestamp,result.KF_X(6,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_v'); % y轴注解
    title('速度误差曲线'); % 图形标题
    legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
    grid on; % 显示格线

    figure(7)
    plot(data.Timestamp,result.KF_X(7,:));hold on;
    plot(data.Timestamp,result.KF_X(8,:));hold on;
    plot(data.Timestamp,result.KF_X(9,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('delta_p'); % y轴注解
    title('位置误差曲线'); % 图形标题
%     legend('横滚角Roll','俯仰角Pitch','偏航角Yaw'); % 图形注解
    grid on; % 显示格线
    
end





