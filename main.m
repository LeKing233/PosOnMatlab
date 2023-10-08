% 清除工作区
clear;
close all;
clc;

%% 读取数据
data = readtable('./RawData/12_12_3quan_60.csv');

% data = readtable('./RawData/test_trulyStatic.csv');
% data = readtable('./RawData/test_doudong.csv');
% data = readtable('./RawData/test_wave.csv');
% data = readtable('./RawData/test_3Swing_4Stance.csv');
% data = readtable('./RawData/test_10_swing.csv');
% data = readtable('./RawData/DataSet919/12_12_1quan_60.csv');
% data = readtable('./RawData/DataSet919/upStairs_2-7Floors.csv');
% data = readtable('./RawData/DataSet919/12_12_1quan_60_slow.csv');
% data = readtable('./RawData/DataSet919/100_1_zhixian.csv');
% data = readtable('./RawData/DataSet919/30_1_zhixian.csv');
% data = readtable('./RawData/DataSet919/12_12_3quan_60_run.csv');
% data = readtable('./RawData/DataSet919/30_4_zhixian.csv');
% data = readtable('./RawData/DataSet919/yuan.csv');




%% 进行运算
result = calculateState( ...
                [data.AngleRoll';data.AnglePitch';data.AngleYaw'],...
               [data.AngularVelX';data.AngularVelY';data.AngularVelZ'], ...
               [data.AccX';data.AccY';data.AccZ'] ...
               );
               %                [data.AngleRoll';data.AnglePitch';data.AngleYaw'],...
                %                [150;-25.3239;170.8245],...
%% 绘图
%----------------------绘图控制--------------------- 
Track_Figure        = true;  
Velocity_Figure     = false;
Phi_Figure          = false;
Gait_Figure         = false;
KF_X_Figure         = false;
RawData_Figure      = false;


%轨迹图
if Track_Figure
    figure(1)
    set(gcf, 'Position', get(0, 'Screensize'));
%     plot(result.P(1,:),result.P(2,:));
%     xlabel('X方向'); % x轴注解
%     ylabel('Y方向'); % y轴注解
%     title('轨迹图'); % 图形标题
%     legend('二维轨迹'); % 图形注解
%     axis equal
%     grid on; % 显示格线
    subplot(1,2,1);
    plot3(result.P(1,:),result.P(2,:),result.P(3,:));
    xlabel('X方向'); % x轴注解
    ylabel('Y方向'); % y轴注解
    zlabel('Z方向'); % z轴注解
    title('轨迹图'); % 图形标题
    legend('三维轨迹'); % 图形注解
    axis equal
    grid on; % 显示格线

    subplot(1,2,2);
    plot(result.P(1,:),result.P(2,:));
    xlabel('X方向'); % x轴注解
    ylabel('Y方向'); % y轴注解
    title('轨迹图'); % 图形标题
    legend('二维轨迹'); % 图形注解
    axis equal
    grid on; % 显示格线



end

%速度图
if Velocity_Figure
    figure(2)
    plot(data.Timestamp,result.V(1,:));hold on
    plot(data.Timestamp,result.V(2,:));hold on
    plot(data.Timestamp,result.V(3,:));
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
    plot(data.Timestamp,result.Phi(3,:));hold on
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


if RawData_Figure
    figure(8)
    plot(data.Timestamp,data.AccX);hold on;
    plot(data.Timestamp,data.AccY);hold on;
    plot(data.Timestamp,data.AccZ);hold on;
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
    plot(data.Timestamp,data.AngularVelX);hold on;
    plot(data.Timestamp,data.AngularVelY);hold on;
    plot(data.Timestamp,data.AngularVelZ);
    xlabel('时间戳'); % x轴注解
    ylabel('AngularVel'); % y轴注解
    title('角速度曲线'); % 图形标题
    legend('X','Y','Z'); % 图形注解
    grid on; % 显示格线


    figure(10)
    plot(data.Timestamp,result.AccENU(1,:));hold on;
    plot(data.Timestamp,result.AccENU(2,:));hold on;
    plot(data.Timestamp,result.AccENU(3,:));hold on;
    xlabel('时间戳'); % x轴注解
    ylabel('加速度'); % y轴注解
    title('ENU坐标系下的加速度曲线,Z轴减去重力加速度'); % 图形标题
    legend('X','Y','Z'); % 图形注解
    grid on; % 显示格线
    
end




res = result.P(:,end)'
length  = sqrt(res(1)^2+res(2)^2)
