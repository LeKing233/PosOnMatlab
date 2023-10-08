% 清除工作区
clear;
close all;
clc;

%% 读取数据
% data = readtable('./RawData/12_12_3quan_60.csv');
% data = readtable('./RawData/test_trulyStatic.csv');
data = readtable('./RawData/test_wave.csv');
% data = readtable('./RawData/DataSet919/12_12_1quan_60_slow.csv');
% data = readtable('./RawData/DataSet919/100_1_zhixian.csv');

%% 进行运算
res = compareImuAngleWithZhao([data.AngleRoll';data.AnglePitch';data.AngleYaw']);



    figure(1)
    plot(data.Timestamp,res.Phi(1,:));hold on
    plot(data.Timestamp,res.Phi(2,:));hold on
    plot(data.Timestamp,res.Phi(3,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(1,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(2,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(3,:));hold on
    xlabel('时间戳'); % x轴注解
    ylabel('Angle(°)'); % y轴注解
    title('姿态矩阵测试对比图'); % 图形标题
    legend('Imu角度X轴','Imu角度Y轴','Imu角度Z轴','Zhao角度X轴','Zhao角度Y轴','Zhao角度Z轴'); % 图形注解
    grid on; % 显示格线


    figure(2)
    plot(data.Timestamp,res.Phi(1,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(1,:));hold on
    plot(data.Timestamp,res.T33);hold on
    plot(data.Timestamp,res.T32);hold on
    xlabel('时间戳'); % x轴注解
    ylabel('Angle(°)'); % y轴注解
    title('姿态矩阵测试对比图'); % 图形标题
    legend('Imu角度X轴','Zhao角度X轴','T33','T32'); % 图形注解
    grid on; % 显示格线

    figure(3)
    plot(data.Timestamp,res.Phi(3,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(3,:));hold on
    plot(data.Timestamp,res.T11);hold on
    plot(data.Timestamp,res.T21);hold on

    xlabel('时间戳'); % x轴注解
    ylabel('Angle(°)'); % y轴注解
    title('姿态矩阵测试对比图'); % 图形标题
    legend('Imu角度Z轴','Zhao角度Z轴','T11','T21'); % 图形注解
    grid on; % 显示格线

    figure(4)

    plot(data.Timestamp,res.Phi(2,:));hold on
    plot(data.Timestamp,res.PhiOnZhao(2,:));hold on
    xlabel('时间戳'); % x轴注解
    ylabel('Angle(°)'); % y轴注解
    title('姿态矩阵测试对比图'); % 图形标题
    legend('Imu角度Y轴','Zhao角度Y轴'); % 图形注解
    grid on; % 显示格线

    

%      figure(5)
% 
%     plot(data.Timestamp,res.Phi(2,:));hold on
%     plot(data.Timestamp,res.PhiOnZhao(2,:));hold on
%     plot(data.Timestamp,res.PhiOnZhao(2,:)-res.Phi(2,:));hold on
%     xlabel('时间戳'); % x轴注解
%     ylabel('Angle(°)'); % y轴注解
%     title('姿态矩阵测试对比图'); % 图形标题
%     legend('Imu角度Y轴','Zhao角度Y轴','差值'); % 图形注解
%     grid on; % 显示格线


function result = compareImuAngleWithZhao(Phi)
    PhiOnZhao = zeros(3,1);
    %结果赋初值
    result.Phi = Phi;
    result.PhiOnZhao = Phi(:,1);
    result.T11 = 0;
    result.T21 = 0;
    result.T33 = 0;
    result.T32 = 0;

    for i = 2:length(Phi)
        [res,T11,T21,T33,T32] = getPhiFromCbn(getCbnFromPhi(Phi(:,i)));
        result.PhiOnZhao = [result.PhiOnZhao,res];
        result.T11 = [result.T11,T11];
        result.T21 = [result.T21,T21];
        result.T33 = [result.T33,T33];
        result.T32 = [result.T32,T32];
    end
end

function  Cbn = getCbnFromPhi(phi)
    phi =  phi *pi/180;
    gamma = phi(1);%横滚角roll
    theta = phi(2);%俯仰角pitch
    psi = phi(3);%翻滚角yaw
    
    Cbn = [cos(theta)*cos(psi),-cos(gamma)*sin(psi)+sin(gamma)*sin(theta)*cos(psi),sin(gamma)*sin(psi)+cos(gamma)*sin(theta)*cos(psi);
           cos(theta)*sin(psi),cos(gamma)*cos(psi)+sin(gamma)*sin(theta)*sin(psi),-sin(gamma)*cos(psi)+cos(gamma)*sin(theta)*sin(psi);
           -sin(theta),sin(gamma)*cos(theta),cos(gamma)*cos(theta)];
end

function [Phi,T11,T21,T33,T32] = getPhiFromCbn(Cbn)
    threshold = 0.000001;
    %横滚角Roll
    roll = atan(Cbn(3,2)/Cbn(3,3));
    T33  = Cbn(3,3);
    T32 = Cbn(3,2);

    if(T33>0)
        roll = roll;
    else 
        if(T32>0)
            roll =  pi + roll;
        else 
            roll =  -pi + roll;
        end 
    end
    
  

    Phi(1) = roll;
    if(T33>0)
        T33 = 10;
    else
        T33 = -10;
    end

    if(T32>0)
        T32 = 20;
    else
        T32 = -20;
    end



    %俯仰角Pitch  
    Phi(2) = asin(-Cbn(3,1));
    %航向角yaw
    yaw = atan(Cbn(2,1)/Cbn(1,1));
    T11 = Cbn(1,1);
    T21 = Cbn(2,1); 


    
    if(T11<0&&T21<0)
        yaw = yaw - pi;
    end
    if(T11<0&&T21>0)
        yaw =  yaw + pi;
    end



    
    Phi(3) = yaw;
    
    Phi = Phi'*180/pi;%转置成列向量
    if(T11>0)
        T11 = 10;
    else
        T11 = -10;
    end

    if(T21>0)
        T21 = 20;
    else
        T21 = -20;
    end
end


