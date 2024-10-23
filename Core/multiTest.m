%该脚本将多个实例一起计算


%%

% 清除工作区
clear;
close all;
clc;


rootPath = '../RawData/1015操场测试/合并/';

%单个实例
% stateCalrArray = StateCalculator(ImuHandler([rootPath 'Imu_方形h5.csv']),PlantarHandler([rootPath 'Plantar_方形h5.csv']));
% stateCalrArray.solveState("1");
% 
% Plotter.plot_Tracks_2D(stateCalrArray);
% Plotter.plot_Tracks_3D(stateCalrArray);



%多个实例
stateCalrArray = [
% StateCalculator(ImuHandler([rootPath 'Imu_直线h31.csv']),PlantarHandler([rootPath 'Plantar_直线h31.csv'])),    
% StateCalculator(ImuHandler([rootPath 'Imu_直线h32.csv']),PlantarHandler([rootPath 'Plantar_直线h32.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线h34.csv']),PlantarHandler([rootPath 'Plantar_直线h34.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线h36.csv']),PlantarHandler([rootPath 'Plantar_直线h36.csv'])), 
% 
% StateCalculator(ImuHandler([rootPath 'Imu_直线a31.csv']),PlantarHandler([rootPath 'Plantar_直线a31.csv'])),    
% StateCalculator(ImuHandler([rootPath 'Imu_直线a32.csv']),PlantarHandler([rootPath 'Plantar_直线a32.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线a34.csv']),PlantarHandler([rootPath 'Plantar_直线a34.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线a36.csv']),PlantarHandler([rootPath 'Plantar_直线a36.csv'])),

% StateCalculator(ImuHandler([rootPath 'Imu_直线a42.csv']),PlantarHandler([rootPath 'Plantar_直线a42.csv'])),    
% StateCalculator(ImuHandler([rootPath 'Imu_直线a43.csv']),PlantarHandler([rootPath 'Plantar_直线a43.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线a45.csv']),PlantarHandler([rootPath 'Plantar_直线a45.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线a46.csv']),PlantarHandler([rootPath 'Plantar_直线a46.csv'])), 
% 
% StateCalculator(ImuHandler([rootPath 'Imu_直线h42.csv']),PlantarHandler([rootPath 'Plantar_直线h42.csv'])),    
% StateCalculator(ImuHandler([rootPath 'Imu_直线h45.csv']),PlantarHandler([rootPath 'Plantar_直线h45.csv'])), 
% StateCalculator(ImuHandler([rootPath 'Imu_直线h46.csv']),PlantarHandler([rootPath 'Plantar_直线h46.csv'])), 
    
%  StateCalculator(ImuHandler([rootPath 'Imu_操场1.csv']),PlantarHandler([rootPath 'Plantar_操场1.csv'])), 
%  StateCalculator(ImuHandler([rootPath 'Imu_操场2.csv']),PlantarHandler([rootPath 'Plantar_操场2.csv'])), 
%  StateCalculator(ImuHandler([rootPath 'Imu_操场3.csv']),PlantarHandler([rootPath 'Plantar_操场3.csv'])), 
%  StateCalculator(ImuHandler([rootPath 'Imu_操场4a.csv']),PlantarHandler([rootPath 'Plantar_操场4a.csv'])),  
%  StateCalculator(ImuHandler([rootPath 'Imu_操场4h.csv']),PlantarHandler([rootPath 'Plantar_操场4h.csv'])), 

%方形————所有数据集
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

%方形————修改参数
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 100,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 200,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 300,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 400,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 800,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 1000,'ZUP_noiseG', 300), 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 120,'ZUP_noiseG', 300), 
% 
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 50),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 80),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 100),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 300),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1000),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1200),
% StateCalculator(ImuHandler([rootPath 'Imu_方形a1.csv']),PlantarHandler([rootPath 'Plantar_方形a1.csv']),'ZUP_noiseW', 0.02,'ZUP_noiseF', 600,'ZUP_noiseG', 1500),


];


for i = 1:length(stateCalrArray)
   str = "data " + num2str(i); % 连接字符串
   stateCalrArray(i).solveState(str);
end





Plotter.plot_Tracks_2D(stateCalrArray);




% Plotter.plot_Tracks_2D(stateCalrArray(2));



%% 测试图片旋转模块
P = stateCalrArray(3).mStateSeq.P';

P_adjust = TrackAdjuster.rotate2D(P(1,:),P(2,:),40);

plot(P(1,:),P(2,:),'LineWidth',3);hold on;
plot(P_adjust(1,:),P_adjust(2,:),'LineWidth',2);hold on;
xlabel('X方向/米','FontSize', 16); % x轴注解
ylabel('Y方向/米','FontSize', 16); % y轴注解
title('二维轨迹图'); % 图形标题
grid on; % 显示格线
axis equal;




TrackAdjuster.plotTrajectory([0,0,-16,-16,0;0,19,19,0,0],'g--');

























