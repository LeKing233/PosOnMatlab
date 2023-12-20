StImuFilePath = '..\RawData\new1220测试两个Imu\Imu_St_twoimu6.csv';
WitImuFilePath = '..\RawData\new1220测试两个Imu\Imu_Wit_twoimu6.csv';

PlantarFilePath = '..\RawData\new1220测试两个Imu\Imu_Wit_twoimu6.csv';

StData = readtable(StImuFilePath);
WitData = readtable(WitImuFilePath);



gcf = figure("Name","TestTwoImus");%创建图窗




% plot(StData.Timestamp,StData.AccX);hold on;
% plot(StData.Timestamp,StData.AccY);hold on;
% plot(StData.Timestamp,StData.AccZ);hold on;
% plot(WitData.Timestamp,WitData.AccX);hold on;
% plot(WitData.Timestamp,WitData.AccY);hold on;
% plot(WitData.Timestamp,WitData.AccZ);hold on;
% legend('StAcc_X','StAcc_Y','StAcc_Z','WitAcc_X','WitAcc_Y','WitAcc_Z');
% 
% 
% plot(StData.Timestamp,StData.AccX);hold on;
% plot(WitData.Timestamp,WitData.AccX);hold on;
% legend('StAcc_X','WitAcc_X');



plot(StData.Timestamp,StData.AngularVelX);hold on;
plot(WitData.Timestamp,WitData.AngularVelX);hold on;
legend('StAngularVel_X','WitAngularVel_X');



xlabel('时间戳/ms'); % x轴注解
ylabel('结果'); % y轴注解
title('两Imu延时对比图'); % 图形标题
grid on; % 显示格线



























