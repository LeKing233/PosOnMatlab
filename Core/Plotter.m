classdef Plotter
    %PLOTTER 绘图器
    %   负责将计算结果显示，采用依赖数据类方式，可以同时绘制多个数据类
    %% 绘制tateCalculator

    properties(Constant)

    end

    methods
        function obj = Plotter()

        end
    end

    methods(Static)

        % @brief 将单个对象转换成数组，以此适应一个或多个数据源
        % @param 对象或数组
        % @retval 转换完的数组
        function objectArray = toArray(objectArray)
            % 检查输入是否为单元数组
            if ~isvector(objectArray) && ~isobject(objectArray)
                error('绘图器错误：输入参数必须是对象实例或对象数组。');
            end

            % 如果输入是单个对象实例，将其转换为单元数组
            if isobject(objectArray) && ~isvector(objectArray)
                objectArray = [objectArray];
            end
        end

        % @brief 将绘图区设置为只有Y轴缩放
        % @param gca 绘图区句柄
        function onlyYAxisZoom(gca)
            ax = gca;
            zoomInt = zoomInteraction('Dimensions','x'); % 仅在 x 轴方向上缩放
            panInt = panInteraction; % 添加拖放功能
            dataTipInt = dataTipInteraction; % 添加数据提示功能
            ax.Interactions = [zoomInt, panInt, dataTipInt]; % 同时启用这些交互
        end

        % @brief 三维轨迹图
        % @param stateCalrArray实例数组
        function plot_Tracks_3D(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            gcf = figure("Name","StateCalculator");%创建图窗
            legendArray = [];
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                plot3(P(:,1),P(:,2),P(:,3),LineWidth=2);hold on;

            end
            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            zlabel('Z方向/米','FontSize', 16); % z轴注解
            title('三维轨迹图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            legend(legendArray);
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
            %             % 调整视角为侧视图
            %             view(0, 0);
        end

        % @brief 二维轨迹图
        % @param stateCalrArray实例数组
        function plot_Tracks_2D(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            gcf = figure("Name","StateCalculator");%创建图窗
            %提取数据
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                plot(P(:,1),P(:,2),LineWidth=2);hold on;

            end
            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            title('二维轨迹图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
            legend(legendArray);
        end


        % @brief 步态检测结果图
        % @param stateCalculator实例
        function plot_Gait(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase,LineWidth=1);hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('融合相位检测、IMU相位检测、Plantar相位检测结果汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合');
            ylim([-0.2 1.2])
            ax = gca;
            ax.Interactions = zoomInteraction('Dimensions','x');
        end

        % @brief 步态和加速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccRaw(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,LineWidth=1);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccX);
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccY);
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccZ);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('步态检测——原始三轴加速度汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合','AccX','AccY','AccZ');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            ax = gca;
            ax.Interactions = zoomInteraction('Dimensions','x');
        end

        % @brief 步态和加速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,LineWidth=1);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,1));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,2));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,3));
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('步态检测——东北天加速度汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合','AccENU_X','AccENU_Y','AccENU_Z');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 步态和加速度、压力和混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,LineWidth=2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,LineWidth=1);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,1));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,2));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,3));
            plot(timeSeq,stateCalculator.pHandler.mSumSeq/5);




            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('步态检测——东北天加速度汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合','AccENU_X','AccENU_Y','AccENU_Z','Sumseq');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end


        % @brief 加速度、压力和混合图
        % @param stateCalculator实例
        function plot_AccRaw_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccX);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccY);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccZ);hold on;
            plot(timeSeq,(stateCalculator.pHandler.mSumSeq-240)*1);hold on;

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('加速度——压力和汇总图'); % 图形标题
            grid on; % 显示格线
            legend('Acc_X','Acc_Y','Acc_Z','Sumseq');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 修正后速度、COP速度图
        % @param stateCalculator实例
        function plot_V_COPVel(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            V = stateCalculator.mStateSeq.V;
            COPVel = stateCalculator.pHandler.mCOPVelSeq/10;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,V(:,1));hold on;
            plot(timeSeq,V(:,2));hold on;            
            plot(timeSeq,COPVel(:,1));hold on;
            plot(timeSeq,COPVel(:,2));hold on; 

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('修正后速度、COP速度图'); % 图形标题
            grid on; % 显示格线
            legend('V_X','V_Y','COPVel_X','COPVel_Y');           
            Plotter.onlyYAxisZoom(gca);
        end

















    end


    %% 绘制PlantarHandler




    %% 绘制ImuHandler

end

