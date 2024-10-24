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
        % @retval None
        function onlyYAxisZoom(gca)
            ax = gca;
            zoomInt = zoomInteraction('Dimensions','x'); % 仅在 x 轴方向上缩放
            panInt = panInteraction; % 添加拖放功能
            dataTipInt = dataTipInteraction; % 添加数据提示功能
            ax.Interactions = [zoomInt, panInt, dataTipInt]; % 同时启用这些交互
        end

        % @brief 三维轨迹图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_Tracks_3D(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            gcf = figure("Name","StateCalculator");%创建图窗
            legendArray = [];
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                plot3(P(:,1),P(:,2),P(:,3),'LineWidth',2);hold on;

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
        % @retval None
        function plot_Tracks_2D(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            gcf = figure("Name","StateCalculator");%创建图窗

            %提取数据
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                legendArray = [legendArray,string(stateCalr.mStateSeq.Legend)]; % 确保转换为字符串
                plot(P(:,1),P(:,2),'LineWidth',2);hold on;
            end
            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            title('二维轨迹图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
            legend(legendArray);
        end




        % @brief 姿态角推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plotERREuler(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                ErrPhi = stateCalr.mStateSeq.ErrPhi;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制姿态角误差观测值曲线
                figure('Name',['姿态角误差观测值 No.', num2str(i)]);
                plot(SupTime,ErrPhi(:,1));hold on
                plot(SupTime,ErrPhi(:,2));hold on
                plot(SupTime,ErrPhi(:,3));hold on
                legend('俯仰角Pitch','横滚角Roll','偏航角Yaw'); % 图形注解
                xlabel('时间戳'); % x轴注解
                ylabel('欧拉角'); % y轴注解
                title('姿态角误差观测值'); % 图形标题
                Plotter.onlyYAxisZoom(gca);

            end
        end

        % @brief 姿态角推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plotEuler(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Phi = stateCalr.mStateSeq.Phi;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制俯仰角曲线
                figure('Name',['俯仰角 No.', num2str(i)]);
                disp(length(TimeStamp(Utils.ALIGNER_STAMP:end)));
                disp(length(Phi(:,1)));
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,1));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('欧拉角'); % y轴注解
                title('俯仰角Pitch'); % 图形标题
                Plotter.onlyYAxisZoom(gca);
                %绘制横滚角曲线
                figure('Name',['横滚角 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,2));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('欧拉角'); % y轴注解
                title('横滚角Roll'); % 图形标题
                Plotter.onlyYAxisZoom(gca);
                %绘制航向角曲线
                figure('Name',['航向角 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('欧拉角'); % y轴注解
                title('航向角Yaw'); % 图形标题
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 姿态角推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plotAllEuler(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Phi = stateCalr.mStateSeq.Phi;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制俯仰角曲线
                figure('Name',['姿态角 No.', num2str(i)]);
                disp(length(TimeStamp(Utils.ALIGNER_STAMP:end)));
                disp(length(Phi(:,1)));
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,1));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,2));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Phi(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('角度'); % y轴注解
                title('姿态角曲线'); % 图形标题
                legend('俯仰角Pitch','横滚角Roll','航向角Yaw'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 三轴加速度推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_Acc(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Acc = stateCalr.mStateSeq.Acc;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制三轴速度曲线
                figure('Name',['三轴加速度No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Acc(:,1));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Acc(:,2));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Acc(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('加速度'); % y轴注解
                title('三轴加速度曲线'); % 图形标题
                legend('X轴加速度','Y轴加速度','Z轴加速度'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end


        % @brief 三轴速度推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_Velocity(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Vel = stateCalr.mStateSeq.V;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制三轴速度曲线
                figure('Name',['三轴速度No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Vel(:,1));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Vel(:,2));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Vel(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('速度'); % y轴注解
                title('三轴速度曲线'); % 图形标题
                legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 重力加速度观测误差结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_ErrG(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                Err_G = stateCalr.mStateSeq.Err_G;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制三轴速度曲线
                figure('Name',['重力加速度观测误差No.', num2str(i)]);
                plot(SupTime,Err_G(:,1));hold on
                plot(SupTime,Err_G(:,2));hold on
                plot(SupTime,Err_G(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('重力加速度观测误差'); % y轴注解
                title('重力加速度观测误差曲线'); % 图形标题
                legend('X轴误差','Y轴误差','Z轴误差'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 地磁场观测误差结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_ErrM(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                Err_M = stateCalr.mStateSeq.Err_M;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制三轴速度曲线
                figure('Name',['地磁场观测误差No.', num2str(i)]);
                plot(SupTime,Err_M(:,1));hold on
                plot(SupTime,Err_M(:,2));hold on
                plot(SupTime,Err_M(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('地磁场观测误差'); % y轴注解
                title('地磁场观测误差曲线'); % 图形标题
                legend('X轴误差','Y轴误差','Z轴误差'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 重力加速度检测值结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_Rkg(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                Rk_G = stateCalr.mStateSeq.Rk_G;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制三轴速度曲线
                figure('Name',['重力加速度检测值No.', num2str(i)]);
                plot(SupTime,Rk_G);hold on
                xlabel('时间戳'); % x轴注解
                ylabel('重力加速度检测值'); % y轴注解
                title('重力加速度检测值'); % 图形标题
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 地磁场检测值结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_Rkm(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                Rk_M = stateCalr.mStateSeq.Rk_M;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制三轴速度曲线
                figure('Name',['地磁场检测值No.', num2str(i)]);
                plot(SupTime,Rk_M);hold on
                xlabel('时间戳'); % x轴注解
                ylabel('地磁场检测值'); % y轴注解
                title('地磁场检测值'); % 图形标题
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 零速度观测误差结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_ErrV(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                SupTime = stateCalr.mStateSeq.SupTime;
                Err_V = stateCalr.mStateSeq.Err_V;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制三轴速度曲线
                figure('Name',['零速度观测误差No.', num2str(i)]);
                plot(SupTime,Err_V(:,1));hold on
                plot(SupTime,Err_V(:,2));hold on
                plot(SupTime,Err_V(:,3));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('零速度观测误差'); % y轴注解
                title('零速度观测误差曲线'); % 图形标题
                legend('X轴误差','Y轴误差','Z轴误差'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 水平速度模长推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_HorizontalVelNorm(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Vel = stateCalr.mStateSeq.V;
                velNormVec = zeros(size(Vel,1),1);
                for j = 1:size(Vel,1)
                    velNormVec(j) = norm([Vel(j,1),Vel(j,2)]); % 求取x方向和y方向速度模长
                end
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制水平方向速度模长曲线
                figure('Name',['水平速度No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),velNormVec);
                xlabel('时间戳(ms)'); % x轴注解
                ylabel('速度模长(m/s)'); % y轴注解
                title('水平速度模长曲线'); % 图形标题
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end


        % @brief 三轴速度模长推算结果图
        % @param stateCalrArray实例数组
        % @retval None
        function plot_TriaxialVelNorm(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Vel = stateCalr.mStateSeq.V;
                velNormVec = zeros(size(Vel,1),1);
                for j = 1:size(Vel,1)
                    velNormVec(j) = norm([Vel(j,1),Vel(j,2),Vel(j,3)]); % 求取三轴速度模长
                end
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                %绘制水平方向速度模长曲线
                figure('Name',['三轴速度No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),velNormVec);
                xlabel('时间戳(ms)'); % x轴注解
                ylabel('速度模长(m/s)'); % y轴注解
                title('三轴速度模长曲线'); % 图形标题
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 四元数推算结果图
        % @param stateCalculator实例
        % @retval None
        function plot_Quaternion(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                SupTime = stateCalr.mStateSeq.SupTime;
                Q = stateCalr.mStateSeq.Q;
                SupQ = stateCalr.mStateSeq.SupQ;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制四元数q0结果曲线
                figure('Name',['四元数q0 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,1));hold on
                plot(SupTime,SupQ(:,1));hold on
                legend('Kalman','AHRS'); % 图形注解
                xlabel('时间戳'); % x轴注解
                ylabel('四元数q0'); % y轴注解
                title('四元数q0'); % 图形标题
                Plotter.onlyYAxisZoom(gca);

                %绘制四元数q1结果曲线
                figure('Name',['四元数q1 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,2));hold on
                plot(SupTime,SupQ(:,2));hold on
                legend('Kalman','AHRS'); % 图形注解
                xlabel('时间戳'); % x轴注解
                ylabel('四元数q1'); % y轴注解
                title('四元数q1'); % 图形标题
                Plotter.onlyYAxisZoom(gca);

                %绘制四元数q2结果曲线
                figure('Name',['四元数q2 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,3));hold on
                plot(SupTime,SupQ(:,3));hold on
                legend('Kalman','AHRS'); % 图形注解
                xlabel('时间戳'); % x轴注解
                ylabel('四元数q2'); % y轴注解
                title('四元数q2'); % 图形标题
                Plotter.onlyYAxisZoom(gca);

                %绘制四元数q3结果曲线
                figure('Name',['四元数q0 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,4));hold on
                plot(SupTime,SupQ(:,4));hold on
                legend('Kalman','AHRS'); % 图形注解
                xlabel('时间戳'); % x轴注解
                ylabel('四元数q3'); % y轴注解
                title('四元数q3'); % 图形标题
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 四元数推算结果图
        % @param stateCalculator实例
        % @retval None
        function plot_AllQuaternion(stateCalrArray)
            %读取数据
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            %绘制不同示例结果图
            for i = 1:length(stateCalrArray)
                %提取数据
                stateCalr = stateCalrArray(i);
                TimeStamp = stateCalr.iHandler.mTSeq;
                Q = stateCalr.mStateSeq.Q;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];

                %绘制四元数q0结果曲线
                figure('Name',['四元数 No.', num2str(i)]);
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,1));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,2));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,3));hold on
                plot(TimeStamp(Utils.ALIGNER_STAMP:end),Q(:,4));hold on
                xlabel('时间戳'); % x轴注解
                ylabel('四元数'); % y轴注解
                title('姿态的四元数曲线'); % 图形标题
                legend('q0','q1','q2', 'q3'); % 图形注解
                grid on; % 显示格线
                Plotter.onlyYAxisZoom(gca);
            end
        end

        % @brief 姿态对准曲线
        % @param stateCalculator实例
        % @retval None
        function plot_Aligner(stateCalculator)
            %提取数据
            Q = stateCalculator.mStateSeq.Aligner_Q;
            Phi = stateCalculator.mStateSeq.Aligner_Phi;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot((1:Utils.ALIGNER_STAMP-1), Phi(:,1));hold on;
            plot((1:Utils.ALIGNER_STAMP-1), Phi(:,2));hold on;
            plot((1:Utils.ALIGNER_STAMP-1), Phi(:,3));hold on;
            xlabel('数据帧'); % x轴注解
            ylabel('结果'); % y轴注解
            title('姿态对准'); % 图形标题
            grid on; % 显示格线
            legend('roll','pitch','yaw');
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 加速度计原始数据
        % @param stateCalculator实例
        % @retval None
        function plot_Accelerometer(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq';
            Acc = stateCalculator.iHandler.mFSeq';
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,Acc(:,1));hold on;
            plot(timeSeq,Acc(:,2));hold on;
            plot(timeSeq,Acc(:,3));hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('加速度计数据'); % 图形标题
            grid on; % 显示格线
            legend('Acc_X','Acc_Y','Acc_Z');
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 陀螺仪原始数据
        % @param stateCalculator实例
        % @retval None
        function plot_Gyroscope(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq';
            Gyro = stateCalculator.iHandler.mWSeq';
            gcf = figure("Name","StateCalculator");%创建图窗
            average_x = mean(Gyro(1:500,1));
            average_y = mean(Gyro(1:500,2));
            average_z = mean(Gyro(1:500,3));
            disp("陀螺仪x轴平均");
            disp(average_x);
            disp("陀螺仪y轴平均");
            disp(average_y);
            disp("陀螺仪z-轴平均");
            disp(average_z);
            plot(timeSeq,Gyro(:,1));hold on;
            plot(timeSeq,Gyro(:,2));hold on;
            plot(timeSeq,Gyro(:,3));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,1));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,2));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,3));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,1));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,2));hold on;
            %             plot(timeSeq(1:500),Gyro(1:500,3));hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('陀螺仪数据'); % 图形标题
            grid on; % 显示格线
            legend('Gyro_X','Gyro_Y','Gyro_Z');
            %             Plotter.onlyYAxisZoom(gca);
        end

        % @brief 磁力计原始数据
        % @param stateCalculator实例
        % @retval None
        function plot_Magnetic(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq';
            Mag = stateCalculator.iHandler.mMSeq';
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,Mag(:,1));hold on;
            plot(timeSeq,Mag(:,2));hold on;
            plot(timeSeq,Mag(:,3));hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('磁力计数据'); % 图形标题
            grid on; % 显示格线
            legend('Mag_X','Mag_Y','Mag_Z');
            Plotter.onlyYAxisZoom(gca);
        end


        % @brief 步态检测结果图
        % @param stateCalculator实例
        % @retval None
        function plot_Gait(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq,'LineWidth',1);hold on;
            %             plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq,'LineWidth',2);hold on;
            %             plot(timeSeq,stateCalculator.mStateSeq.GaitPhase,'LineWidth',1);hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('融合相位检测、IMU相位检测、Plantar相位检测结果汇总图'); % 图形标题
            grid on; % 显示格线
            %             legend('IMU','Plantar','融合');
            ylim([-0.2 1.2])
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 步态和加速度混合图
        % @param stateCalculator实例
        % @retval None
        function plot_Gait_AccRaw(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,'LineWidth',1);hold on;
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
        % @retval None
        function plot_Gait_AccENU(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,'LineWidth',1);hold on;
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
        % @retval None
        function plot_Gait_AccENU_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,'LineWidth',1);hold on;
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
        % @retval None
        function plot_AccRaw_Sum(stateCalculator)
            gcf = figure("Name","StateCalculator");%创建图窗
            timeSeq = stateCalculator.iHandler.mTSeq';
            acc = stateCalculator.iHandler.mFSeq';
            acc_sum = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2) - 9.8;
            plot(timeSeq,acc_sum);hold on;
            plot(timeSeq,(stateCalculator.pHandler.mSumSeq)*0.1);hold on;
            plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.GaitPhase*50);hold on

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('加速度——压力和汇总图'); % 图形标题
            grid on; % 显示格线
            legend('Acc_Sum','Planter', '步态');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 角速度、压力和混合图
        % @param stateCalculator实例
        % @retval None
        function plot_GyroRaw_Sum(stateCalculator)
            gcf = figure("Name","StateCalculator");%创建图窗
            timeSeq = stateCalculator.iHandler.mTSeq';
            gyro = stateCalculator.iHandler.mWSeq';
            gyro_sum = sqrt(gyro(:,1).^2 + gyro(:,2).^2 + gyro(:,3).^2);
            plot(timeSeq,gyro_sum);hold on;
            plot(timeSeq,(stateCalculator.pHandler.mSumSeq)*1);hold on;
            plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.GaitPhase*500,'LineWidth',1);hold on;

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('角速度——压力和汇总图'); % 图形标题
            grid on; % 显示格线
            legend('Gyro_Sum','Planter', '步态');
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 绘制综合结果展示图
        % @param stateCalculator实例
        function displayResults(stateCalculator,varargin)
            %解析设置参数
            Config = inputParser;
            addParameter(Config, 'types',""); %选择绘制轴，默认都不绘制
            parse(Config, varargin{:});
            types  = Config.Results.types;

            timeSeq = stateCalculator.iHandler.mTSeq;
            timeSeqRaw = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗

            if contains(types,"Gait")
                disp('时间戳长度');
                disp(length(timeSeq(Utils.ALIGNER_STAMP:end)));
                %                 disp(length(timeSeq));
                disp('向量长度');
                disp(length(stateCalculator.mStateSeq.GaitPhase));
                plot(timeSeq, stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,'LineWidth',2,'DisplayName', 'Gait-IMU');hold on;
                plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,'LineWidth',2, 'DisplayName', 'Gait-Plantar');hold on;
                plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeqWithTransfer*50,'LineWidth',2,'LineStyle',":","Color","green", 'DisplayName', 'Gait-Plantar-Transfer');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.GaitPhase*50,'LineWidth',1, 'DisplayName', 'Gait-Both');hold on;
            end

            if contains(types,"Vel")
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.V(:,1)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'Vel-X');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.V(:,2)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'Vel-Y');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.V(:,3)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'Vel-Z');hold on;

            end

            if contains(types,"Pe")
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.P(:,1)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'P-X');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.P(:,2)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'P-Y');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.P(:,3)*20,'LineWidth',2,'LineStyle',"-",'DisplayName', 'P-Z');hold on;

                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Pe(1,:)*20,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'Pe-X');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Pe(2,:)*20,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'Pe-Y');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Pe(3,:)*20,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'Pe-Z');hold on;
            end

            if contains(types,"AccENU")
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.AccENU(:,1),'LineWidth',1,'DisplayName', 'AccENU-X');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.AccENU(:,2),'LineWidth',1,'DisplayName', 'AccENU-Y');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.AccENU(:,3),'LineWidth',1,'DisplayName', 'AccENU-Z');hold on;
            end

            if contains(types,"AccRaw")
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AccX,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'AccRaw-X');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AccY,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'AccRaw-Y');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AccZ,'LineWidth',2,'LineStyle',"-.",'DisplayName', 'AccRaw-Z');hold on;
            end

            if contains(types,"Angularvel")
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AngularVelX/10,'LineStyle',"-.", 'LineWidth',1, 'DisplayName', 'AngularVel-X');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AngularVelY/10,'LineStyle',"-.", 'LineWidth',1, 'DisplayName', 'AngularVel-Y');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.AngularVelZ/10,'LineStyle',"-.", 'LineWidth',1, 'DisplayName', 'AngularVel-Z');hold on;
            end

            if contains(types,"AngleZupt")
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Phi(:,1),'LineWidth',2,'LineStyle',"-",'DisplayName', 'Phi-X');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Phi(:,2),'LineWidth',2,'LineStyle',"-",'DisplayName', 'Phi-Y');hold on;
                plot(timeSeq(Utils.ALIGNER_STAMP:end),stateCalculator.mStateSeq.Phi(:,3),'LineWidth',2,'LineStyle',"-",'DisplayName', 'Phi-Z');hold on;

            end

            if contains(types,"Mag")
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.MagX/1000,'LineWidth',2, 'DisplayName', 'Mag-X');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.MagY/1000,'LineWidth',2, 'DisplayName', 'Mag-Y');hold on;
                plot(timeSeqRaw,stateCalculator.iHandler.mRawData.MagZ/1000,'LineWidth',2, 'DisplayName', 'Mag-Z');hold on;
            end

            %足底压力相关

            if contains(types,"AreaMaxPressure")
                plot(stateCalculator.pHandler.mSumSeqInArea.H_Max,'LineWidth',2, 'DisplayName', 'H-Max');hold on;
                plot(stateCalculator.pHandler.mSumSeqInArea.M_Max,'LineWidth',2, 'DisplayName', 'M-Max');hold on;
                plot(stateCalculator.pHandler.mSumSeqInArea.T_Max,'LineWidth',2, 'DisplayName', 'T-Max');hold on;
                plot(stateCalculator.pHandler.mSumSeqInArea.C_Max,'LineWidth',2, 'DisplayName', 'C-Max');hold on;
                plot(stateCalculator.pHandler.mSumSeqInArea.L_Max,'LineWidth',2, 'DisplayName', 'L-Max');hold on;
                plot(stateCalculator.pHandler.mSumSeqInArea.W_Max,'LineWidth',2, 'DisplayName', 'W-Max');hold on;

            end

            if contains(types,"COP")
                COPSeq = stateCalculator.pHandler.getCOPSeq();
                plot(timeSeq,COPSeq(:,1)*10, 'DisplayName', 'COP-X');hold on;
                plot(timeSeq,COPSeq(:,2)*10, 'DisplayName', 'COP-Y');hold on;
            end

            if contains(types,"pHandler.mWalkSpeed")
                plot(timeSeq,(stateCalculator.pHandler.mWalkSpeedSeq)*1, 'LineWidth',2,'DisplayName', '步行速度（m/s）','LineStyle',"-");hold on;
            end

            if contains(types,"Sum")
                plot(timeSeq,(stateCalculator.pHandler.mSumSeq-200)*1, 'LineWidth',2,'DisplayName', '足底压力总和','LineStyle',":");hold on;
            end

            legend('show');  % 自定义坐标
            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('综合结果显示图'); % 图形标题
            grid on; % 显示格线
            Plotter.onlyYAxisZoom(gca);
        end



        % @brief 步态、加速度、角速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU_AngularVel_Mag(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq*50,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase*50,'LineWidth',1);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,1));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,2));
            plot(timeSeq,stateCalculator.mStateSeq.AccENU(:,3));
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelX/10,'LineWidth',2);
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelY/10,'LineWidth',2);
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelZ/10,'LineWidth',2);

            plot(timeSeq,stateCalculator.iHandler.mRawData.MagX/1000,'LineWidth',2);
            plot(timeSeq,stateCalculator.iHandler.mRawData.MagY/1000,'LineWidth',2);
            plot(timeSeq,stateCalculator.iHandler.mRawData.MagZ/1000,'LineWidth',2);

            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('步态检测、东北天加速度、角速度汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合','AccENU_X','AccENU_Y','AccENU_Z','AngulearVel_X','AngulearVel_Y','AngulearVel_Z','Mag_X','Mag_Y','Mag_Z');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end






        % @brief 加速度、压力和、混合图
        % @param stateCalculator实例
        function plot_AngularVel_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelX/10);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelY/10);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AngularVelZ/10);hold on;
            plot(timeSeq,(stateCalculator.pHandler.mSumSeq-240)*1);hold on;

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('角速度——压力和汇总图'); % 图形标题
            grid on; % 显示格线
            legend('AngularVel_X','AngularVel_Y','AngularVel_Z','Sumseq');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            %             Plotter.onlyYAxisZoom(gca);
        end

        % @brief 修正后速度、COP速度图
        % @param stateCalculator实例
        function plot_V_COPVel(stateCalculator)
            timeSeq = stateCalculator.iHandler.mTSeq;
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


        % @brief 二维轨迹图
        % @param stateCalrArray实例数组
        function plot_Tracks_2D_with_Plantar(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            gcf = figure("Name","StateCalculator");%创建图窗
            %提取数据
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                Pe = stateCalr.mStateSeq.Pe;
                plot(P(:,1),P(:,2),'LineWidth',2,"DisplayName",stateCalr.mStateSeq.Legend);hold on;
                plot(Pe(1,:),Pe(2,:),'LineWidth',2,"DisplayName","足底压力推算结果");hold on;
            end

            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            title('二维轨迹图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
            legend("show");
        end

        %% 绘制PlantarHandler
        %% 热力图
        %绘制单帧热力图
        function gcf = drawHeatMapOfIndex(pHandler,index)
            gcf = Plotter.drawHeatMapOfSection(pHandler,index,index,1);
        end

        %持续绘制所有时间热力图 interval——绘图间隔（帧）
        function gcf = drawHeatMapOfAll(pHandler,interval)
            gcf = Plotter.drawHeatMapOfSection(pHandler,1,size(pHandler.mRawData.valueMat,1),interval);
        end


        % @brief 持续绘制热力图并保存成MP4
        % @param interval 间隔帧
        function drawHeatMapOfAllSaveToMp4(pHandler,interval)
            % 获取当前的系统时间
            currentTime = datetime('now');
            % 将当前时间转换为字符串，只包含月份、日期和时间
            strTime = datestr(currentTime, 'mm-dd_HH-MM-SS'); %#ok<*DATST>
            fileName = ['../result/HeatMap_OfInterval_',num2str(interval),'_',strTime,'.mp4'];
            % 创建一个 VideoWriter 对象，指定 'Profile' 为 'MPEG-4'
            v = VideoWriter(fileName, 'MPEG-4');
            %希望播放速度与真实速度一致，调整帧率
            v.FrameRate = 1000/(pHandler.mFrameInterval*interval);
            % 打开视频文件进行写入
            open(v);
            for i = 1:size(pHandler.mRawData.valueMat,1)
                gcf = Plotter.drawHeatMapOfIndex(pHandler,i);
                % 捕获此图形为帧
                frame = getframe(gcf);
                % 将帧写入视频
                writeVideo(v, frame);
            end
            % 关闭视频文件
            close(v);
        end


        % @brief 持续绘制介于两个点的热力图
        % @param startIndex——开始索引，endIndex——结束索引，interval——间隔帧，每几帧绘制一次
        function gcf = drawHeatMapOfSection(pHandler,startIndex,endIndex,interval)
            xPosVec = pHandler.mPosCoordMat(:,1);%位置序列的第一列
            xPosMax = max(xPosVec);
            xPosMin = min(xPosVec);
            yPosVec = pHandler.mPosCoordMat(:,2);%位置序列的第二列
            yPosMax = max(yPosVec);
            yPosMin = min(yPosVec);
            if(startIndex<0||startIndex>size(pHandler.mRawData.valueMat,1)||endIndex<0||endIndex>size(pHandler.mRawData.valueMat,1))
                disp("绘制热力图超过区间范围")
            end
            % 网格化x,y二维空间
            [X,Y] = meshgrid(linspace(xPosMin,xPosMax,pHandler.mNx),linspace(yPosMin,yPosMax,pHandler.mNy));
            %绘图配置
            %窗体配置
            if  isprop(pHandler, 'mHeatMapGcf') &&( isempty(pHandler.mHeatMapGcf )|| ~isvalid(pHandler.mHeatMapGcf)) %存在且为空
                %如果未创建，创建一个新视图
                pHandler.mHeatMapGcf = figure("Name","动态足底压力热力图",'Position', [200,100, 1000, 920]);
            end

            %             xlim([min(X(:)) max(X(:))]);%设置x坐标范围
            %             ylim([min(Y(:)) max(Y(:))]);

            %逐帧更新数据
            for i = startIndex:endIndex
                reminder = mod(i,interval);
                %如果余数不为0，返回
                if reminder ~= 0
                    continue;
                else
                    disp("plot")
                    %执行
                    frameVec = pHandler.getFrameProcessed(i);%获取第i帧数据

                    outlineVec = zeros(1,size(pHandler.mOutlineCoordMat,1));

                    valueVec = horzcat(frameVec,outlineVec);
                    % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
                    Z = griddata(xPosVec,yPosVec,valueVec,X,Y,'cubic');
                    figure(pHandler.mHeatMapGcf)%找到指定图窗
                    % 等高线法
                    contourf(X,Y,Z,pHandler.mNy, 'LineColor','none');
                    TimeSinceCollection = double(pHandler.mRawData.Timestamp(i)-pHandler.mRawData.Timestamp(1));%获取秒数
                    title(['这是第', num2str(TimeSinceCollection/1000, '%.3f'), '秒的数据'],'FontSize', 16); % 图形标题
                    colorbar;%启用颜色控制条
                    c = colorbar;
                    c.Label.String = '压力数值/N';
                    % 设置colorbar的范围
                    maxValue = max(frameVec(:));
                    defaultMaxValue = 20;
                    if maxValue >defaultMaxValue

                        set(gca, 'CLim', [0, maxValue]);
                    else
                        set(gca, 'CLim', [0, defaultMaxValue]);
                    end
                    colormap('jet');%设置颜色映射方案
                    axis tight manual;%设置坐标轴范围紧密包裹数据
                    axis equal;
                    xlabel('X/cm','FontSize', 16); % x轴注解
                    ylabel('Y/cm','FontSize', 16); % y轴注解
                    drawnow; % 强制立即更新图形
                end
            end
            gcf = pHandler.mHeatMapGcf;%结果赋值
        end


        %% 足底压力数据提取序列
        %绘制足底压力和时序图
        function plot_SumSeq(pHandlerArray)
            pHandlerArray = Plotter.toArray(pHandlerArray);
            legendArray =  strings(1, length(pHandlerArray));
            gcf = figure("Name","Plotter");%创建图窗
            for i = 1:length(pHandlerArray)
                pHandler = pHandlerArray(i);
                sumSeq = pHandler.getSumSeq();
                legendArray(i) = num2str(pHandler.mWalkSpeed);
            end
            xlabel('时间戳/ms'); % x轴注解
            ylabel('压力和/N'); % y轴注解
            title('足底压力时序图'); % 图形标题
            grid on; % 显示格线
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12
            legend(legendArray);
        end

        %绘制COP速度图
        function plot_COPVelSeq(pHandler)
            gcf = figure("Name","COP速度");
            copSeq = pHandler.getCOPVelSeq();
            plot(pHandler.mRawData.Timestamp,copSeq(:,1));hold on;
            plot(pHandler.mRawData.Timestamp,copSeq(:,2));
            xlabel('时间戳/ms'); % x轴注解
            ylabel('速度/cm/s'); % y轴注解
            title('COP速度图'); % 图形标题
            legend("X方向","Y方向");
            grid on; % 显示格线
            Plotter.onlyYAxisZoom(gca);
        end



        %COP速度模长
        function plot_COPVelNorm(pHandler)
            gcf = figure("Name","COP速度模长");
            copSeq = pHandler.getCOPVelSeq();
            copNormVec = zeros(size(copSeq,1),1);%初始化向量
            for i = 1:size(copNormVec,1)
                copNormVec(i) = norm(copSeq(i,:));
            end
            plot(pHandler.mRawData.Timestamp,copNormVec);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('速度/cm/s'); % y轴注解
            title('COP速度模长图'); % 图形标题
            grid on; % 显示格线
        end

        % @brief 绘制区间压力中心坐标
        % @param axisType——坐标轴选择，startIndex——开始索引，endIndex——结束索引
        function plot_COP_GaitPhase(pHandler,varargin)
            %解析设置参数
            Config = inputParser;
            addParameter(Config, 'axis',"All"); %选择绘制轴，默认全部绘制
            parse(Config, varargin{:});
            axis  = Config.Results.axis;
            gcf = figure("Name","PlantarHandler");
            COPSeq = pHandler.getCOPSeq();
            GaitPhaseSeq = pHandler.mGaitPhaseSeq*10;
            TimeSeq = pHandler.mRawData.Timestamp;
            % 根据传入的字符串参数进行逻辑判断
            if strcmpi(axis, 'X')
                % 绘制X轴数据
                plot(TimeSeq,COPSeq(:,1));hold on;
                plot(TimeSeq,GaitPhaseSeq,"LineWidth",2);hold on;
                legend("X方向");
            elseif strcmpi(axis, 'Y')
                % 绘制Y轴数据
                plot(TimeSeq,COPSeq(:,2));hold on;
                plot(TimeSeq,GaitPhaseSeq,"LineWidth",2);hold on;
                legend("Y方向");
            elseif strcmpi(axis, 'All')
                % 同时绘制XY轴数据
                plot(TimeSeq,COPSeq(:,1));hold on;
                plot(TimeSeq,COPSeq(:,2));hold on;
                plot(TimeSeq,GaitPhaseSeq,"LineWidth",2);hold on;
                legend("X方向","Y方向");
            end
            xlabel('时间戳/ms'); % x轴注解
            ylabel('坐标/cm'); % y轴注解
            title('压力中心坐标图'); % 图形标题
            grid on; % 显示格线
            Plotter.onlyYAxisZoom(gca);
        end

        % @brief 绘制插值前后的单帧数据和序列进行比较，用于判断插值前后时间戳有没有错位
        function plot_SumSeq_Original_Insert(pHandler)

            sumSeqAfterInsert = zeros(size(pHandler.mRawData.Timestamp,1),1);
            sumSeqBeforeInsert = zeros(size(pHandler.mRawDataBeforeInsert.Timestamp,1),1);
            for i = 1:size(sumSeqAfterInsert,1)
                sum = 0;
                for j = 1:pHandler.mFrameSize
                    sum = sum + pHandler.mRawData.valueMat(i,j);
                end
                sumSeqAfterInsert(i) = sum;
            end

            for i = 1:size(sumSeqBeforeInsert,1)
                sum = 0;
                for j = 1:pHandler.mFrameSize
                    sum = sum + pHandler.mRawDataBeforeInsert.valueMat(i,j);
                end
                sumSeqBeforeInsert(i) = sum;
            end
            gcf = figure("Name","PlantarHandler");
            plot(pHandler.mRawDataBeforeInsert.Timestamp,sumSeqBeforeInsert);hold on;
            plot(pHandler.mRawData.Timestamp,sumSeqAfterInsert);hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('足底压力和/N'); % y轴注解
            title('足底压力和序列图'); % 图形标题
            legend('插值前','插值后');
            grid on; % 显示格线
        end



        % @brief 绘制区间压力中心坐标
        % @param axisType——坐标轴选择，startIndex——开始索引，endIndex——结束索引
        function plot_COP_GaitPhase_Sum_InSection(pHandler,axisType,startIndex,endIndex)
            gcf = figure("Name","PlantarHandler");
            COPSeq = pHandler.mCOPSeq(startIndex:endIndex,:);
            GaitPhaseSeq = pHandler.mGaitPhaseSeq(startIndex:endIndex)*10;
            SumSeq = pHandler.mSumSeq(startIndex:endIndex)/100;
            TimeSeq = pHandler.mRawData.Timestamp(startIndex:endIndex,:);
            % 根据传入的字符串参数进行逻辑判断
            if strcmpi(axisType, 'X')
                % 绘制X轴数据
                plot(TimeSeq,COPSeq(:,1));hold on;
                plot(TimeSeq,GaitPhaseSeq);hold on;
                plot(TimeSeq,SumSeq);hold on;
                legend("X方向");
            elseif strcmpi(axisType, 'Y')
                % 绘制Y轴数据
                plot(TimeSeq,COPSeq(:,2));hold on;
                plot(TimeSeq,GaitPhaseSeq);hold on;
                plot(TimeSeq,SumSeq);hold on;
                legend("Y方向");
            elseif strcmpi(axisType, 'ALL')
                % 同时绘制XY轴数据
                plot(TimeSeq,COPSeq(:,1));hold on;
                plot(TimeSeq,COPSeq(:,2));hold on;
                plot(TimeSeq,GaitPhaseSeq);hold on;
                plot(TimeSeq,SumSeq);hold on;
                legend("X方向","Y方向");
            end
            xlabel('时间戳/ms'); % x轴注解
            ylabel('坐标/cm'); % y轴注解
            title('压力中心坐标图'); % 图形标题
            grid on; % 显示格线
        end

        % @brief 绘制多个plantarHandler内的压力分区数据峰值
        % @param pHandlerArray 足底压力数据数组
        function plot_PeakPressure_Speed(pHandlerArray)
            pHandlerArray = Plotter.toArray(pHandlerArray);
            gcf = figure("Name","Plotter");%创建图窗
            for i = 1:length(pHandlerArray)
                pHandler = pHandlerArray(i);
                SumSeqInArea = pHandler.mSumSeqInArea;
                speed = pHandler.mWalkSpeed;
                scatter(speed, mean(SumSeqInArea.H_peaks), 'o', 'SizeData', 50,'LineWidth', 2);hold on;
            end
            %绘图配置
            xlabel('步行速度(km/h)'); % x轴注解
            ylabel('压力峰值(N)'); % y轴注解
            title('足底压力峰值随步行速度变化图'); % 图形标题
            grid on; % 显示格线
        end

        % @brief 绘制区域压力和数据
        % @param pHandler 足底压力管理类
        function plot_SumInAreaSeq(pHandler,varargin)
            %解析参数
            Config = inputParser;
            addParameter(Config, 'type',"Sum");%绘制类型，默认为sum
            parse(Config, varargin{:});
            type = Config.Results.type;

            timestamps = pHandler.mRawData.Timestamp;
            gcf = figure("Name","Plotter");%创建图窗

            if(type == "Sum")


                plot(timestamps,pHandler.mSumSeqInArea.T_SumSeq);hold on;
                plot(timestamps,pHandler.mSumSeqInArea.M_SumSeq);hold on;
                plot(timestamps,pHandler.mSumSeqInArea.C_SumSeq);hold on;
                plot(timestamps,pHandler.mSumSeqInArea.L_SumSeq);hold on;
                plot(timestamps,pHandler.mSumSeqInArea.H_SumSeq);hold on;
                xlabel('时间戳（ms）'); % x轴注解
                ylabel('压力和(N)'); % y轴注解
                title('足底压力区域和随时间变化图'); % 图形标题
                legend("TSumSeq","MSumSeq","CSumSeq","LSumSeq","HSumSeq");
            elseif(type == 'Peak')
                scatter(pHandler.mSumSeqInArea.T_locations,pHandler.mSumSeqInArea.T_peaks,"LineWidth",2,"Color","red"    );hold on;
                scatter(pHandler.mSumSeqInArea.M_locations,pHandler.mSumSeqInArea.M_peaks,"LineWidth",2,"Color","green"  );hold on;
                scatter(pHandler.mSumSeqInArea.C_locations,pHandler.mSumSeqInArea.C_peaks,"LineWidth",2,"Color","blue"   );hold on;
                scatter(pHandler.mSumSeqInArea.L_locations,pHandler.mSumSeqInArea.L_peaks,"LineWidth",2,"Color","cyan"   );hold on;
                scatter(pHandler.mSumSeqInArea.H_locations,pHandler.mSumSeqInArea.H_peaks,"LineWidth",2,"Color","magenta");hold on;

                plot(pHandler.mSumSeqInArea.T_locations,pHandler.mSumSeqInArea.T_peaks,"LineWidth",1,"Color","red"    );hold on;
                plot(pHandler.mSumSeqInArea.M_locations,pHandler.mSumSeqInArea.M_peaks,"LineWidth",1,"Color","green"  );hold on;
                plot(pHandler.mSumSeqInArea.C_locations,pHandler.mSumSeqInArea.C_peaks,"LineWidth",1,"Color","blue"   );hold on;
                plot(pHandler.mSumSeqInArea.L_locations,pHandler.mSumSeqInArea.L_peaks,"LineWidth",1,"Color","cyan"   );hold on;
                plot(pHandler.mSumSeqInArea.H_locations,pHandler.mSumSeqInArea.H_peaks,"LineWidth",1,"Color","magenta");hold on;

                xlabel('索引'); % x轴注解
                ylabel('压力峰值(N)'); % y轴注解
                title('足底压力区域和峰值随时间变化图'); % 图形标题
                legend("TPeaks","MPeaks","CPeaks","LPeaks","HPeaks");
            elseif(type == 'SecPeak')
                scatter(pHandler.mSumSeqInArea.T_Sec_locations,pHandler.mSumSeqInArea.T_Sec_peaks,"LineWidth",2,"Color","red"    );hold on;
                scatter(pHandler.mSumSeqInArea.M_Sec_locations,pHandler.mSumSeqInArea.M_Sec_peaks,"LineWidth",2,"Color","green"  );hold on;
                scatter(pHandler.mSumSeqInArea.C_Sec_locations,pHandler.mSumSeqInArea.C_Sec_peaks,"LineWidth",2,"Color","blue"   );hold on;
                scatter(pHandler.mSumSeqInArea.L_Sec_locations,pHandler.mSumSeqInArea.L_Sec_peaks,"LineWidth",2,"Color","cyan"   );hold on;
                scatter(pHandler.mSumSeqInArea.H_Sec_locations,pHandler.mSumSeqInArea.H_Sec_peaks,"LineWidth",2,"Color","magenta");hold on;

                plot(pHandler.mSumSeqInArea.T_Sec_locations,pHandler.mSumSeqInArea.T_Sec_peaks,"Color","red"    );hold on;
                plot(pHandler.mSumSeqInArea.M_Sec_locations,pHandler.mSumSeqInArea.M_Sec_peaks,"Color","green"  );hold on;
                plot(pHandler.mSumSeqInArea.C_Sec_locations,pHandler.mSumSeqInArea.C_Sec_peaks,"Color","blue"   );hold on;
                plot(pHandler.mSumSeqInArea.L_Sec_locations,pHandler.mSumSeqInArea.L_Sec_peaks,"Color","cyan"   );hold on;
                plot(pHandler.mSumSeqInArea.H_Sec_locations,pHandler.mSumSeqInArea.H_Sec_peaks,"Color","magenta");hold on;

                xlabel('索引'); % x轴注解
                ylabel('压力二次峰值(N)'); % y轴注解
                title('足底压力区域和二次峰值随时间变化图'); % 图形标题
                legend("TSecPeaks","MSecPeaks","CSecPeaks","LSecPeaks","HSecPeaks");

            end
            grid on; % 显示格线
            Plotter.onlyYAxisZoom(gca);
        end



        %% 绘制ImuHandler




        %% 绘制最终结果图

        % @brief 绘制方形
        % @param stateCalrArray实例数组
        % @retval None
        function gcf = plot_Tracks_Square(stateCalrArray,varargin)
            %解析参数
            settings = inputParser;
            addParameter(settings,'P1_angle',50);%P1的旋转角度
            addParameter(settings,'P2_angle',50);%P2的旋转角度
            parse(settings,varargin{:});
            P1_angle = settings.Results.P1_angle;
            P2_angle = settings.Results.P2_angle;


            stateCalrArray = Plotter.toArray(stateCalrArray);
            gcf = figure("Name","Square");%创建图窗
%             set(gca, 'Color', 'none');            % 设置坐标区域背景为透明
%             set(gcf, 'Color', 'none');            % 设置图形背景为透明

            %绘制Proposed
            P1 = stateCalrArray(1).mStateSeq.P';
            P1_adjust = TrackAdjuster.rotate2D(P1(1,:),P1(2,:),P1_angle);
            plot(P1_adjust(1,:),P1_adjust(2,:),'r-','LineWidth',2,'DisplayName','Proposed ZUPT');hold on;

            %绘制Traditional
            P2 = stateCalrArray(2).mStateSeq.P';
            P2_adjust = TrackAdjuster.rotate2D(P2(1,:),P2(2,:),P2_angle);
            plot(P2_adjust(1,:),P2_adjust(2,:),'b-','LineWidth',2,'DisplayName','Traditional ZUPT');hold on;

            %绘制参考轨迹
            plot([0,0,-16,-16,0],[0,19.5,19.5,0,0],'g--','LineWidth',2,'DisplayName','Reference Path');

            %绘制结束点
            plot(P1_adjust(1,end),P1_adjust(2,end),'ro','LineWidth',2,'DisplayName','Proposed ZUPT EndPoint');hold on;
            plot(P2_adjust(1,end),P2_adjust(2,end),'bo','LineWidth',2,'DisplayName','Traditional ZUPT EndPoint');hold on;

            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解0000h
            title('室内方形场地测试图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            legend('show');
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12

        end

        % @brief 绘制机械楼
        % @param stateCalrArray实例数组
        % @retval None
        function gcf = plot_Tracks_JiXieLou(stateCalrArray,varargin)
            %解析参数
            settings = inputParser;
            addParameter(settings,'P1_angle',50);%P1的旋转角度
            addParameter(settings,'P2_angle',50);%P2的旋转角度
            addParameter(settings,'refTrackIndex',1);%参考轨迹序号
            parse(settings,varargin{:});
            P1_angle = settings.Results.P1_angle;
            P2_angle = settings.Results.P2_angle;
            refTrackIndex = settings.Results.refTrackIndex;
            
            % 将 stateCalrArray 转换为数组
            stateCalrArray = Plotter.toArray(stateCalrArray);

            % 创建图窗并最大化显示（保留工具栏和菜单栏）
            gcf = figure('Name', 'JiXieLou'); % 创建图窗
            set(gcf, 'WindowState', 'maximized'); % 将图窗最大化显示
            % 绘制 Proposed ZUPT
            P1 = stateCalrArray(1).mStateSeq.P';
            P1_adjust = TrackAdjuster.rotate2D(P1(1,:), P1(2,:), P1_angle);
            plot(P1_adjust(1,:), P1_adjust(2,:), 'r-', 'LineWidth', 2, 'DisplayName', 'Proposed ZUPT'); hold on;

            % 绘制 Traditional ZUPT
            P2 = stateCalrArray(2).mStateSeq.P';
            P2_adjust = TrackAdjuster.rotate2D(P2(1,:), P2(2,:), P2_angle);
            plot(P2_adjust(1,:), P2_adjust(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Traditional ZUPT'); hold on;

            % 定义参考轨迹
            if refTrackIndex == 1
                trajectory_ref = [
                    0, -16.5, -17, -68.99, -69.15, -84.55, -86.64, -2.6, 0;  % x 坐标
                    0, 2, 10.33, 8.49, 0.22, -1.54, -81.84, -80.66, 0       % y 坐标
                    ];
            elseif refTrackIndex == 2
                trajectory_ref = [
                    0, -12, -13, -68.99, -69.15, -84.55, -86.64, -2.6, 0;  % x 坐标
                    0, 0, 9, 9.5, 0.22, 1.54, -81.84, -80.66, 0       % y 坐标
                    ];
            

%                 trajectory_ref = [
%                     0, -12, -14.4, -67.8, -68.6,-84.9, -89.2, -3.57, 0;  % x 坐标
%                     0, -2.5, 5.69, 5.69,-2.33, -2.58, -83.18, -89.01, 0       % y 坐标
%                     ];
      
            end
     
            % 提取 x 和 y 坐标
            x_ref = trajectory_ref(1, :);
            y_ref = trajectory_ref(2, :);

            % 绘制参考轨迹
            plot(x_ref, y_ref, 'g--', 'LineWidth', 2, 'DisplayName', 'Reference Path');

            % 绘制结束点
            plot(P1_adjust(1,end), P1_adjust(2,end), 'ro', 'LineWidth', 2, 'DisplayName', 'Proposed ZUPT EndPoint'); hold on;
            plot(P2_adjust(1,end), P2_adjust(2,end), 'bo', 'LineWidth', 2, 'DisplayName', 'Traditional ZUPT EndPoint'); hold on;

            % 标注
            xlabel('X方向/米', 'FontSize', 16);
            ylabel('Y方向/米', 'FontSize', 16);
            title('室内方形场地测试图');
            grid on;
            axis equal;
            legend('show');
            set(gca, 'FontSize', 12);
        end
        


        % @brief 绘制机械楼数组
        % @param stateCalrArray实例数组
        % @retval None
        function gcf = plot_Tracks_JiXieLou_Array(stateCalrArray,varargin)
            %解析参数
            settings = inputParser;
            addParameter(settings,'P1_angle',50);%P1的旋转角度
            addParameter(settings,'P2_angle',50);%P2的旋转角度
            addParameter(settings,'refTrackIndex',1);%参考轨迹序号
            parse(settings,varargin{:});
            P1_angle = settings.Results.P1_angle;
            P2_angle = settings.Results.P2_angle;
            refTrackIndex = settings.Results.refTrackIndex;

            % 将 stateCalrArray 转换为数组
            stateCalrArray = Plotter.toArray(stateCalrArray);

            % 创建图窗并最大化显示（保留工具栏和菜单栏）
            gcf = figure('Name', 'JiXieLouArray'); % 创建图窗
            set(gcf, 'WindowState', 'maximized'); % 将图窗最大化显示
            
            for i = 1:length(stateCalrArray)
                % 绘制
                P1 = stateCalrArray(i).mStateSeq.P';
     
                P1_adjust = TrackAdjuster.rotate2D(P1(1,:), P1(2,:), P1_angle);
                str = "track " + num2str(i);
                plot(P1_adjust(1,:), P1_adjust(2,:), 'LineWidth', 2, 'DisplayName', str); hold on;
            end

            % 定义参考轨迹
            if refTrackIndex == 1
                trajectory_ref = [
                    0, -16.5, -17, -68.99, -69.15, -84.55, -86.64, -2.6, 0;  % x 坐标
                    0, 2, 10.33, 8.49, 0.22, -1.54, -81.84, -80.66, 0       % y 坐标
                    ];
            elseif refTrackIndex == 2
                trajectory_ref = [
                    0, -12, -14.4, -67.8, -68.6,-84.9, -89.2, -3.57, 0;  % x 坐标
                    0, -0, 5.69, 5.69,-2.33, -2.58, -83.18, -85, 0       % y 坐标
                    ];

%                 trajectory_ref = [
%                     0, -12, -14.4, -67.8, -68.6,-84.9, -89.2, -3.57, 0;  % x 坐标
%                     0, -2.5, 5.69, 5.69,-2.33, -2.58, -83.18, -89.01, 0       % y 坐标
%                     ];
      
            end
     
            % 提取 x 和 y 坐标
            x_ref = trajectory_ref(1, :);
            y_ref = trajectory_ref(2, :);

            % 绘制参考轨迹
            plot(x_ref, y_ref, 'g--', 'LineWidth', 2, 'DisplayName', 'Reference Path');

            % 标注
            xlabel('X方向/米', 'FontSize', 16);
            ylabel('Y方向/米', 'FontSize', 16);
            title('室内方形场地测试图');
            grid on;
            axis equal;
            legend('show');
            set(gca, 'FontSize', 12);
        end


        % @brief 绘制直线
        % @param stateCalrArray实例数组
        % @retval None
%         function gcf = plot_Tracks_Line(stateCalrArray,varargin)
%             %解析参数
%             settings = inputParser;
%             addParameter(settings,'exprName',"");%P1的旋转角度
%             parse(settings,varargin{:});
%             exprName = settings.Results.exprName;
%             
% 
% 
%             % 将 stateCalrArray 转换为数组
%             stateCalrArray = Plotter.toArray(stateCalrArray);
% 
%             % 创建图窗并最大化显示（保留工具栏和菜单栏）
%             gcf = figure('Name', 'Line'); % 创建图窗
%             set(gcf, 'WindowState', 'maximized'); % 将图窗最大化显示
%             
%             % 读取背景图片
%             backgroundImage = imread('操场地图.png'); % 替换成你想用作背景的图片路径
%             % 显示背景图片并调整坐标轴
%             imshow(backgroundImage, 'XData', [0 1], 'YData', [0 1]); % 调整坐标范围以适配
%             hold on;
%             
%             
%             for i = 1:length(stateCalrArray)
%                 % 绘制
%                 P1 = stateCalrArray(i).mStateSeq.P';
%                 i
%                 X_mean = mean(P1(1,:))
%                 Y_mean = mean(P1(2,:))
%                 P1_angle =  rad2deg(-atan2(mean(P1(2,:)) , mean(P1(1,:))))
%                 P1_adjust = TrackAdjuster.rotate2D(P1(1,:), P1(2,:), P1_angle);
%                 str = "track " + num2str(i);
%                 plot(P1_adjust(1,:), P1_adjust(2,:), 'r-', 'LineWidth',0.5, 'DisplayName', str); hold on;
%             end
% 
% 
% 
% 
%             %             % 定义参考轨迹
%             %             trajectory_ref = [
%             %                 0, -16.5, -17, -68.99, -69.15, -84.55, -86.64, -2.6, 0;  % x 坐标
%             %                 0, 2, 10.33, 8.49, 0.22, -1.54, -81.84, -80.66, 0       % y 坐标
%             %                 ];
%             %
%             %             % 提取 x 和 y 坐标
%             %             x_ref = trajectory_ref(1, :);
%             %             y_ref = trajectory_ref(2, :);
%             %
%             %             % 绘制参考轨迹
%             %             plot(x_ref, y_ref, 'g--', 'LineWidth', 2, 'DisplayName', 'Reference Path');
%             %
%             %             % 绘制结束点
%             %             plot(P1_adjust(1,end), P1_adjust(2,end), 'ro', 'LineWidth', 2, 'DisplayName', 'Proposed ZUPT EndPoint'); hold on;
%             %             plot(P2_adjust(1,end), P2_adjust(2,end), 'bo', 'LineWidth', 2, 'DisplayName', 'Traditional ZUPT EndPoint'); hold on;
% 
%             % 标注
%             xlabel('X方向/米', 'FontSize', 16);
%             ylabel('Y方向/米', 'FontSize', 16);
%             titleStr = exprName + "直线测试图";
%             title(titleStr);
%             grid on;
%             axis equal;
%             legend('show');
%             set(gca, 'FontSize', 12);
%         end
        


      function gcf = plot_Tracks_Line(stateCalrArray, varargin)
    % 解析参数
    settings = inputParser;
    addParameter(settings, 'exprName', ""); % P1的旋转角度
    parse(settings, varargin{:});
    exprName = settings.Results.exprName;
    
    % 将 stateCalrArray 转换为数组
    stateCalrArray = Plotter.toArray(stateCalrArray);
    
    % 创建图窗并最大化显示
    gcf = figure('Name', 'Line');
    set(gcf, 'WindowState', 'maximized');
    

    % 获取所有数据的 X 和 Y 范围
    allX = [];
    allY = [];
    % 遍历 stateCalrArray 并绘制数据
    for i = 1:length(stateCalrArray)
        P1 = stateCalrArray(i).mStateSeq.P';
        
        % 计算平均值和旋转角度
        P1_angle = rad2deg(-atan2(mean(P1(2, :)), mean(P1(1, :))));
        
        % 调整轨迹旋转角度
        P1_adjust = TrackAdjuster.rotate2D(P1(1, :), P1(2, :), P1_angle);
        
        allX = [allX, P1_adjust(1, :)];
        allY = [allY, P1_adjust(2, :)];
    end
    
 
    minX = min(allX)
    maxX = max(allX)
    minY = min(allY)
    maxY = max(allY)
    
    % 读取背景图片
    backgroundImage = imread('操场地图.png'); % 替换成你想用作背景的图片路径
    
    % 显示背景图片并将其缩放到数据范围
    imagesc([minX maxX], [minY maxY], flipud(backgroundImage)); 
    colormap gray; % 设置颜色映射，如果需要不同的背景颜色可以调整
    hold on; % 保持图窗，使其他绘图可以叠加在背景图上
    
    % 调整坐标轴
    axis on; % 打开坐标轴
    set(gca, 'YDir', 'normal'); % 确保 Y 轴方向正常
    axis equal; % 保持比例
    xlim([minX maxX]); % 根据数据范围调整 X 轴
    ylim([minY maxY]); % 根据数据范围调整 Y 轴
    
    % 遍历 stateCalrArray 并绘制数据
    for i = 1:length(stateCalrArray)
        P1 = stateCalrArray(i).mStateSeq.P';
        
        % 计算平均值和旋转角度
        P1_angle = rad2deg(-atan2(mean(P1(2, :)), mean(P1(1, :))));
        
        % 调整轨迹旋转角度
        P1_adjust = TrackAdjuster.rotate2D(P1(1, :), P1(2, :), P1_angle);
        
        % 绘制轨迹
        str = "track " + num2str(i);
        plot(P1_adjust(1, :), P1_adjust(2, :), 'r-', 'LineWidth', 0.5, 'DisplayName', str); 
        hold on;
    end
    
    % 标注和图例
    xlabel('X方向/米', 'FontSize', 16);
    ylabel('Y方向/米', 'FontSize', 16);
    titleStr = exprName + " 直线测试图";
    title(titleStr);
    grid on;
    axis equal;
    legend('show');
    set(gca, 'FontSize', 12);
end



    end
end

