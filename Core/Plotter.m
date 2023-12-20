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

        % @brief 将绘图区设置为只有Y轴缩放的装饰器
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
                P(end,:)
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
        function plot_Tracks_2D(stateCalrArray)
            stateCalrArray = Plotter.toArray(stateCalrArray);
            legendArray = [];
            gcf = figure("Name","StateCalculator");%创建图窗
            %提取数据
            for i = 1:length(stateCalrArray)
                stateCalr = stateCalrArray(i);
                P = stateCalr.mStateSeq.P;
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
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
                legendArray = [legendArray,stateCalr.mStateSeq.Legend];
                plot(P(:,1),P(:,2),'LineWidth',2);hold on;
                plot(Pe(:,1),Pe(:,2),'LineWidth',2);hold on;
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
            plot(timeSeq,stateCalculator.iHandler.mGaitDtr.gaitPhaseSeq,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.pHandler.mGaitPhaseSeq,'LineWidth',2);hold on;
            plot(timeSeq,stateCalculator.mStateSeq.GaitPhase,'LineWidth',1);hold on;
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

        % @brief 步态和东北天加速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
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



        % @brief 步态、东北天加速度、角速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU_AngularVel(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
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
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('步态检测、东北天加速度、角速度汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合','AccENU_X','AccENU_Y','AccENU_Z','AngulearVel_X','AngulearVel_Y','AngulearVel_Z');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            Plotter.onlyYAxisZoom(gca);
        end


        % @brief 步态、加速度、角速度混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU_AngularVel_Mag(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
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



        % @brief 步态和东北天加速度、压力和混合图
        % @param stateCalculator实例
        function plot_Gait_AccENU_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
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


        % @brief 加速度、压力和、混合图
        % @param stateCalculator实例
        function plot_AccRaw_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
            gcf = figure("Name","StateCalculator");%创建图窗
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccX);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccY);hold on;
            plot(timeSeq,stateCalculator.iHandler.mRawData.AccZ);hold on;
            plot(timeSeq,(stateCalculator.pHandler.mSumSeq-200)*1);hold on;

            xlabel('时间戳/ms'); % x轴注解
            ylabel('结果'); % y轴注解
            title('加速度——压力和汇总图'); % 图形标题
            grid on; % 显示格线
            legend('Acc_X','Acc_Y','Acc_Z','Sumseq');
            maxValue = max(max(stateCalculator.iHandler.mFSeq));
            minValue = min(min(stateCalculator.iHandler.mFSeq));
            ylim([minValue maxValue])
            %             Plotter.onlyYAxisZoom(gca);
        end

        % @brief 加速度、压力和、混合图
        % @param stateCalculator实例
        function plot_AngularVel_Sum(stateCalculator)
            timeSeq = stateCalculator.iHandler.mRawData.Timestamp;
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

    %% 绘制PlantarHandler
    %绘制单帧热力图
    function gcf = drawHeatMapOfIndex(pHandler,index)
        gcf = pHandler.drawHeatMapOfSection(index,index,1);
    end

    %持续绘制所有时间热力图 interval——绘图间隔（帧）
    function gcf = drawHeatMapOfAll(pHandler,interval)
        gcf = pHandler.drawHeatMapOfSection(1,size(pHandler.mRawData.valueMat,1),interval);
    end

    %持续绘制并保存成MP4
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
            gcf = pHandler.drawHeatMapOfIndex(i);
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
                %执行
                frameVec = pHandler.getFrameProcessed(i);%获取第i帧数据

                outlineVec = zeros(1,size(pHandler.mOutlineCoordMat,1));

                valueVec = horzcat(frameVec,outlineVec);
                % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
                Z = griddata(xPosVec,yPosVec,valueVec,X,Y,'cubic');
                figure(pHandler.mHeatMapGcf)%找到指定图窗
                % 等高线法
                contourf(X,Y,Z,pHandler.mNy, 'LineColor','none');
                TimeSinceCollection = double(pHandler.mRawData.Timestamp(i)-pHandler.mRawData.Timestamp(0));%获取秒数
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
    end

    %绘制COP速度图
    function plot_COPVelYSeq(pHandler)
        gcf = figure("Name","COP速度");
        copSeq = pHandler.getCOPVelSeq();
        %             plot(pHandler.mRawData.Timestamp,copSeq(:,1));hold on;
        plot(pHandler.mRawData.Timestamp,copSeq(:,2));
        xlabel('时间戳/ms'); % x轴注解
        ylabel('速度/cm/s'); % y轴注解
        title('COP速度图'); % 图形标题
        legend("Y方向");
        grid on; % 显示格线
        ax = gca;
        zoomInt = zoomInteraction('Dimensions','x'); % 仅在 x 轴方向上缩放
        panInt = panInteraction; % 添加拖放功能
        dataTipInt = dataTipInteraction; % 添加数据提示功能
        ax.Interactions = [zoomInt, panInt, dataTipInt]; % 同时启用这些交互
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
    % @param axisType——坐标轴选择
    function plot_COPSeqAll(pHandler,axisType)
        pHandler.plot_COPSeqInSection(axisType,1,pHandler.mSeqLength);
    end

    % @brief 绘制区间压力中心坐标
    % @param axisType——坐标轴选择，startIndex——开始索引，endIndex——结束索引
    function plot_COPSeqInSection(pHandler,axisType,startIndex,endIndex)
        gcf = figure("Name","PlantarHandler");
        COPSeq = pHandler.getCOPSeq();
        COPSeq = COPSeq(startIndex:endIndex,:);
        TimeSeq = pHandler.mRawData.Timestamp(startIndex:endIndex,:);
        % 根据传入的字符串参数进行逻辑判断
        if strcmpi(axisType, 'X')
            % 绘制X轴数据
            plot(TimeSeq,COPSeq(:,1));hold on;
            legend("X方向");
        elseif strcmpi(axisType, 'Y')
            % 绘制Y轴数据
            plot(TimeSeq,COPSeq(:,2));hold on;
            legend("Y方向");
        elseif strcmpi(axisType, 'ALL')
            % 同时绘制XY轴数据
            plot(TimeSeq,COPSeq(:,1));hold on;
            plot(TimeSeq,COPSeq(:,2));hold on;
            legend("X方向","Y方向");
        end
        xlabel('时间戳/ms'); % x轴注解
        ylabel('坐标/cm'); % y轴注解
        title('压力中心坐标图'); % 图形标题
        grid on; % 显示格线
        Plotter.onlyYAxisZoom(gca);
    end

    % @brief 绘制区间压力中心坐标
    % @param axisType——坐标轴选择，startIndex——开始索引，endIndex——结束索引
    function plot_COP_GaitPhase_InSection(pHandler,axisType,startIndex,endIndex)
        gcf = figure("Name","PlantarHandler");
        COPSeq = pHandler.getCOPSeq();
        COPSeq = COPSeq(startIndex:endIndex,:);
        GaitPhaseSeq = pHandler.mGaitPhaseSeq(startIndex:endIndex)*10;
        TimeSeq = pHandler.mRawData.Timestamp(startIndex:endIndex,:);
        % 根据传入的字符串参数进行逻辑判断
        if strcmpi(axisType, 'X')
            % 绘制X轴数据
            plot(TimeSeq,COPSeq(:,1));hold on;
            plot(TimeSeq,GaitPhaseSeq);hold on;
            legend("X方向");
        elseif strcmpi(axisType, 'Y')
            % 绘制Y轴数据
            plot(TimeSeq,COPSeq(:,2));hold on;
            plot(TimeSeq,GaitPhaseSeq);hold on;
            legend("Y方向");
        elseif strcmpi(axisType, 'ALL')
            % 同时绘制XY轴数据
            plot(TimeSeq,COPSeq(:,1));hold on;
            plot(TimeSeq,COPSeq(:,2));hold on;
            plot(TimeSeq,GaitPhaseSeq);hold on;
            legend("X方向","Y方向");
        end
        xlabel('时间戳/ms'); % x轴注解
        ylabel('坐标/cm'); % y轴注解
        title('压力中心坐标图'); % 图形标题
        grid on; % 显示格线
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





    

    %% 绘制ImuHandler













    
    end
end

