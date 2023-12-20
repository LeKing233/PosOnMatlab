classdef PlantarHandler <handle


    properties(Access = public)
        %常量
        mFilePath;%文件路径
        mPosCoordMat;%定义压力数据点位置
        mSensorCoordMat;%定义传感器坐标位置矩阵
        mOutlineCoordMat;%轮廓坐标矩阵
        mFrameSize;%帧长度
        mSeqLength;%序列长度
        mNx;%X方向上网格数
        mNy;%Y方向上网格数
        mFrameInterval;%帧间隔时间（毫秒）
        mFrameIntervalIMU;%imu帧间隔


        %变量
        mRawTable;%原始数据表
        mRawData;%原始数据
        mRawDataBeforeInsert;%插值前的原始数据
        mPressureValueMat;%压力点阵数据
        mProdData;%处理过数据
        mVoltToPressureData;%压力转换中间量
        mHeatMapGcf;%热力图figure句柄
        mFid;%文件句柄

        %区域数据提取
        mSumSeqInArea;%区域压力和结构体
        mWalkSpeed;%步行速度

        %计算结果
        mSumSeq;
        mCOPSeq;
        mCOPVelSeq;
        mGaitPhaseSeq;
        mLogicResults;
        mHeelLocs;  
        mWalkSpeedSeq;%步长序列

        %依赖组件
        iHandler;%imu管理类
    end

    %公共方法
    methods(Access = public)
        %% 初始化
        %构造函数
        function obj = PlantarHandler(filePath,imuHandler,varargin)
            %解析plantar设置参数
            Config = inputParser;
            addParameter(Config, 'WalkSpeed',-1); %步行速度
            parse(Config, varargin{:});
            obj.mWalkSpeed = Config.Results.WalkSpeed;

            %配置
            obj.mFilePath = filePath;
            obj.iHandler = imuHandler;

            %提取数据
            obj.extractData();
            obj.matchRawDataWithIMU(obj.iHandler.mRawData.Timestamp);

            obj.init();%初始化
            obj.calculateRequireData();%计算所需数据
        end

        % 提取数据
        function extractData(obj)
            obj.mRawTable = readtable(obj.mFilePath);
            obj.mPressureValueMat = zeros(height(obj.mRawTable), 45);
            % 在脚本或函数开始时打开文件
            %             obj.mFid = fopen('intermediate_results1.txt', 'a');
            for i = 2:height(obj.mRawTable) % 从第二行开始遍历
                for j = 1:size(obj.mPressureValueMat, 2)
                    pressureValue = obj.voltageToForce(obj.mRawTable{i, j + 1});

                    % 错误帧过滤 检查当前行的压力值是否超过阈值                    
                    if pressureValue > 2000
                        % 如果不是第一行，则使用前一行替代
                        if i > 1
                            obj.mPressureValueMat(i, :) = obj.mPressureValueMat(i - 1, :);
                        else
                            % 如果是第一行，则使用下一行替代
                            obj.mPressureValueMat(i, :) = obj.mPressureValueMat(i + 1, :);
                        end
                        % 替代完成后退出内层循环
                        break;
                    else
                        % 如果条件不满足，更新压力值矩阵
                        obj.mPressureValueMat(i, j) = pressureValue;
                    end
                end
            end

            %             % 在程序结束时关闭文件
            %             finishup = onCleanup(@() fclose(obj.mFid));
            obj.mRawData.Timestamp = obj.mRawTable.Timestamp;
            obj.mRawData.valueMat = obj.mPressureValueMat;%i行 45列
        end

        function matchRawDataWithIMU(obj, imuTimeSeq)
            % 由于硬件端的时间不匹配，将 plantar 时间戳整体偏移
            obj.mRawData.Timestamp = obj.mRawData.Timestamp;
            originalRawData = obj.mRawData; % 缓存插值之前的
            obj.mRawDataBeforeInsert = obj.mRawData;

            % 指定备选插值方法：'linear'、'nearest'、'next'、'previous'、'pchip'、'cubic'、'v5cubic'、'makima' 或 'spline'。默认方法为 'linear'。
            obj.mRawData.valueMat = interp1(double(obj.mRawDataBeforeInsert.Timestamp), obj.mRawDataBeforeInsert.valueMat, double(imuTimeSeq), 'linear');
            obj.mRawData.Timestamp = imuTimeSeq;

            % 处理头尾数据可能出现的 NaN 值
            obj.mRawData.valueMat = fillmissing(obj.mRawData.valueMat, 'nearest');

            % 将所有时间戳重新转换成 int32
            obj.mRawData.Timestamp = int32(obj.mRawData.Timestamp);
        end




        % @brief 将电压值转换成压力值
        % @param 电压值（uint16)  (mV）
        % @retval 压力值（牛顿）
        function force = voltageToForce(obj, volt)
            voltage = volt ./ 1000; % 伏特
            R = (3630 ./ voltage - 100) ./ 1000; % kohms千欧姆
            pho = 1 ./ R; % 电导率

            % 调用拟合公式进行拟合
            a = 4.241;
            b = 3.211;

            force = a * exp(b * pho);

            %             % 保存中间量到文件
            %             obj.saveIntermediateResult(voltage, R, pho, force);
        end

        function saveIntermediateResult(obj,voltage, R, pho, force)
            % 将中间量写入文件
            fprintf(obj.mFid, 'Voltage: %.4f, R: %.4f, Pho: %.4f, Force: %.4f\n', voltage, R, pho, force);

        end


        %初始化
        function init(obj)
            %按照发送格式的45个点顺序，对左脚来说，以左下角为坐标原点，进行坐标录入
            %每行从左向右，从脚跟起向上计数
            obj.mSensorCoordMat =   [1.2 0.8;  1.7 0.6;  2.3 0.8;  %第一行
                0.9 1.8;  1.5 1.8;  2.0 1.8;  2.6 1.8;  %第二行
                0.9 2.8;  1.5 2.8;  2.0 2.8;  2.6 2.8;  %第三行
                0.9 3.9;  1.5 3.9;  2.0 3.9;  2.6 3.9;  %第四行
                0.7 5.0;  1.4 5.0;  2.0 5.0;  2.6 5.0;  %第五行
                0.5 6.2;  1.1 6.2;  1.7 6.2;  2.2 6.2;  2.8 6.2;%第六行
                0.4 7.3;  1.0 7.3;  1.5 7.3;  2.0 7.3;  2.5 7.3; 3.0 7.3;%第七行
                0.6 8.4;  1.1 8.4;  1.7 8.4;  2.2 8.4;  2.8 8.4; 3.3 8.4;%第八行
                1.1 9.5;  1.7 9.5;  2.2 9.5;  2.8 9.5;  3.3 9.5;%第九行
                1.5 10.4; 2.0 10.5; 2.5 10.5; 3.2 10.4]; %第十行

            %足底轮廓坐标
            obj.mOutlineCoordMat = [1.5 0.0;  2.0 0.0;  2.6 0.3;  2.9 0.9;  3.0 1.9;  3.0 2.8;  3.0 3.7;  3 4.4; 3.1 5.5;   %右下1/4
                3.4 6.4;  3.6 7.5;  3.7 8.3;  3.8 9.0;  3.5 10.0; 3.1 10.8; 2.1 11.0;                   %右上1/4
                1.5 10.9; 1.0 10.5; 0.7 9.9;  0.5 9.1;  0.2 8.1;  0.0 7.4;  0.1 6.6;  0.1 5.9;          %左上1/4
                0.2 5.2;  0.3 4.4;  0.35 3.7; 0.4 3.0;  0.4 2.1;  0.5 1.1;  0.9 0.4 ];                   %左下1/4
            %对轮廓进行插值
            % 原始数据
            originalData = obj.mOutlineCoordMat;

            % 提取x和y坐标
            x = originalData(:, 1);
            y = originalData(:, 2);

            % 生成参数化坐标点
            t = 1:length(x); % 使用整数作为参数，可以根据实际情况调整
            ts = linspace(1, length(x), 1000); % 生成更多点的参数
            newX = interp1(t, x, ts, 'spline'); % 使用样条插值方法
            newY = interp1(t, y, ts, 'spline'); % 使用样条插值方法
            % 更新 obj.mOutlineCoordMat
            obj.mOutlineCoordMat = [newX', newY'];

            %拼接得到坐标矩阵
            obj.mPosCoordMat = vertcat(obj.mSensorCoordMat, obj.mOutlineCoordMat);

            obj.mNx = 40;%X方向上网格数
            obj.mNy = 110;%Y方向上网格数
            obj.mSeqLength = size(obj.mRawData.valueMat,1);%行数作为序列长度
            obj.mFrameSize = size(obj.mSensorCoordMat,1);%单帧压力点数
            %序列间隔时间，取平均值得到
            obj.mFrameInterval = double((obj.mRawData.Timestamp(end)-obj.mRawData.Timestamp(1))/obj.mSeqLength);
            obj.mFrameIntervalIMU = 6;
            obj.preProcessData();%对数据进行预处理
        end

        %获取需要的数据
        function calculateRequireData(obj)
            obj.mSumSeq = obj.getSumSeq();
            obj.mCOPSeq = obj.getCOPSeq();
            obj.mCOPVelSeq = obj.getCOPVelSeq();
            obj.getGaitPhaseSeqByCOP();
            obj.getSumInAreaSeq();
            obj.estimateWalkSpeedSeq();
        end






        %% 单帧数据处理
        %数据预处理
        function preProcessData(obj)
            obj.mProdData  = obj.mRawData;
            obj.noiseFilter();
        end

        %求取第index帧数据向量
        function pVec = getFrameProcessed(obj,index)
            pVec = obj.mProdData.valueMat(index,:);%获取第index行压力数值向量
        end

        %求取第index帧原始数据
        function pressureVec = getFrameRaw(obj,index)
            pressureVec = obj.mRawData.valueMat(index,:);%获取第index行压力数值向量
        end

        % 求第i帧和
        function sum = getSum(obj,i)
            sum = 0;
            for point = obj.getFrameProcessed(i)
                sum = sum + point;
            end
        end
        % 求和序列 （时间序列的压力和序列）
        function sumSeq = getSumSeq(obj)
            sumSeq = zeros(obj.mSeqLength,1);
            for i = 1:obj.mSeqLength
                sumSeq(i) = obj.getSum(i);
            end
        end

        %求解第index帧的压力中心坐标
        function pos = getCOP(obj,index)
            pos = zeros(1,2);
            sumX = 0;
            sumY = 0;
            pressureVec = obj.getFrameProcessed(index);
            for i = 1:obj.mFrameSize
                sumX = sumX + pressureVec(i) * obj.mPosCoordMat(i,1);
                sumY = sumY + pressureVec(i) * obj.mPosCoordMat(i,2);
            end
            sumTotal = obj.getSum(index);
            if sumTotal ~= 0
                pos(1) = double(1/sumTotal*sumX);%X方向位置
                pos(2) = double(1/sumTotal*sumY);%y方向位置
            else
                pos(1) = 0;%X方向位置
                pos(2) = 0;%y方向位置
            end


        end
        %获取压力中心序列
        function pCenterSeq = getCOPSeq(obj)
            pCenterSeq = zeros(obj.mSeqLength,2);%预先分配内存,i行2列，第一列X，第二列Y
            for i = 1:obj.mSeqLength
                pos = obj.getCOP(i);
                pCenterSeq(i,1) = pos(1);
                pCenterSeq(i,2) = pos(2);
            end
        end


        % 求解COP速度序列
        function copSeq = getCOPVelSeq(obj)
            copSeq = zeros(obj.mSeqLength,2);%i行两列，第一列：X方向速度，第二列：y方向速度
            copSeq(1,:) = zeros(1,2);%初始值为零
            for i = 2:obj.mSeqLength
                t_Now = obj.mRawData.Timestamp(i);%当前时间
                t_Prev = obj.mRawData.Timestamp(i-1);%上一时刻时间
                delta_t = double(t_Now - t_Prev)*0.001;
                copSeq(i,:)= (obj.getCOP(i)-obj.getCOP(i-1))/delta_t;
            end
        end


        %噪声滤波器
        % （去除悬空时传感器感知到的压力数据,其因鞋底和传感器接触产生）
        function noiseFilter(obj)

            %             for i = 1:obj.mSeqLength
            %                 for j = 1:obj.mFrameSize
            %                     if obj.mRawData.valueMat(i,j) < 20%阈值
            %                         obj.mProdData.valueMat(i,j) = 0;%小于阈值认为是零
            %                     end
            %                 end
            %             end
        end


        %% 步态检测
        %获取单点步态
        function phase = getGaitPhase(obj,index)
            phase = obj.mGaitPhaseSeq(index);
        end

        %计算步态相位序列
        function getGaitPhaseSeqByCOPVel(obj)
            obj.mGaitPhaseSeq = zeros(obj.mSeqLength,1);%预分配内存
            [pks,HeelLocs] = findpeaks(obj.mCOPVelSeq(:,2),'Threshold',150);
            [pks,ToeLocs] = findpeaks(-obj.mCOPVelSeq(:,2),'Threshold',300);

            COPYVelSeq = obj.mCOPVelSeq(:,2);%获取COP速度序列

            %通过循环，将mGaitPhase中的指定索引改成检测到的相位
            for i = 1:size(ToeLocs,1)
                loc = ToeLocs(i);%获取峰值索引
                if loc+2<obj.mSeqLength%判断索引是否合理
                    if (COPYVelSeq(loc+1) == 0) && (COPYVelSeq(loc+2) == 0)
                        %如果峰值后面两位都为0
                        %确认为脚尖离地
                        obj.mGaitPhaseSeq(loc) = Utils.Phase_ToeOff;
                    end
                else
                    %索引为末尾，直接认为是脚尖离地
                    obj.mGaitPhaseSeq(loc) = Utils.Phase_ToeOff;
                end

            end
            for i = 1:size(HeelLocs,1)
                loc = HeelLocs(i);%获取峰值索引
                if loc-2>0 %判断索引是否合理
                    if  COPYVelSeq(loc-1) == 0 && COPYVelSeq(loc-2) == 0
                        %若前面两位都为0
                        obj.mGaitPhaseSeq(loc) = Utils.Phase_HeelStrike;
                    end
                else
                    obj.mGaitPhaseSeq(loc) = Utils.Phase_HeelStrike;
                end
            end

            lastPoint = Utils.Phase_Unknown;%默认前一时刻的相位为未知
            %为除峰值之外的点赋值
            %根据前一时刻进行赋值
            for i = 1:obj.mSeqLength
                if(obj.mGaitPhaseSeq(i) == Utils.Phase_HeelStrike)
                    lastPoint = Utils.Phase_HeelStrike;
                    %认为脚跟着地着地相
                    obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                elseif(obj.mGaitPhaseSeq(i) == Utils.Phase_ToeOff)
                    %认为脚尖离地即为离地相
                    lastPoint = Utils.Phase_ToeOff;
                    %前一时刻脚尖离地
                    obj.mGaitPhaseSeq(i) = Utils.Phase_Floating;
                else
                    %不是关键节点
                    if(lastPoint == Utils.Phase_Unknown)
                        %未知情况默认为着地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                    elseif(lastPoint == Utils.Phase_HeelStrike)
                        %前一时刻为脚跟着地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                    elseif(lastPoint == Utils.Phase_ToeOff)
                        %前一时刻脚尖离地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Floating;
                    end
                end
            end
        end

        %对数据进行处理
        function data = seqProcessToFindValley(obj,data)
            data = -data + 11;
        end


        % @brief 通过COP坐标获取步态
        function getGaitPhaseSeqByCOP(obj)
            obj.mGaitPhaseSeq = zeros(obj.mSeqLength,1);%预分配内存

            [pks,HeelLocs] = findpeaks(obj.seqProcessToFindValley(obj.mCOPSeq(:,2)));
            HeelLocs = HeelLocs(pks>obj.seqProcessToFindValley(5));
            [pks,ToeLocs] = findpeaks(obj.mCOPSeq(:,2));
            ToeLocs = ToeLocs(pks>7);
            %通过循环，将mGaitPhase中的指定索引改成检测到的相位
            for i = 1:size(ToeLocs,1)
                loc = ToeLocs(i);%获取峰值索引
                %确认为脚尖离地
                obj.mGaitPhaseSeq(loc) = Utils.Phase_ToeOff;
            end
            for i = 1:size(HeelLocs,1)
                loc = HeelLocs(i);%获取峰值索引
                obj.mGaitPhaseSeq(loc) = Utils.Phase_HeelStrike;
            end

            lastPoint = Utils.Phase_Unknown;%默认前一时刻的相位为未知

            %为除峰值之外的点赋值
            %根据前一时刻进行赋值
            for i = 1:obj.mSeqLength
                if(obj.mGaitPhaseSeq(i) == Utils.Phase_HeelStrike)
                    lastPoint = Utils.Phase_HeelStrike;
                    %认为脚跟着地着地相
                    obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                elseif(obj.mGaitPhaseSeq(i) == Utils.Phase_ToeOff)
                    %认为脚尖离地即为离地相
                    lastPoint = Utils.Phase_ToeOff;
                    %前一时刻脚尖离地
                    obj.mGaitPhaseSeq(i) = Utils.Phase_Floating;
                else
                    %不是关键节点
                    if(lastPoint == Utils.Phase_Unknown)
                        %未知情况默认为着地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                    elseif(lastPoint == Utils.Phase_HeelStrike)
                        %前一时刻为脚跟着地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Landing;
                    elseif(lastPoint == Utils.Phase_ToeOff)
                        %前一时刻脚尖离地
                        obj.mGaitPhaseSeq(i) = Utils.Phase_Floating;
                    end
                end
            end

        end


        % @brief 获取区域数据压力和
        function getSumInAreaSeq(obj)
            valueMat = obj.mRawData.valueMat;
            %提取T、M、C、L四个区域压力和序列
            %预先分配内存
            T_SumSeq = zeros(obj.mSeqLength,1);
            M_SumSeq = zeros(obj.mSeqLength,1);
            C_SumSeq = zeros(obj.mSeqLength,1);
            L_SumSeq = zeros(obj.mSeqLength,1);
            H_SumSeq = zeros(obj.mSeqLength,1);
            %指定区域索引向量
            T_AreaIndexVector = [40,41,44,45];
            M_AreaIndexVector = [29,30,35,36];
            C_AreaIndexVector = [25,26,31,32];
            L_AreaIndexVector = [27,28,33,34];
            H_AreaIndexVector = 1:1:11;

            for i = 1:obj.mSeqLength
                for j = 1:obj.mFrameSize
                    if(ismember(j,T_AreaIndexVector))
                        T_SumSeq(i) = T_SumSeq(i) + valueMat(i,j);
                    elseif(ismember(j,M_AreaIndexVector))
                        M_SumSeq(i) = M_SumSeq(i) + valueMat(i,j);
                    elseif(ismember(j,C_AreaIndexVector))
                        C_SumSeq(i) = C_SumSeq(i) + valueMat(i,j);
                    elseif(ismember(j,L_AreaIndexVector))
                        L_SumSeq(i) = L_SumSeq(i) + valueMat(i,j);
                    elseif(ismember(j,H_AreaIndexVector))
                        H_SumSeq(i) = H_SumSeq(i) + valueMat(i,j);
                    end
                end
            end

            % 使用 findpeaks 找到峰值
            [obj.mSumSeqInArea.T_peaks, obj.mSumSeqInArea.T_locations] = findpeaks(T_SumSeq);
            [obj.mSumSeqInArea.M_peaks, obj.mSumSeqInArea.M_locations] = findpeaks(M_SumSeq);
            [obj.mSumSeqInArea.C_peaks, obj.mSumSeqInArea.C_locations] = findpeaks(C_SumSeq);
            [obj.mSumSeqInArea.L_peaks, obj.mSumSeqInArea.L_locations] = findpeaks(L_SumSeq);
            [obj.mSumSeqInArea.H_peaks, obj.mSumSeqInArea.H_locations] = findpeaks(H_SumSeq);

            %找到峰值中的峰值
            [obj.mSumSeqInArea.T_Sec_peaks, obj.mSumSeqInArea.T_Sec_locations] = findpeaks(obj.mSumSeqInArea.T_peaks);


            %保存结果
            %序列数据
            obj.mSumSeqInArea.T_SumSeq = T_SumSeq;
            obj.mSumSeqInArea.M_SumSeq = M_SumSeq;
            obj.mSumSeqInArea.C_SumSeq = C_SumSeq;
            obj.mSumSeqInArea.L_SumSeq = L_SumSeq;
            obj.mSumSeqInArea.H_SumSeq = H_SumSeq;           
        end

        % @brief 步长估计
        function estimateWalkSpeedSeq(obj)
            obj.mWalkSpeedSeq = zeros(obj.mSeqLength,1);
            for i = 1:height(obj.mSumSeqInArea.T_Sec_locations) 
                speed = (0.07218 * obj.mSumSeqInArea.T_Sec_peaks(i) + -1.491)/3.6;%步行速度（m/s）
                obj.mWalkSpeedSeq(obj.mSumSeqInArea.T_locations(i)) = speed;
            end
        end



    end
end