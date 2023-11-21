classdef PlantarHandler <handle
   

    properties(Access = public)
        mFilePath;%文件路径
        mRawData;%原始数据      
        mProdData;%处理过数据
        mPosCoordMat;%定义压力数据点位置
        mFrameSize;%帧长度
        mSeqLength;%序列长度
        mNx;%X方向上网格数
        mNy;%Y方向上网格数
        mFrameInterval;%帧间隔时间（毫秒）
        mHeatMapGcf;%热力图figure句柄
    end
    
    %公共方法
    methods(Access = public)
        %% 初始化
        %构造函数
        function obj = PlantarHandler(filePath)
            obj.mFilePath = filePath;                    
            obj.extractData();
            obj.init(); 
        end

        % 提取数据
        function extractData(obj)
            rawDataTable = readtable(obj.mFilePath, 'Format', '%d%d%s', 'Delimiter', ',');
            % 提取压力数据字符串
            str = rawDataTable.PlantarValue;
            % 初始化压力数据矩阵（i行，45列）
            valueMat = zeros(height(rawDataTable), 45);
            % 将压力数据字符串转换为向量，并存入矩阵
            for i = 1:height(rawDataTable)
                valueMat(i,:) = str2double(split(str{i}, ';'));
            end
            obj.mRawData.TimeSinceCollection = rawDataTable.TimeSinceCollection;
            obj.mRawData.Timestamp = rawDataTable.Timestamp;
            obj.mRawData.valueMat = valueMat;%i行 45列
            
        
        end
        
        %从压力数据转换成电压数据

        %初始化
        function init(obj)
            %按照发送格式的45个点顺序，对左脚来说，以左下角为坐标原点，进行坐标录入
            %每行从左向右，从脚跟起向上计数
            obj.mPosCoordMat = [ 1.2 0.8;
                                1.7 0.6;
                                2.3 0.8;%第一行
                                0.9 1.8;
                                1.5 1.8;
                                2.0 1.8;
                                2.6 1.8;%第二行
                                0.9 2.8;
                                1.5 2.8;
                                2.0 2.8;
                                2.6 2.8;%第三行
                                0.9 3.9;
                                1.5 3.9;
                                2.0 3.9;
                                2.6 3.9;%第四行
                                0.7 5.0;
                                1.4 5.0;
                                2.0 5.0;
                                2.6 5.0;%第五行
                                0.5 6.2;
                                1.1 6.2;
                                1.7 6.2;
                                2.2 6.2;
                                2.8 6.2;%第六行
                                0.4 7.3;
                                1.0 7.3;
                                1.5 7.3;
                                2.0 7.3;
                                2.5 7.3;
                                3.0 7.3;%第七行
                                0.6 8.4;
                                1.1 8.4;
                                1.7 8.4;
                                2.2 8.4;
                                2.8 8.4;
                                3.3 8.4;%第八行
                                1.1 9.5;
                                1.7 9.5;
                                2.2 9.5;
                                2.8 9.5;
                                3.3 9.5;%第九行
                                1.5 10.4;
                                2.0 10.5;
                                2.5 10.5;
                                3.2 10.4];%第十行
            
            obj.mNx = 400;%X方向上网格数
            obj.mNy = 1100;%Y方向上网格数
            obj.mSeqLength = size(obj.mRawData.valueMat,1);%行数作为序列长度
            obj.mFrameSize = size(obj.mPosCoordMat,1);%单帧压力点数
            %序列间隔时间，取平均值得到
            obj.mFrameInterval = double((obj.mRawData.TimeSinceCollection(end)-obj.mRawData.TimeSinceCollection(1))/obj.mSeqLength);
            
            obj.preProcessData();%对数据进行预处理

        end
        %% 单帧数据处理

        %数据预处理
        function preProcessData(obj)
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
            sumSeq = zeros(obj.mSeqLength);
            for i = 1:obj.mSeqLength
                sumSeq(i) = obj.getSum(i);
            end
        end     

        %求解第index帧的压力中心坐标
        function pos = getPressureCenter(obj,index)
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
        function pCenterSeq = getPressureCenterSeq(obj)
            pCenterSeq = zeros(obj.mSeqLength,2);%预先分配内存,i行2列，第一列X，第二列Y
            for i = 1:obj.mSeqLength
                pos = obj.getPressureCenter(i);
                pCenterSeq(i,1) = pos(1);
                pCenterSeq(i,2) = pos(2);
            end
        end
        

        % 求解COP速度序列
        function copSeq = getCOPSeq(obj)
            copSeq = zeros(obj.mSeqLength,2);%i行两列，第一列：X方向速度，第二列：y方向速度
            copSeq(1,:) = zeros(1,2);%初始值为零
            for i = 2:obj.mSeqLength
                t_Now = obj.mRawData.TimeSinceCollection(i);%当前时间
                t_Prev = obj.mRawData.TimeSinceCollection(i-1);%上一时刻时间
                delta_t = double(t_Now - t_Prev)*0.001;                
                copSeq(i,:)= (obj.getPressureCenter(i)-obj.getPressureCenter(i-1))/delta_t;
            end
        end


        %噪声滤波器
        % （去除悬空时传感器感知到的压力数据,其因鞋底和传感器接触产生）
        function noiseFilter(obj)
            for i = 1:obj.mSeqLength           
                for j = 1:obj.mFrameSize
                    if obj.mRawData.valueMat(i,j) < 2%阈值
                        obj.mProdData.valueMat(i,j) = 0;%小于阈值认为是零
                    else
                        obj.mProdData.valueMat(i,j) = obj.mRawData.valueMat(i,j);%大于阈值不做处理
                    end                    
                end
            end            
        end


        %% 绘图函数
        %绘制单帧热力图
        function gcf = drawHeatMapOfIndex(obj,index)
           gcf = obj.drawHeatMapOfSection(index,index,1);
        end

        %持续绘制所有时间热力图 interval——绘图间隔（帧）
        function gcf = drawHeatMapOfAll(obj,interval)
            gcf = obj.drawHeatMapOfSection(1,size(obj.mRawData.valueMat,1),interval);
        end
        
        %持续绘制并保存成MP4
        function drawHeatMapOfAllSaveToMp4(obj,interval)           
            % 获取当前的系统时间
            currentTime = datetime('now');            
            % 将当前时间转换为字符串，只包含月份、日期和时间
            strTime = datestr(currentTime, 'mm-dd_HH-MM-SS'); %#ok<*DATST> 
            fileName = ['../result/HeatMap_OfInterval_',num2str(interval),'_',strTime,'.mp4'];
            % 创建一个 VideoWriter 对象，指定 'Profile' 为 'MPEG-4'         
            v = VideoWriter(fileName, 'MPEG-4');
            %希望播放速度与真实速度一致，调整帧率
            v.FrameRate = 1000/(obj.mFrameInterval*interval);
            % 打开视频文件进行写入
            open(v);
            for i = 1:size(obj.mRawData.valueMat,1)
                 gcf = obj.drawHeatMapOfIndex(i);
                 % 捕获此图形为帧
                 frame = getframe(gcf); 
                 % 将帧写入视频
                 writeVideo(v, frame);
            end
            % 关闭视频文件
            close(v);
        end


         %持续绘制介于两个点的热力图 interval——绘图间隔（帧）
        function gcf = drawHeatMapOfSection(obj,startIndex,endIndex,interval)
            xPosVec = obj.mPosCoordMat(:,1);%位置序列的第一列            
            xPosMax = max(xPosVec);
            xPosMin = min(xPosVec);
            yPosVec = obj.mPosCoordMat(:,2);%位置序列的第二列
            yPosMax = max(yPosVec);
            yPosMin = min(yPosVec);
            if(startIndex<0||startIndex>size(obj.mRawData.valueMat,1)||endIndex<0||endIndex>size(obj.mRawData.valueMat,1))
                disp("绘制热力图超过区间范围")
            end
            % 网格化x,y二维空间
            [X,Y] = meshgrid(linspace(xPosMin,xPosMax,obj.mNx),linspace(yPosMin,yPosMax,obj.mNy));
            %绘图配置
            %窗体配置
            if  isprop(obj, 'mHeatMapGcf') && isempty(obj.mHeatMapGcf) %存在且为空
                %如果未创建，创建一个新视图
                obj.mHeatMapGcf = figure("Name","动态足底压力热力图",'Position', [200,100, 1000, 920]);
            end
                  
%             xlim([min(X(:)) max(X(:))]);%设置x坐标范围
%             ylim([min(Y(:)) max(Y(:))]);
            %逐帧更新数据
            for i = startIndex:endIndex
                reminder = mod(i,interval);
                %如果余数不为0，返回
                if reminder ~= 0
                    continue;
                end
                %执行
                frameVec = obj.getFrameProcessed(i);%获取第i帧数据
                % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
                Z = griddata(xPosVec,yPosVec,frameVec,X,Y,'natural');
                figure(obj.mHeatMapGcf)%找到指定图窗
                % 等高线法
                contourf(X,Y,Z,obj.mNy, 'LineColor','none');               
                TimeSinceCollection = double(obj.mRawData.TimeSinceCollection(i));%获取秒数
                title(['这是第', num2str(TimeSinceCollection/1000, '%.3f'), '秒的数据'],'FontSize', 16); % 图形标题           
                colorbar;%启用颜色控制条 
                c = colorbar;
                c.Label.String = '压力数值/N';
                % 设置colorbar的默认范围从0开始
               maxValue = max(frameVec(:));
                if maxValue <= 0
                    maxValue = 1;  % 或者您选择的任何大于0的值
                    set(gca, 'CLim', [0, maxValue]);
                end                
                colormap('jet');%设置颜色映射方案
                axis tight manual;%设置坐标轴范围紧密包裹数据
                axis equal;
                xlabel('X/cm','FontSize', 16); % x轴注解
                ylabel('Y/cm','FontSize', 16); % y轴注解
                drawnow; % 强制立即更新图形
            end
            gcf = obj.mHeatMapGcf;%结果赋值
        end





        




        %绘制足底压力和时序图
        function plotSumSeq(obj)
            gcf = figure("Name",'足底压力和时序图');
            sumSeq = obj.getSumSeq();
            plot(obj.mRawData.Timestamp,sumSeq);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('压力和/N'); % y轴注解
            title('足底压力时序图'); % 图形标题
            grid on; % 显示格线
        end

        %绘制COP速度图
        function plotCOPSeq(obj)
            gcf = figure("Name","COP速度");
            copSeq = obj.getCOPSeq();
            plot(obj.mRawData.Timestamp,copSeq(:,1));hold on;
            plot(obj.mRawData.Timestamp,copSeq(:,2));
            xlabel('时间戳/ms'); % x轴注解
            ylabel('速度/cm/s'); % y轴注解
            title('COP速度图'); % 图形标题
            legend("X方向","Y方向");
            grid on; % 显示格线
        end

         %COP速度模长
        function plotCOPNorm(obj)
            gcf = figure("Name","COP速度模长");
            copSeq = obj.getCOPSeq();
            copNormVec = zeros(size(copSeq,1),1);%初始化向量
            for i = 1:size(copNormVec,1)
                copNormVec(i) = norm(copSeq(i,:));
            end
            plot(obj.mRawData.Timestamp,copNormVec);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('速度/cm/s'); % y轴注解
            title('COP速度模长图'); % 图形标题
            grid on; % 显示格线
        end
        
        %压力中心坐标
        function plotPressureCenterSeq(obj)
            gcf = figure("Name","压力中心坐标");
            pressureCenterSeq = obj.getPressureCenterSeq();
             plot(obj.mRawData.Timestamp,pressureCenterSeq(:,1));hold on;
             plot(obj.mRawData.Timestamp,pressureCenterSeq(:,2));
             xlabel('时间戳/ms'); % x轴注解
            ylabel('坐标/cm'); % y轴注解
            title('压力中心坐标图'); % 图形标题
            legend("X方向","Y方向");
            grid on; % 显示格线
        end
        
        
        

    end
end