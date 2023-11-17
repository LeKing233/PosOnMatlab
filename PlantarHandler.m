classdef PlantarHandler <handle
   

    properties
        mFilePath;
        mRawData;%原始数据      
        posCoordMat;%定义压力数据点位置
        xN;%X方向上网格数
        yN;%Y方向上网格数
    end

    methods
        function obj = PlantarHandler(filePath)
            obj.mFilePath = filePath;
            obj.init();         
            obj.extractData();
        end
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
            obj.mRawData.valueMat = valueMat;
        end

        function init(obj)
            %按照发送格式的45个点顺序，对左脚来说，以左下角为坐标原点，进行坐标录入
            %每行从左向右，从脚跟起向上计数
            obj.posCoordMat = [ 1.2 0.8;
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
            
            obj.xN = 40;%X方向上网格数
            obj.yN = 110;%Y方向上网格数
%             obj.mRawData = struct('TimeSinceCollection', [], 'Timestamp', [], 'valueMat', []);

        end


        function drawHeatMap(obj,index)
            xPosVec = obj.posCoordMat(:,1);%位置序列的第一列            
            xPosMax = max(xPosVec);
            xPosMin = min(xPosVec);
            yPosVec = obj.posCoordMat(:,2);%位置序列的第二列
            yPosMax = max(yPosVec);
            yPosMin = min(yPosVec);
            % 网格化x,y二维空间
            [X,Y] = meshgrid(linspace(xPosMin,xPosMax,obj.xN),linspace(yPosMin,yPosMax,obj.yN));
            figure("Name","单帧足底压力热力图",'Position', [100, 100, 500, 300])
            valueVec = obj.mRawData.valueMat(index,:);%获取第index行数值作为高度值
            % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
            Z = griddata(xPosVec,yPosVec,valueVec,X,Y,'natural');
            % 等高线法
            contourf(X,Y,Z,N, 'LineColor','none');
            colormap('jet');%设置颜色映射方案            
            colorbar;%启用颜色控制条
            %设置坐标范围
            xlim([min(X(:)) max(X(:))]);
            ylim([min(Y(:)) max(Y(:))]);
            axis equal;
            xlabel('X','FontSize', 16); % x轴注解
            ylabel('Y','FontSize', 16); % y轴注解
            timestamp = obj.mRawData.Timestamp(index);
            title(['这是第', num2str(timestamp*12/1000), '秒的数据'],'FontSize', 16); % 图形标题
        end

        function drawHeatMapInSection(obj,startIndex,endIndex,interval)
            xPosVec = obj.posCoordMat(:,1);%位置序列的第一列            
            xPosMax = max(xPosVec);
            xPosMin = min(xPosVec);
            yPosVec = obj.posCoordMat(:,2);%位置序列的第二列
            yPosMax = max(yPosVec);
            yPosMin = min(yPosVec);
            if(startIndex<0||startIndex>size(xPosVec,1)||endIndex<0||endIndex>size(xPosVec,1))
                disp("绘制热力图超过区间范围")
            end
            % 网格化x,y二维空间
            [X,Y] = meshgrid(linspace(xPosMin,xPosMax,obj.xN),linspace(yPosMin,yPosMax,obj.yN));
            figure("Name","单帧足底压力热力图",'Position', [100, 100, 500, 300])
            for i = startIndex:endIndex
                reminder = mod(i,interval);
                %如果余数不为0，返回
                if reminder ~= 0
                    continue;
                end
                %执行
                valueVec = obj.mRawData.valueMat(index,:);%获取第index行数值作为高度值
                % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
                Z = griddata(xPosVec,yPosVec,valueVec,X,Y,'natural');
                % 等高线法
                contourf(X,Y,Z,N, 'LineColor','none');
                colormap('jet');%设置颜色映射方案            
                colorbar;%启用颜色控制条
                %设置坐标范围
                xlim([min(X(:)) max(X(:))]);
                ylim([min(Y(:)) max(Y(:))]);
                axis equal;
                xlabel('X','FontSize', 16); % x轴注解
                ylabel('Y','FontSize', 16); % y轴注解
                timestamp = obj.mRawData.Timestamp(i);
                title(['这是第', num2str(timestamp*12/1000), '秒的数据'],'FontSize', 16); % 图形标题
            end

        end
  
    end
end