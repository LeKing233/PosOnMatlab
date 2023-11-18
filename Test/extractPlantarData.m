function data = extractPlantarData(filename)
    rawDataTable = readtable(filename, 'Format', '%d%d%s', 'Delimiter', ',');

    % 提取压力数据字符串
    pressureDataStr = rawDataTable.PlantarValue;
    
    % 初始化电压向量
    v = zeros(height(rawDataTable), 45);
    
    % 将压力数据字符串转换为向量
    for i = 1:height(rawDataTable)
        v(i,:) = str2double(split(pressureDataStr{i}, ';'));
    end

    %数据保存
    data.TimeSinceCollection = rawDataTable.TimeSinceCollection;
    data.Timestamp = rawDataTable.Timestamp;
    data.v = v;




    %定义压力数据点位置
    %按照发送格式的45个点顺序，进行坐标录入
    %按照左脚，每行从左向右，从脚跟起向上计数
    posInSeq = [1.2 0.8;
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

    % 定义点(x,y,z)
    xPos = posInSeq(:,1);%位置序列的第一列
    xPosMax = max(xPos);
    xPosMin = min(xPos);
    yPos = posInSeq(:,2);%位置序列的第二列
    yPosMax = max(yPos);
    yPosMin = min(yPos);
    N = 100; % 每个维度的数据点数
    % 网格化x,y二维空间
    [X,Y] = meshgrid(linspace(xPosMin,xPosMax,N),linspace(yPosMin,yPosMax,N));
%     a =624.1 ;
%     b =0.6779;
%     c =0.2652;

    
    figure(1);
    
    for i = 1:size(v,1)
        reminder = mod(i,2);
        if reminder == 1
            vout = v(i,:);
%             r = (3630./vout-100)/1000;%千欧
%             pho = 1./r;
%             pressureValue = a*exp(-((pho-b)./c).^2);%
%             z = pressureValue;
%             data.pressure = pressureValue;
        
            z = vout;
           
            % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
            Z = griddata(xPos,yPos,z,X,Y,'natural');
            % 因为图像坐标和笛卡尔坐标起始位置不一样，需要上下翻转
%             contourf(X,Y,Z,N, 'LineColor','none');
            % 因为图像坐标和笛卡尔坐标起始位置不一样，需要上下翻转
            imagesc(flipud(Z));
            colormap('jet');
            colorbar;
            xlim([min(X(:)) max(X(:))]);
            ylim([min(Y(:)) max(Y(:))]);
            axis equal;
            xlabel('X','FontSize', 16); % x轴注解
            ylabel('Y','FontSize', 16); % y轴注解
            title(['这是第', num2str(i*12/1000), '秒的数据'],'FontSize', 16); % 图形标题
            drawnow; % 强制立即更新图形
        end
        
    end
%     vout = v(1,:);
%     r = (3630./vout-100)/1000;%千欧
%     pho = 1./r;
%     pressureValue = a*exp(-((pho-b)./c).^2);%
%     z = pressureValue;
%     data.pressure = pressureValue;
% 
%     
%     N = 500; % 每个维度的数据点数
%     % 网格化x,y二维空间
%     [X,Y] = meshgrid(linspace(xPosMin,xPosMax,N),linspace(yPosMin,yPosMax,N));
%     % 采用插值法扩展数据，可用方法有'linear'(default)|'nearest'|'natural'|'cubic'|'v4'|
%     Z = griddata(xPos,yPos,z,X,Y,'v4');
%     % 等高线法
%     figure('NumberTitle','off','Name','等高线法','Color','w','MenuBar','none','ToolBar','none');
%     contourf(X,Y,Z,N, 'LineColor','none');
%     xlim([min(X(:)) max(X(:))]);
%     ylim([min(Y(:)) max(Y(:))]);
%     colormap('jet');
%     colorbar;
%     axis off;
% 
%     figure(2)
%     plot(pressureValue,pho);
end

% 
%     a =624.1 ;
%     b =0.6779;
%     c =0.2652;
%     x3 = 0:0.01:0.5;%x 电导
%     fun3 = a*exp(-((x3-b)./c).^2);%N