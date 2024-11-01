classdef GraphCalculator
    %GRAPHCALCULATOR 绘图计算器
    %   计算绘图有关的数据

    properties

    end

    methods(Static)
        % @brief 获取直线计算结果
        % @param prodArray——proposed计算的StateCalculator数组，
        %        tradArray——traditional方法计算的StateCalculator数组
        % @retval 生成四组数据————proposed和traditional方法的ABCD四个点的相对误差、绝对误差
        function resMat = getLineErrorResult(prodArray,tradArray)
            if(length(prodArray)~= length(tradArray))
                error("两种算法长度不一致");
            end
            x_ref  = [15,30,45,60];% 四个参考点的坐标
            arrayLen = length(prodArray);
            prodAE_Mat = zeros(4,arrayLen);%prod误差数据矩阵
            tradAE_Mat = zeros(4,arrayLen);%trad

            %计算误差数据
            for j =  1:arrayLen
                %计算proposed
                P_prod_adjust = TrackAdjuster.rotateByMean(prodArray(j).mStateSeq.P');
                x_prod = P_prod_adjust(1,:);
                y_prod = abs(P_prod_adjust(2,:));
                x_ref = min(max(x_ref, min(x_prod)), max(x_prod));
                prodAE_Mat(:,j) =  interp1(x_prod, y_prod, x_ref, 'linear', 'extrap');
            
                %计算traditional
                P_trad_adjust = TrackAdjuster.rotateByMean(tradArray(j).mStateSeq.P');
                x_trad = P_trad_adjust(1,:);
                y_trad = P_trad_adjust(2,:);
                x_ref = min(max(x_ref, min(x_trad)), max(x_trad));
                tradAE_Mat(:,j) =  interp1(x_trad, y_trad, x_ref, 'linear', 'extrap');
            end

            assignin("base",'prodAE_Mat',prodAE_Mat);
            assignin("base",'tradAE_Mat', tradAE_Mat);
            assignin("base",'P_prod_adjust', tradAE_Mat);

            %最终的结果表格
            % ---------------------------------------------------
            % | \ | pAE | pAE_Er | tAE | tAE_Er | pRE | pRE_Er | tRE | tRE_Er |
            % ---------------------------------------------------
            % | A |     |        |     |        |     |        |     |        |
            % ---------------------------------------------------
            % | B |     |        |     |        |     |        |     |        |
            % ---------------------------------------------------
            % | C |     |        |     |        |     |        |     |        |
            % ---------------------------------------------------
            % | D |     |        |     |        |     |        |     |        |
            % ---------------------------------------------------
            resMat = zeros(4,8);
            lineLen = 60;%直线总长度
            for i = 1:4
                [pAE,pAE_Er] = GraphCalculator.getMeanAndCI(prodAE_Mat(i,:));
                [tAE,tAE_Er] = GraphCalculator.getMeanAndCI(tradAE_Mat(i,:));
                [pRE,pRE_Er] = GraphCalculator.getMeanAndCI(prodAE_Mat(i,:)/lineLen);
                [tRE,tRE_Er] = GraphCalculator.getMeanAndCI(tradAE_Mat(i,:)/lineLen);
                resMat(i,:) = [pAE,pAE_Er,tAE,tAE_Er,pRE,pRE_Er,tRE,tRE_Er];
            end
            
            



        end


       % @brief 计算数据的均值（mean）和95%置信区间的绝对值（Confidence Interval）
       % @param y 数据
       % @retval [mean_y, ci_error] mean_y——y均值, ci_error——95%置信区间的绝对值
       function [mean_y, ci_error] = getMeanAndCI(y)
           % 计算均值
           mean_y = mean(y)
           % 计算标准误差
           y
           stdres =  std(y)
           SE_y = std(y) / sqrt(length(y))
           % 计算误差带（95%置信区间）
           ci_error = 1.96 * SE_y;  % 95%置信区间的误差绝对值
       end







        % @brief 计算误差带图所需要的数据
        % @param x——未均一化的数据，y数据
        % @retval [unique_x,mean_y,error_y] unique_x——归并化的x数据,mean_y——y均值,error_y——95%置信度误差绝对值
        function [unique_x,mean_y,error_y] = getErrorBoundDataAndUniqueX(x,y)
            % 获取每个唯一的 x 值
            unique_x = unique(x);

            % 初始化均值、标准误差、误差带上下限,误差绝对值
            mean_y = zeros(size(unique_x));
            SE_y = zeros(size(unique_x));
            upper_bound = zeros(size(unique_x));
            lower_bound = zeros(size(unique_x));
            error_y = zeros(size(unique_x));

            % 计算每个 x 值对应的 y 的均值和标准误差
            for i = 1:length(unique_x)
                % 找到当前 x 值对应的所有 y 值
                y_values_cell = y(x == unique_x(i));
                y_values = cell2mat(y_values_cell(:)');



                % 计算均值和标准误差
                mean_y(i) = mean(y_values);
                SE_y(i) = std(y_values) / sqrt(length(y_values));

                % 计算误差带（95% 置信区间）
                error_y(i) = 1.96 * SE_y(i);
                upper_bound(i) = mean_y(i) + error_y(i);
                lower_bound(i) = mean_y(i) - error_y(i);

            end
        end













    end
end

