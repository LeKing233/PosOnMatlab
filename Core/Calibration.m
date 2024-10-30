classdef Calibration < handle
    properties(Access = public)
        gyroBias = [0; 0; 0];
        magCalSeq;
        magCalCenter = [0; 0; 0];;
        magCalScale = [1; 1; 1];;
    end
    
    methods(Access = public)
        
        function obj = Calibration(wseq, mseq)
            if ~isempty(wseq)
                obj.gyroCalibration(wseq);
            end
            if ~isempty(mseq)
                obj.magCalibration(mseq);
            end
        end
        
        function bias = getGyroCali(obj)
            bias = obj.gyroBias;
        end
        
        function [center, scale_zoom] = getMagCali(obj)
            center = obj.magCalCenter;
            scale_zoom = obj.magCalScale;
        end
        
    end
    
    methods(Access = private)
        % 陀螺仪零偏校准
        function gyroCalibration(obj, wseq)
            obj.gyroBias = mean(wseq')';
        end
        
        % 椭球拟合
        % 输入 mseq 为 3*n 的矩阵，n 个点的三维坐标
        % 最小二乘法
        % a(1)x^2+a(2)y^2+a(3)z^2+a(4)xy+a(5)xz+a(6)yz+a(7)x+a(8)y+a(9)z=1
        % 输出 Center 为 3*1 的矩阵，椭球中心坐标，单位为uT，需要减去
        % Scale_zoom 为 3*1 的矩阵，三个轴的缩放比例，需要乘上
        % 内含大连默认的磁场强度53.5uT
        % 调用方式为[Center, Scale_zoom] = magCalibration(mseq)
        function magCalibration(obj, mseq)
            mseq = mseq';
            scale_dalian = 53.3;

            mean_x = mean(mseq(:,1));
            mean_y = mean(mseq(:,2));
            mean_z = mean(mseq(:,3));

            x = mseq(:,1) - mean_x;
            y = mseq(:,2) - mean_y;
            z = mseq(:,3) - mean_z;

            D = [x.*x y.*y z.*z x.*y x.*z y.*z x y z ];
            a = inv(D'*D)*D'*ones(size(x));
            M = [a(1) a(4)/2 a(5)/2;...
                a(4)/2 a(2) a(6)/2;...
                a(5)/2 a(6)/2 a(3)]; 
            obj.magCalCenter = -1/2*[a(7),a(8),a(9)]*inv(M);  
            SS = obj.magCalCenter * M * obj.magCalCenter'+ 1;
            [U,V] = eig(M);                       %Matlab计算特征值是大小顺序
            [~,n1] = max(abs(U(:,1)));            %输出的，但和xyz轴顺序不   
            [~,n2] = max(abs(U(:,2)));            %不同，这个操作就是让特征
            [~,n3] = max(abs(U(:,3)));            %值和xyz轴对应上。
            lambda(n1) = V(1,1);
            lambda(n2) = V(2,2);
            lambda(n3) = V(3,3);
            Scale_axis = [sqrt(SS/lambda(1)), sqrt(SS/lambda(2)), sqrt(SS/lambda(3))];
            obj.magCalScale = [scale_dalian/sqrt(SS/lambda(1)), scale_dalian/sqrt(SS/lambda(2)), scale_dalian/sqrt(SS/lambda(3))]';
            obj.magCalCenter = (obj.magCalCenter + [mean_x, mean_y, mean_z])';
            obj.magCalSeq = mseq;

        end
    end
end