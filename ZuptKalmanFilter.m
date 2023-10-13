classdef ZuptKalmanFilter <handle
    properties(Access = public)
        %变量
        X;    %state
        K;    %kalman gain
        P;    %covariance 
        Q;    %system noise variance 
        R;    %observation noise variance 
        H;    %observation Matrix
        %参数，用于构建系统噪声矩阵Q和观测噪声矩阵R
        noise_w = 0.01 * ones(3,1);     %陀螺仪噪声
        noise_f = 375  * ones(3,1);     %加速度计噪声
        noise_r = 200  * ones(3,1);     %系统噪声（速度噪声）  
    end

    methods(Access = public)      
        %构造函数    
        function obj = ZuptKalmanFilter()
            %状态量初始化
           obj.init();
            obj.Q = diag([obj.noise_w(1)^2,obj.noise_w(2)^2,obj.noise_w(3)^2,...
                          obj.noise_f(1)^2,obj.noise_f(2)^2,obj.noise_f(3)^2]);
            obj.R = diag([obj.noise_r(1)^2,obj.noise_r(2)^2,obj.noise_r(3)^2]);
            obj.H = [zeros(3,3),eye(3,3),zeros(3,3)];
            
        end

        %初始化
        function init(obj)
            obj.X = zeros(9,1);
            obj.K = zeros(9,3);
            obj.P = eye(9);
        end
        
        %预测函数
        function predict(obj,F,G)
            obj.P = F*obj.P*F' + G*obj.Q*G';            
        end
        
        %更新函数
        function update(obj,Z)
            I = eye(9);
            obj.K = obj.P*obj.H'/(obj.H * obj.P * obj.H' + obj.R);
            obj.X = obj.K * Z;
            obj.P = (I - obj.K*obj.H)*obj.P*(I - obj.K*obj.H)' + obj.K*obj.R*obj.K';
        end      
    end
end