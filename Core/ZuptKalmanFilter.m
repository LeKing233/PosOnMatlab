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
        noise_w;        %预测噪声（陀螺仪噪声）
        noise_f;        %预测噪声（加速度计噪声）
        noise_g;        %观测噪声（互补滤波重力加速度噪声）
        noise_m;        %观测噪声（磁力计噪声）
        noise_v;        %观测噪声（速度噪声）
        noise_h;        %观测噪声（高度计噪声）
        factor_s;       %渐消记忆因子
        
        %算法设置
        ob_Attitude;    %姿态修正设置
        ob_Position;    %位移修正设置
    end

    methods(Access = public)
        
        % @brief ZuptKalmanFilter构造函数 
        % @param varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = ZuptKalmanFilter(varargin)
            %读取设置
            Kalman_setting = inputParser;
            addParameter(Kalman_setting, 'Attitude', Utils.WHOLE_ATTITUDE);     %默认启用三轴姿态修正
            addParameter(Kalman_setting, 'Position', Utils.VERTICAL_POSITION);  %默认启用高度修正
            addParameter(Kalman_setting, 'noiseW', 0.001);                      %预测噪声（陀螺仪噪声）
            addParameter(Kalman_setting, 'noiseF', 600);                        %预测噪声（加速度计噪声）
            addParameter(Kalman_setting, 'noiseG', 120);                        %观测噪声噪声（互补滤波重力加速度噪声）
            addParameter(Kalman_setting, 'noiseM', 240);                        %系统噪声（磁力计噪声）
            addParameter(Kalman_setting, 'noiseV', 400);                        %系统噪声（速度噪声）
            addParameter(Kalman_setting, 'noiseH', 60);                         %系统噪声（高度计计噪声）
            addParameter(Kalman_setting, 'factorS', 1.00005);                   %渐消记忆因子
            parse(Kalman_setting, varargin{:});
            obj.ob_Attitude = Kalman_setting.Results.Attitude;
            obj.ob_Position = Kalman_setting.Results.Position;
            obj.noise_w = Kalman_setting.Results.noiseW * ones(3,1);
            obj.noise_f = Kalman_setting.Results.noiseF * ones(3,1);
            obj.noise_g = Kalman_setting.Results.noiseG * ones(2,1);
            obj.noise_m = Kalman_setting.Results.noiseM * ones(1,1);
            obj.noise_v = Kalman_setting.Results.noiseV * ones(3,1);
            obj.noise_h = Kalman_setting.Results.noiseH * ones(1,1);
            obj.factor_s = Kalman_setting.Results.factorS;
            
            %状态量初始化
            obj.init();
        end

        % @brief 卡尔曼滤波器变量初始化初始化 
        % @param None
        % @retval None
        function init(obj)
            dimension = 3;
            obj.Q = diag([obj.noise_w(1)^2,obj.noise_w(2)^2,obj.noise_w(3)^2,...
                          obj.noise_f(1)^2,obj.noise_f(2)^2,obj.noise_f(3)^2]);
            R_ele = [obj.noise_v(1)^2,obj.noise_v(2)^2,obj.noise_v(3)^2];
            obj.H = [zeros(3,3),eye(3,3),zeros(3,3)];
            %初始化R和H矩阵（根据不同的算法初始化不同的H和R）
            if obj.ob_Attitude == Utils.WHOLE_ATTITUDE
                R_ele = [obj.noise_g(1)^2, obj.noise_g(1)^2, obj.noise_m^2, R_ele];
                obj.H = [eye(3,3), zeros(3,6); obj.H];
                dimension = dimension+3;
            elseif obj.ob_Attitude == Utils.YAW_ATTITUDE
                R_ele = [obj.noise_m^2, R_ele];
                obj.H = [0, 0, 1, zeros(1,6); obj.H];
                dimension = dimension+1;
            end
            if obj.ob_Position == Utils.VERTICAL_POSITION
                R_ele = [R_ele, obj.noise_h^2];
                obj.H = [obj.H; zeros(1,6), 0, 0, 1];
                dimension = dimension+1;
            end
            obj.R = diag(R_ele);
            obj.X = zeros(9,1);
            obj.K = zeros(9,dimension);
            obj.P = 0.5*eye(9);
            %打印卡尔曼滤波器变量
%             disp('预测噪声协方差矩阵Q:'); %打印预测噪声协方差矩阵Q
%             disp(obj.Q);
%             disp('观测噪声协方差矩阵R:'); %打印观测噪声协方差矩阵R
%             disp(obj.R);
%             disp('观测矩阵H:'); %打印观测矩阵H
%             disp(obj.H);
%             disp('误差估计矩阵X:'); %打印误差估计矩阵X
%             disp(obj.X);
%             disp('卡尔曼增益K:'); %打印卡尔曼增益K
%             disp(obj.K);
%             disp('状态估计协方差矩阵P:'); %打印状态估计协方差矩阵P
%             disp(obj.P);
        end
        
        % @brief 卡尔曼预测函数
        % @param F 状态转移矩阵   G 噪声输入矩阵
        % @retval None
        function predict(obj,F,G)
            obj.P = F*(obj.factor_s*obj.P)*F' + G*obj.Q*G';            
        end
        
        % @brief 卡尔曼更新函数
        % @param Z 观测矩阵
        % @retval None
        function update(obj,Z)
            I = eye(9);
            obj.K = obj.P*obj.H'/(obj.H * obj.P * obj.H' + obj.R);
            obj.X = obj.K * Z;
            obj.P = (I - obj.K*obj.H)*obj.P*(I - obj.K*obj.H)' + obj.K*obj.R*obj.K';
        end 
    
        % @brief 获取状态转移矩阵F
        % @param Cbn 旋转矩阵   fn 加速度向量
        % @retval None
        function F = getF(~,obj2,Cbn,fn)            
            I = eye(3);
            ZeroMat = zeros(3,3);
            F = [I,ZeroMat,ZeroMat;
                 (Utils.getCrossMatrix(Cbn*fn))*obj2.delta_t,I,ZeroMat;
                 ZeroMat,I*obj2.delta_t,I];
        end
        
        % @brief 获取误差分配矩阵G
        % @param Cbn 旋转矩阵
        % @retval None
        function G = getG(~,obj2,Cbn)
            ZeroMat = zeros(3,3);
            G = [-Cbn*obj2.delta_t,ZeroMat;
                 ZeroMat,Cbn*obj2.delta_t;
                 ZeroMat,ZeroMat];
        end
            
        % @brief 速度更新函数
        % @param vel_error 速度误差
        % @retval None
        function reviseVn(~,obj2,vel_error)
            obj2.V = obj2.V + vel_error;
        end 

        % @brief 位移更新函数
        % @param pos_error 位移误差
        % @retval None
        function revisePn(~,obj2,pos_error)
            obj2.P = obj2.P + pos_error;
        end
       
        % @brief 姿态更新函数
        % @param phi_error 姿态误差
        % @retval None
        function reviseCbn(~,obj2,phi_error) 
            I = eye(3);
            phi_error_x = Utils.getCrossMatrix(phi_error)*180/pi;
            obj2.Cbn =  ((2*I + phi_error_x)\(2*I - phi_error_x)) * obj2.Cbn;
        end
    end
end