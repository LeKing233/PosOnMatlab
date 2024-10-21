classdef EKFAHRS <handle
     properties(Access = private)
        %扩展卡尔曼滤波变量
        X_pri;          %X先验估计
        X_pos           %X后验估计
        K;              %卡尔曼增益
        P_pri;          %先验估计误差协方差矩阵
        P_pos;          %后验估计误差协方差矩阵
        Q;              %预测噪声协方差矩阵 
        R;              %观测噪声协方差矩阵
        M;              %观测修正矩阵（不修正航向角）
        A;              %渐消记忆因子矩阵
        %航向修正卡尔曼滤波变量
        X_yaw;          %[yaw，wb]
        K_yaw;          %卡尔曼增益
        P_yaw;          %误差协方差矩阵
        Q_yaw;          %预测噪声协方差矩阵
        R_mag;          %观测修正矩阵
        %参数，用于构建系统噪声矩阵Q和观测噪声矩阵R
        noise_w;        %陀螺仪噪声
        noise_wb;       %陀螺仪偏置噪声
        noise_f;        %加速度计噪声
        noise_m;        %磁力计噪声
        %姿态参考系统参数
        q;              %姿态四元数表示
        phi;            %姿态欧拉角表示
        Delta_t;        %采集间隔
        
     end
     
      methods(Access = public)
          
        % @brief EKFAHRS 构造函数 
        % @param q 初始化姿态四元数表示(列向量)    varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = EKFAHRS(q, varargin)
            %读取设置
            EKF_setting = inputParser;
            addParameter(EKF_setting, 'SamplePeriod', 0.006);   %采集间隔
            addParameter(EKF_setting, 'noiseW',0.02);           %陀螺仪噪声
            addParameter(EKF_setting, 'noiseWb',0.0001);        %陀螺仪偏置噪声
            addParameter(EKF_setting, 'noiseF',40);             %加速度计噪声
            addParameter(EKF_setting, 'noiseM',50);             %磁力计噪声
            parse(EKF_setting, varargin{:});
            obj.Delta_t = EKF_setting.Results.SamplePeriod;
            obj.noise_w = EKF_setting.Results.noiseW * ones(4,1);
            obj.noise_wb = EKF_setting.Results.noiseWb * ones(3,1);
            obj.noise_f = EKF_setting.Results.noiseF * ones(3,1);
            obj.noise_m = EKF_setting.Results.noiseM * ones(1,1); 
            %状态量初始化
            obj.init(q);
        end
        
        % @brief 姿态参考系统变量初始化初始化 
        % @param obj当前实例  q 初始化姿态四元数表示(列向量)
        % @retval None
        function init(obj, q)
            %更新姿态
            obj.q = q;
            obj.phi = Utils.getPhiFromQ(obj.q);
            %初始化噪声协方差矩阵
            obj.Q = diag([obj.noise_w(1)^2,obj.noise_w(2)^2,obj.noise_w(3)^2,obj.noise_w(4)^2,...
                          obj.noise_wb(1)^2,obj.noise_wb(2)^2]);
            obj.R = diag([obj.noise_f(1)^2,obj.noise_f(2)^2,obj.noise_f(3)^2]);
            obj.Q_yaw = diag([obj.noise_w(4)^2, obj.noise_wb(3)^2]);
            obj.R_mag = obj.noise_m^2;
            %初始化EKF变量
            obj.X_pri = [q;0.0001;0.0001];
            obj.X_pos = [q;0.0001;0.0001];
            obj.K = zeros(6,6);
            obj.P_pri = 0.5*eye(6);
            obj.P_pos = 0.5*eye(6);
            obj.M = diag([1,1,1,0,1,1]);
            obj.A = diag([1,1,1,1,0.9996,0.9996]);
            obj.updateAttitude();
            %初始化YawKF变量
            obj.X_yaw = [obj.phi(3)*pi/180, 0]';
            obj.P_yaw = eye(2);
        end
        
        % @brief 更新姿态参考系统 
        % @param obj当前实例   acc 加速度(列向量,单位:m/s²)   gyro 角速度(列向量,单位:°/s)   mag 磁力计(列向量,单位:uT)
        % @retval None
        function UpdateMIMU(obj, acc, gyro, mag)
            obj.predict(gyro);
            obj.update(acc);
            obj.update_yaw(mag);
        end
        
        % @brief 更新姿态参考系统(仅使用IMU) 
        % @param obj当前实例    acc 加速度(列向量,单位:m/s²)   gyro 角速度(列向量,单位:°/s)
        % @retval None
        function UpdateIMU(obj, acc, gyro)
            obj.predict(gyro);
            obj.update(acc);
        end
        
        % @brief EKF预测函数
        % @param obj当前实例    gyro 角速度(列向量,单位:°/s)
        % @retval None
        function predict(obj,gyro)
            omega_x = gyro(1)*pi/180 - obj.X_pos(5);  % wx-wxb
            omega_y = gyro(2)*pi/180 - obj.X_pos(6);  % wy-wyb
            omega_z = gyro(3)*pi/180;
            Omega = [0 -omega_x -omega_y -omega_z;
                     omega_x 0 omega_z -omega_y;
                     omega_y -omega_z 0 omega_x;
                     omega_z omega_y -omega_x 0];
            O = [(obj.q(2)*obj.Delta_t)/2, (obj.q(3)*obj.Delta_t)/2;
                 -(obj.q(1)*obj.Delta_t)/2, (obj.q(4)*obj.Delta_t)/2;
                 -(obj.q(4)*obj.Delta_t)/2, -(obj.q(1)*obj.Delta_t)/2;
                 (obj.q(3)*obj.Delta_t)/2, -(obj.q(2)*obj.Delta_t)/2];
            F = [eye(4)+(Omega*obj.Delta_t)/2, O;
                 zeros(2,4), eye(2)];
            obj.X_pri = [obj.q + 0.5*Omega*obj.Delta_t*obj.q;
                    obj.X_pos(5);
                    obj.X_pos(6)];
            obj.P_pri =  F* obj.P_pos*F' + obj.Q;      
        end
        
        % @brief EKF更新函数
        % @param obj当前实例   acc 加速度(列向量,单位:m/s²)
        % @retval None
        function update(obj,acc)
            Z = obj.getZ(acc);
            H = [-2*obj.q(3), 2*obj.q(4), -2*obj.q(1), 2*obj.q(2), zeros(1,2);
                 2*obj.q(2), 2*obj.q(1), 2*obj.q(4), 2*obj.q(3), zeros(1,2);
                 2*obj.q(1), -2*obj.q(2), -2*obj.q(3), 2*obj.q(4), zeros(1,2)];
            hx = [2*(obj.q(2)*obj.q(4) - obj.q(1)*obj.q(3));
                  2*(obj.q(3)*obj.q(4) + obj.q(1)*obj.q(2));
                  1 - 2*(obj.q(2)^2 + obj.q(3)^2)];
            obj.K = obj.P_pri*H'/(H*obj.P_pri*H' + obj.R);
            obj.X_pos = obj.X_pri + obj.M*obj.K*(Z - hx);
            obj.P_pos = obj.A*(eye(6) - obj.K*H)*obj.P_pri;
            obj.updateAttitude();
        end

        % @brief EKF更新航向角
        % @param obj当前实例    mag 磁力计(列向量,单位:uT)
        % @retval None
        function update_yaw(obj, mag)
            F = [1, -obj.Delta_t; 0, 1];
            H = [1, 0];
            yaw = obj.getYaw(obj.phi, mag);
            obj.X_yaw = [obj.phi(3)*pi/180, obj.X_yaw(2)]';
            obj.P_yaw = F*(1.0001*obj.P_yaw)*F' + obj.Q_yaw;
            obj.K_yaw = obj.P_yaw*H'/(H*obj.P_yaw*H' + obj.R_mag);
            obj.X_yaw = obj.X_yaw + obj.K_yaw*(yaw - H*obj.X_yaw);
            obj.P_yaw = (eye(2) - obj.K_yaw*H)*obj.P_yaw;
            obj.phi = [obj.phi(1); obj.phi(2); obj.X_yaw(1)*180/pi];
            obj.q = Utils.getQFromPhi(obj.phi);     
        end

   
        % @brief 获取当前航向角
        % @param obj当前实例   phi 当前姿态欧拉角表示(列向量)    mag 磁力计(列向量,单位:uT)
        % @retval yaw  磁力计观测航向角
        function yaw = getYaw(~, phi,mag)
            % 计算重力向量的倾斜角度
            roll = phi(2)*pi/180;
            pitch = phi(1)*pi/180;
            % 计算磁场向量的偏航角
            Mxb = mag(1)*cos(roll) + mag(2)*sin(pitch)*sin(roll) + mag(3)*cos(pitch)*sin(roll); 
            Myb = mag(2)*cos(pitch) - mag(3)*sin(pitch);
            yaw = atan2(double(Mxb), double(Myb));
        end

        % @brief 更新当前参考姿态
        % @param obj当前实例
        % @retval None
        function updateAttitude(obj)
            obj.q = Utils.normalizeQ(obj.X_pos(1:4));
            obj.X_pos(1:4) = obj.q;
            obj.phi = Utils.getPhiFromQ(obj.q);
        end
        
        % @brief 获取观测矩阵Z
        % @param acc 加速度向量
        % @retval Z 观测矩阵
        function Z = getZ(~,acc)            
            Z = acc/norm(acc,2);
        end
        
        % @brief 返回参考姿态Q四元数表示
        % @param obj当前实例
        % @retval Q 当前姿态四元数表示(列向量)
        function Q = returnQ(obj)
            Q = obj.q;
        end
        
        % @brief 返回参考姿态Phi欧拉角表示
        % @param obj当前实例
        % @retval Phi 当前姿态欧拉角表示(列向量)
        function Phi = returnPhi(obj)
            Phi = obj.phi;
        end
        
      end

end