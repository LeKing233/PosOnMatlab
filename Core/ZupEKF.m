classdef ZupEKF <handle
    properties(Access = private)
        
       %扩展卡尔曼滤波变量
        X;              %状态
        hx1;            %重力加速度观测方程
        hx2;            %地磁场观测方程
        hx3;            %零速度观测方程
        hx4;            %位移观测方程
        H1;             %重力加速度观测转移矩阵
        H2;             %地磁场观测转移矩阵
        H3;             %零速度观测转移矩阵
        H4;             %位移观测转移矩阵
        K1;             %卡尔曼增益
        K2;             %卡尔曼增益
        K3;             %卡尔曼增益
        K4;             %卡尔曼增益
        P;              %误差协方差矩阵
        Q;              %预测噪声协方差矩阵 
        R1;             %观测噪声协方差矩阵
        R2;             %观测噪声协方差矩阵
        R3;             %观测噪声协方差矩阵
        R4;             %观测噪声协方差矩阵
        S;              %状态量渐消记忆矩阵
        A;              %修正矩阵
        B;              %修正矩阵

        %姿态参考系统参数
        q;              %姿态四元数表示
        phi;            %姿态欧拉角表示
        fc;             %比力
        vel;            %速度
        pos;            %位移
        err_g;          %重力加速度观测误差
        err_m;          %地磁场观测误差
        err_v;          %零速度观测误差
        err_p;          %位移观测误差
        rk_g;           %重力加速度观测量检测值
        rk_m;           %地磁场观测量检测值
        Delta_t;        %采集间隔
        gravity = [0;0;9.8];
        
        %参数，用于构建系统噪声矩阵Q和观测噪声矩阵R
        noise_w;        %预测噪声（陀螺仪噪声）
        noise_wb;       %预测噪声（陀螺仪偏置噪声）
        noise_f;        %预测噪声（加速度计噪声）
        noise_g;        %观测噪声（重力加速度噪声）
        noise_m;        %观测噪声（磁力计噪声）
        noise_v;        %观测噪声（速度噪声）
        noise_h;        %观测噪声（高度计噪声）
        fade_q;         %姿态渐消记忆因子
        fade_wb;        %陀螺仪偏置渐消记忆因子
        fade_v;         %速度渐消记忆因子
        fade_p;         %位移渐消记忆因子
    end
   
    methods(Access = public)
        % @brief ZupEKF 构造函数 
        % @param q 初始化姿态四元数表示(列向量)    varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = ZupEKF(q, varargin)
            %读取设置
            EKF_setting = inputParser;
            addParameter(EKF_setting, 'SamplePeriod', 0.006);       %采集间隔
            addParameter(EKF_setting, 'noiseW',160);                %陀螺仪噪声
            addParameter(EKF_setting, 'noiseWb',0.01);              %陀螺仪偏置噪声
            addParameter(EKF_setting, 'noiseF',1400);               %加速度计噪声
            addParameter(EKF_setting, 'noiseG',1100);               %重力加速度噪声
            addParameter(EKF_setting, 'noiseM',8500);               %磁力计噪声
            addParameter(EKF_setting, 'noiseV',1000);               %速度噪声
            addParameter(EKF_setting, 'noiseH',50);                 %高度计噪声
            addParameter(EKF_setting, 'fadeQ', 1.00002);            %姿态渐消记忆因子
            addParameter(EKF_setting, 'fadeWb', 1.00002);           %陀螺仪偏置渐消记忆因子
            addParameter(EKF_setting, 'fadeV', 1.00002);            %速度渐消记忆因子
            addParameter(EKF_setting, 'fadeP', 1.00002);            %位移渐消记忆因子
            parse(EKF_setting, varargin{:});
            obj.Delta_t = EKF_setting.Results.SamplePeriod;
            obj.noise_w = EKF_setting.Results.noiseW * ones(4,1);
            obj.noise_wb = EKF_setting.Results.noiseWb * ones(3,1);
            obj.noise_f = EKF_setting.Results.noiseF * ones(3,1);
            obj.noise_g = EKF_setting.Results.noiseG * ones(3,1); 
            obj.noise_m = EKF_setting.Results.noiseM * ones(3,1);
            obj.noise_v = EKF_setting.Results.noiseV * ones(3,1);
            obj.noise_h = EKF_setting.Results.noiseH * ones(1,1);
            obj.fade_q = EKF_setting.Results.fadeQ;
            obj.fade_wb = EKF_setting.Results.fadeWb;
            obj.fade_v = EKF_setting.Results.fadeV;
            obj.fade_p = EKF_setting.Results.fadeP;
            %状态量初始化
            obj.init(q);
        end
        
        % @brief ZupEKF系统变量初始化初始化 
        % @param obj当前实例  q 初始化姿态四元数表示(列向量)
        % @retval None
        function init(obj, q)
            %更新姿态
            obj.q = q;
            obj.phi = Utils.getPhiFromQ(obj.q);
            %初始化噪声协方差矩阵
            obj.R1 = diag(obj.noise_g.^2);
            obj.R2 = diag(obj.noise_m.^2);
            obj.R3 = diag(obj.noise_v.^2);
            obj.R4 = diag(obj.noise_h.^2);
            %初始化EKF变量
            obj.X = [q; zeros(9,1)];
            obj.P = 0.5*eye(13);
            obj.updateVariable();
            obj.S = diag([obj.fade_q*ones(1,4), obj.fade_wb*ones(1,3), obj.fade_v*ones(1,3), obj.fade_p*ones(1,3)]);
            obj.A = diag([ones(1,3), 0, ones(1,9)]);
            obj.B = diag([1, 0, 0, ones(1,10)]);
        end
        
        % @brief ZupEKF预测函数
        % @param obj当前实例  acc 加速度(列向量,单位:m/s²)  gyro 角速度(列向量,单位:°/s)
        % @retval None
        function predict(obj, acc, gyro)
            %消除陀螺仪偏置误差
            omega_x = gyro(1)*pi/180 - obj.X(5);    % wx-wxb
            omega_y = gyro(2)*pi/180 - obj.X(6);    % wy-wyb
            omega_z = gyro(3)*pi/180 - obj.X(7);    % wz-wzb
            %计算当前姿态比力
            obj.fc = Utils.quaternProd(Utils.quaternProd(obj.q', [0; acc]'), Utils.quaternConj(obj.q'))';
            obj.fc = obj.fc(2:4) - obj.gravity;
            %计算状态转移方程fx及雅可比矩阵F
            Omega = [0          -omega_x        -omega_y        -omega_z;
                     omega_x    0               omega_z         -omega_y;
                     omega_y    -omega_z        0               omega_x;
                     omega_z    omega_y         -omega_x        0        ];   
            G = 0.5*obj.Delta_t*[obj.q(2),  obj.q(3),   obj.q(4);
                                -obj.q(1),  obj.q(4),   -obj.q(3);
                                -obj.q(4),  -obj.q(1),  obj.q(2);
                                obj.q(3),   -obj.q(2),  -obj.q(1)];
            D = 2*obj.Delta_t*[acc(1)*obj.q(1)-acc(2)*obj.q(4)+acc(3)*obj.q(3), acc(1)*obj.q(2)+acc(2)*obj.q(3)+acc(3)*obj.q(4), ...
                               acc(2)*obj.q(2)-acc(1)*obj.q(3)+acc(3)*obj.q(1), acc(3)*obj.q(2)-acc(1)*obj.q(4)-acc(2)*obj.q(1);
                               acc(2)*obj.q(1)+acc(1)*obj.q(4)-acc(3)*obj.q(2), acc(1)*obj.q(3)-acc(2)*obj.q(2)-acc(3)*obj.q(1), ...
                               acc(1)*obj.q(2)+acc(2)*obj.q(3)+acc(3)*obj.q(4), acc(1)*obj.q(1)-acc(2)*obj.q(4)+acc(3)*obj.q(3);
                               acc(2)*obj.q(2)-acc(1)*obj.q(3)+acc(3)*obj.q(1), acc(2)*obj.q(1)+acc(1)*obj.q(4)-acc(3)*obj.q(2), ...
                               acc(2)*obj.q(4)-acc(1)*obj.q(1)-acc(3)*obj.q(3), acc(1)*obj.q(2)+acc(2)*obj.q(3)+acc(3)*obj.q(4)];
            fx = [obj.q + 0.5*Omega*obj.Delta_t*obj.q;      %计算状态转移方程fx
                    obj.X(5);
                    obj.X(6);
                    obj.X(7);
                    obj.X(8:10) + obj.fc*obj.Delta_t;
                    obj.X(11:13) + obj.X(8:10)*obj.Delta_t]; 
            F = [eye(4)+0.5*Omega*obj.Delta_t, G, zeros(4,6);       %计算状态转移雅可比矩阵F
                zeros(3,4), eye(3), zeros(3,6);
                D, zeros(3,3), eye(3), zeros(3,3);
                zeros(3,7), obj.Delta_t*eye(3), eye(3)];
            %预测过程
            obj.X = fx;
            obj.P =  F* (obj.S*obj.P)*F' + obj.getQ(obj.fc);
            %更新状态变量
            obj.updateVariable();
        end
        
        % @brief EKF更新函数
        % @param obj当前实例   acc 加速度(列向量,单位:m/s²)
        % @retval None
        function update(obj, acc, mag)
            % 获取参考加速度及地磁场
            norm_acc = acc/norm(acc,2);
            norm_mag = mag/norm(mag,2);
            b = Utils.quaternProd(obj.q', Utils.quaternProd([0; norm_mag]', Utils.quaternConj(obj.q')))';
            b = [0 0 norm([b(2) b(3)]) b(4)];   
            %量测方程一
            obj.hx1 = [2*(obj.q(2)*obj.q(4) - obj.q(1)*obj.q(3));       %计算量测方程hx1
                  2*(obj.q(3)*obj.q(4) + obj.q(1)*obj.q(2));
                  1 - 2*(obj.q(2)^2 + obj.q(3)^2)];
            obj.H1 = [-2*obj.q(3),   2*obj.q(4),     -2*obj.q(1),    2*obj.q(2),     zeros(1,9);        %计算量测雅可比矩阵H1
                 2*obj.q(2),    2*obj.q(1),     2*obj.q(4),     2*obj.q(3),     zeros(1,9);
                 0,             -4*obj.q(2),    -4*obj.q(3),    0,              zeros(1,9)];
            obj.err_g = norm_acc - obj.hx1;     %计算观测误差 
            obj.rk_g = obj.examineEg();
            if obj.rk_g <= 240
                obj.K1 = obj.P*obj.H1'/(obj.H1*obj.P*obj.H1' + obj.R1);
                obj.X = obj.X + obj.A*obj.K1*obj.err_g;
                obj.P = (eye(13) - obj.K1*obj.H1)*obj.P;
            end
            %量测方程二
            M = [2*b(3)*obj.q(4) - 2*b(4)*obj.q(3),     2*b(3)*obj.q(3) + 2*b(4)*obj.q(4),      2*b(3)*obj.q(2) - 2*b(4)*obj.q(1),      2*b(3)*obj.q(1) + 2*b(4)*obj.q(2);
                 2*b(4)*obj.q(2),                        -4*b(3)*obj.q(2) + 2*b(4)*obj.q(1),     2*b(4)*obj.q(4),                        -4*b(3)*obj.q(4) + 2*b(4)*obj.q(3);
                 -2*b(3)*obj.q(2),                       -2*b(3)*obj.q(1) - 4*b(4)*obj.q(2) ,    2*b(3)*obj.q(4) - 4*b(4)*obj.q(3),      2*b(3)*obj.q(3)];    
            obj.hx2 = [2*b(3)*(obj.q(1)*obj.q(4) + obj.q(2)*obj.q(3)) + 2*b(4)*(obj.q(2)*obj.q(4) - obj.q(1)*obj.q(3));     %计算量测方程hx2
                   2*b(3)*(0.5 - obj.q(2)^2 - obj.q(4)^2) + 2*b(4)*(obj.q(1)*obj.q(2) + obj.q(3)*obj.q(4));
                   2*b(3)*(obj.q(3)*obj.q(4) - obj.q(1)*obj.q(2)) + 2*b(4)*(0.5 - obj.q(2)^2 - obj.q(3)^2)];
            obj.H2 = [M , zeros(3,9);];     %计算量测雅可比矩阵H2
            obj.err_m = norm_mag - obj.hx2;     %计算观测误差
            obj.rk_m = obj.examineEm();
            if obj.rk_m <= 320
                obj.K2 = obj.P*obj.H2'/(obj.H2*obj.P*obj.H2' + obj.R2);
                obj.X = obj.X + obj.B*obj.K2*obj.err_m;
                obj.P = (eye(13) - obj.K2*obj.H2)*obj.P;
            end
            %量测方程三
            obj.hx3 = obj.X(8:10);      %计算量测方程hx3
            obj.H3 = [zeros(3,7), eye(3), zeros(3,3)];      %计算量测雅可比矩阵H3
            obj.err_v = zeros(3,1) - obj.hx3;     %计算观测误差
            obj.K3 = obj.P*obj.H3'/(obj.H3*obj.P*obj.H3' + obj.R3);
            obj.X = obj.X + obj.K3*obj.err_v;
            obj.P = (eye(13) - obj.K3*obj.H3)*obj.P;
            %量测方程四
            obj.hx4 = obj.X(13);        %计算量测方程hx4
            obj.H4 = [zeros(1,12), 1];      %计算量测雅可比矩阵H4
            obj.err_p = zeros(1,1) - obj.hx4;     %计算观测误差
            obj.K4 = obj.P*obj.H4'/(obj.H4*obj.P*obj.H4' + obj.R4);
            obj.X = obj.X + obj.K4*obj.err_p;
            obj.P = (eye(13) - obj.K4*obj.H4)*obj.P;
            %更新状态变量
            obj.updateVariable();
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
        
        % @brief 返回当前比力Fc
        % @param obj当前实例
        % @retval Fc 当前比力(列向量)
        function Fc = returnFc(obj)
            Fc = obj.fc;
        end
        
        % @brief 返回当前速度V
        % @param obj当前实例
        % @retval V 当前速度(列向量)
        function V = returnV(obj)
            V = obj.vel;
        end
        
        % @brief 返回当前位移P
        % @param obj当前实例
        % @retval P 当前位移(列向量)
        function P = returnP(obj)
            P = obj.pos;
        end
        
        % @brief 返回陀螺仪偏置
        % @param obj当前实例
        % @retval wb 陀螺仪偏置
        function Wb = returnWb(obj)
            Wb = obj.X(5:7);
        end
        
        % @brief 返回观测重力加速度差值
        % @param obj当前实例
        % @retval Eg 重力加速度差值
        function Eg = returnEg(obj)
            Eg = obj.err_g;
        end
        
        % @brief 返回观测地磁场差值
        % @param obj当前实例
        % @retval Em 地磁场差值
        function Em = returnEm(obj)
            Em = obj.err_m;
        end
        
        % @brief 返回观测零速度差值
        % @param obj当前实例
        % @retval Ev 零速度差值
        function Ev = returnEv(obj)
            Ev = obj.err_v;
        end
        
        % @brief 返回卡尔曼增益
        % @param obj当前实例
        % @retval K 卡尔曼增益
        function K = returnK(obj)
            K = [obj.K1; obj.K2; obj.K3; obj.K4];
        end
        
        % @brief 返回重力加速度检测值
        % @param obj当前实例
        % @retval Rkg 重力加速度检测值
        function Rkg = returnRg(obj)
            Rkg = obj.rk_g;
        end
        
        % @brief 返回地磁场检测值
        % @param obj当前实例
        % @retval Rkm 地磁场检测值
        function Rkm = returnRm(obj)
            Rkm = obj.rk_m;
        end
        
    end
        
    methods(Access = private)

        % @brief 获取预测噪声协方差矩阵Q
        % @param acc 加速度向量
        % @retval Q 预测噪声协方差矩阵
        function Q = getQ(obj,f)
            err_q = 0.5*obj.Delta_t*[-obj.q(2)*obj.noise_w(1) - obj.q(3)*obj.noise_w(2) - obj.q(4)*obj.noise_w(3);
                                     obj.q(1)*obj.noise_w(1)  - obj.q(4)*obj.noise_w(2) + obj.q(3)*obj.noise_w(3);
                                     obj.q(4)*obj.noise_w(1) + obj.q(1)*obj.noise_w(2) - obj.q(2)*obj.noise_w(3) ;
                                     -obj.q(3)*obj.noise_w(1) + obj.q(2)*obj.noise_w(2) + obj.q(1)*obj.noise_w(3)];
            q_true = Utils.normalizeQ(obj.q+err_q);
            errf = Utils.quaternProd(Utils.quaternProd(q_true', [0; f+obj.noise_f]'), Utils.quaternConj(q_true'))'...
                   - Utils.quaternProd(Utils.quaternProd(obj.q', [0; f]'), Utils.quaternConj(obj.q'))';
            errf = errf(2:4);
            errp = errf*obj.Delta_t;
            Q = diag([err_q.^2; obj.noise_wb.^2; errf.^2; errp.^2]);
        end
        
        % @brief 更新当前参考姿态
        % @param obj当前实例`
        % @retval None
        function updateVariable(obj)
            obj.q = Utils.normalizeQ(obj.X(1:4));
            if obj.q(1) < 0
                obj.q = -obj.q;
            end
            obj.X(1:4) = obj.q;
            obj.phi = Utils.getPhiFromQ(obj.q);
            obj.vel = obj.X(8:10);
            obj.pos = obj.X(11:13);
        end
        
        % @brief 加速度观测值检验
        % @param obj当前实例
        % @retval rkg  加速度量测检验值
        function rkg = examineEg(obj)
            Dk = obj.H1*obj.P*obj.H1' + obj.R1;
            rkg = obj.err_g'/Dk*obj.err_g;
        end
        
        % @brief 地磁场观测值检验
        % @param obj当前实例
        % @retval rkm  地磁场量测检验值
        function rkm = examineEm(obj)
            Dk = obj.H2*obj.P*obj.H2' + obj.R2;
            rkm = obj.err_m'/Dk*obj.err_m;
        end
        
    end

end