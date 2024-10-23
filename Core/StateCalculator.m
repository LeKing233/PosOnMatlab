classdef StateCalculator <handle
    
    properties
        
        %时间戳变量
        TimeStamp;  %时间戳
        SupTime;    %支撑相时刻
        
        %状态变量
        AccENU;     %东北天坐标系下的加速度向量
        V;          %航位推算速度V
        P;          %航位推算位移P
        Cbn;        %航位推算姿态矩阵Cbn
        Phi;        %航位推算欧拉角Phi
        Q;          %航位推算四元数Q
        q;          %互补滤波推算q
        phi;        %互补滤波推算phi
        err_phi;    %姿态误差观测值phi
        
        GaitPhase;  %步态相位
        mStateSeq;  %状态向量序列
        sup_ctr;%   支撑相时间戳计数器
   
        %常量
        delta_t = 0.006;    %更新间隔 (秒)
        gravity = 9.8;      %重力加速度数值(米每平方秒)
        gravityVec;         %重力加速度向量

        %核心组件
        iHandler;           %imu数据管理类
        pHandler;           %足底压力传感器数据管理类
        zuptKF;             %卡尔曼滤波器（零速度修正特殊版本）
        aligner;            %姿态初始对准器
        ahrs;               %姿态参考系统
        
        %算法设置
        ob_Attitude;        %姿态修正设置
        ob_Position;        %位移修正设置
        ahrs_algorithm;     %AHRS采用算法 
        aligner_algorithm;  %Aligner采用算法

        %ZUP卡尔曼滤波算法参数
        ZUP_noiseW;         %预测噪声（陀螺仪噪声）
        ZUP_noiseF;         %预测噪声（加速度计噪声）
        ZUP_noiseG;         %观测噪声噪声（互补滤波重力加速度噪声）
        ZUP_noiseM;         %观测噪声（磁力计噪声）
        ZUP_noiseV;         %观测噪声（速度噪声）
        ZUP_noiseH;         %观测噪声（高度计计噪声）
        ZUP_factorS;        %渐消记忆因子
        
        %EKF航位参考算法参数
        EKF_noiseW;         %预测噪声（陀螺仪噪声）
        EKF_noiseWb;        %预测噪声（陀螺仪偏置噪声）
        EKF_noiseF;         %观测噪声（加速度计噪声）
        EKF_noiseM;         %观测噪声（磁力计噪声）
        
        %Mahony航位参考算法参数
        Mahony_Kp;          %算法比例增益Kp
        Mahony_Ki;          %算法积分增益Ki
        
        %Madgwick航位参考算法参数
        Madgwick_Beta;      %算法增益beta
    end

    methods
        
        % @brief StateCalculator构造函数 
        % @param imuHandler IMU数据处理示例   plantarHandler 压力传感器数据处理示例
        % @param varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = StateCalculator(imuHandler,plantarHandler,varargin)          
            %常量初始化
            obj.gravityVec = [0;0;obj.gravity];
            %构建核心组件
            obj.iHandler = imuHandler;
            obj.pHandler = plantarHandler;
            %解析算法设置
            Algorithm_setting = inputParser;
            addParameter(Algorithm_setting, 'Attitude', Utils.WHOLE_ATTITUDE);      %默认启用三轴姿态修正
            addParameter(Algorithm_setting, 'Position', Utils.VERTICAL_POSITION);   %默认启用高度修正
            addParameter(Algorithm_setting, 'Aligner', Utils.AHRS_MADGWICK);             %默认启用EKF算法
            addParameter(Algorithm_setting, 'AHRS', Utils.AHRS_EKF);                %默认启用EKF算法
            %读取ZUP卡尔曼滤波参数设置
            addParameter(Algorithm_setting, 'ZUP_noiseW', 0.02);        %预测噪声（陀螺仪噪声）
            addParameter(Algorithm_setting, 'ZUP_noiseF', 600);         %预测噪声（加速度计噪声）
            addParameter(Algorithm_setting, 'ZUP_noiseG', 300);         %观测噪声噪声（互补滤波重力加速度噪声）
            addParameter(Algorithm_setting, 'ZUP_noiseM', 450);         %观测噪声（磁力计噪声）
            addParameter(Algorithm_setting, 'ZUP_noiseV', 300);         %观测噪声（速度噪声）
            addParameter(Algorithm_setting, 'ZUP_noiseH', 40);          %观测噪声（高度计计噪声）
            addParameter(Algorithm_setting, 'ZUP_factorS', 1.00005);    %渐消记忆因子
            %读取EKF航位参考算法参数设置
            addParameter(Algorithm_setting, 'EKF_noiseW', 0.02);        %预测噪声（陀螺仪噪声）
            addParameter(Algorithm_setting, 'EKF_noiseWb', 0.001);      %预测噪声（陀螺仪偏置噪声）
            addParameter(Algorithm_setting, 'EKF_noiseF', 40);          %观测噪声（加速度计噪声）
            addParameter(Algorithm_setting, 'EKF_noiseM', 50);         %观测噪声（磁力计噪声）
            %读取Mahony航位参考算法参数设置
            addParameter(Algorithm_setting, 'Mahony_Kp', 0.01);         %算法比例增益Kp
            addParameter(Algorithm_setting, 'Mahony_Ki', 0.0025);       %算法积分增益Ki
            %读取Madgwick航位参考算法参数设置
            addParameter(Algorithm_setting, 'Madgwick_Beta', 0.01);     %算法增益beta
            %解析varargin变量
            parse(Algorithm_setting, varargin{:});
            %初始化算法设置
            obj.ob_Attitude = Algorithm_setting.Results.Attitude;
            obj.ob_Position = Algorithm_setting.Results.Position;
            obj.aligner_algorithm = Algorithm_setting.Results.Aligner;
            obj.ahrs_algorithm = Algorithm_setting.Results.AHRS;
            %初始化ZUP卡尔曼滤波参数
            obj.ZUP_noiseW = Algorithm_setting.Results.ZUP_noiseW;
            obj.ZUP_noiseF = Algorithm_setting.Results.ZUP_noiseF;
            obj.ZUP_noiseG = Algorithm_setting.Results.ZUP_noiseG;
            obj.ZUP_noiseM = Algorithm_setting.Results.ZUP_noiseM;
            obj.ZUP_noiseV = Algorithm_setting.Results.ZUP_noiseV;
            obj.ZUP_noiseH = Algorithm_setting.Results.ZUP_noiseH;
            obj.ZUP_factorS = Algorithm_setting.Results.ZUP_factorS;
            %初始化EKF航位参考算法参数
            obj.EKF_noiseW = Algorithm_setting.Results.EKF_noiseW;
            obj.EKF_noiseWb = Algorithm_setting.Results.EKF_noiseWb;
            obj.EKF_noiseF = Algorithm_setting.Results.EKF_noiseF;
            obj.EKF_noiseM = Algorithm_setting.Results.EKF_noiseM;
            %初始化Mahony航位参考算法参数
            obj.Mahony_Kp = Algorithm_setting.Results.Mahony_Kp;
            obj.Mahony_Ki = Algorithm_setting.Results.Mahony_Ki;
            %初始化Madgwick航位参考算法参数
            obj.Madgwick_Beta = Algorithm_setting.Results.Madgwick_Beta;
            %构建卡尔曼滤波器
            obj.zuptKF  = ZuptKalmanFilter('Attitude', obj.ob_Attitude, 'Position', obj.ob_Position, ...
                'noiseW', obj.ZUP_noiseW, 'noiseF', obj.ZUP_noiseF, 'noiseG', obj.ZUP_noiseG, ...
                'noiseM', obj.ZUP_noiseM, 'noiseV', obj.ZUP_noiseV, 'noiseH', obj.ZUP_noiseH, ...
                'factorS', obj.ZUP_factorS);
        end

       % @brief 状态初始化
       % @param phi 初始姿态角
       % @retval None
       function stateInit(obj,phi)
            obj.Phi = phi;
            obj.AccENU = zeros(3,1);
            obj.V = zeros(3,1);
            obj.P = zeros(3,1);
            obj.Cbn = Utils.getCbnFromPhi(obj.Phi);
            obj.Q = Utils.getQFromCbn(obj.Cbn);
            obj.zuptKF.init();%滤波器初始化 
            obj.GaitPhase = Utils.Phase_Unknown;
            obj.initStateSeq();
            obj.saveStateToSeq(1);
            obj.sup_ctr = 0;
       end
       
       % @brief 初始化储存序列
       % @param None
       % @retval None 
       function  initStateSeq(obj)          
            obj.mStateSeq = struct( 'V',MatList(),'P',MatList(),'Cbn',MatList(),'Phi',MatList(),'AccENU',MatList(), ...
                                    'GaitPhase',MatList(), ...                                    
                                    'KF_X',MatList(), 'KF_P',MatList(),'KF_K',MatList(),...
                                    'Q',MatList(),'SupTime',MatList(),'SupQ',MatList(),'SupPhi',MatList(), 'ErrPhi',MatList()...
                                 );
       end

        % @brief 将结果保存进序列
        % @param index 迭代器当前索引
        % @retval None 
        function saveStateToSeq(obj,index)
            %状态量
            obj.mStateSeq.AccENU.addOne(obj.AccENU');
            obj.mStateSeq.V.addOne(obj.V');
            obj.mStateSeq.P.addOne(obj.P');
            obj.mStateSeq.Phi.addOne(obj.Phi');
            obj.mStateSeq.Cbn.addOne([obj.Cbn;zeros(1,3)]);
            %步态检测参数
            obj.mStateSeq.GaitPhase.addOne(obj.GaitPhase);
            %kalman滤波器参数
            obj.mStateSeq.KF_X.addOne(obj.zuptKF.X');
            obj.mStateSeq.KF_P.addOne(obj.zuptKF.P);
            obj.mStateSeq.KF_K.addOne(obj.zuptKF.K');
            %保存其他状态量
            obj.mStateSeq.Q.addOne(obj.Q');
            if obj.GaitPhase == Utils.Phase_Stance
                obj.mStateSeq.SupTime.addOne(obj.iHandler.mTSeq(index));
                obj.mStateSeq.SupPhi.addOne(obj.phi');
                obj.mStateSeq.SupQ.addOne(obj.q');
                obj.mStateSeq.ErrPhi.addOne(obj.err_phi');
            end        
            
        end
       
        % @brief 将结果保存为Mat
        % @param None
        % @retval None
        function stateSeqToMat(obj)          
            %状态量
            obj.mStateSeq.AccENU = obj.mStateSeq.AccENU.toMat();
            obj.mStateSeq.V = obj.mStateSeq.V.toMat();
            obj.mStateSeq.P = obj.mStateSeq.P.toMat();
            obj.mStateSeq.Phi =  obj.mStateSeq.Phi.toMat();
            obj.mStateSeq.Cbn = obj.mStateSeq.Cbn.toMat() ;
            %步态检测参数
            obj.mStateSeq.GaitPhase =  obj.mStateSeq.GaitPhase.toMat();
            %kalman滤波器参数
            obj.mStateSeq.KF_X = obj.mStateSeq.KF_X.toMat();
            obj.mStateSeq.KF_P = obj.mStateSeq.KF_P.toMat();
            obj.mStateSeq.KF_K = obj.mStateSeq.KF_K.toMat();
            obj.mStateSeq.Q = obj.mStateSeq.Q.toMat();
            obj.mStateSeq.SupTime = obj.mStateSeq.SupTime.toMat();
            obj.mStateSeq.SupPhi = obj.mStateSeq.SupPhi.toMat();
            obj.mStateSeq.SupQ = obj.mStateSeq.SupQ.toMat();
            obj.mStateSeq.ErrPhi = obj.mStateSeq.ErrPhi.toMat();
        end       
       
        % @brief 状态解算
        % @param gaitDetectType
        % @retval None
        function result = solveState(obj,gaitDetectType)  
            %初始化姿态对准器
            if obj.aligner_algorithm == Utils.AHRS_MADGWICK
                obj.aligner = Aligner(obj.iHandler.mFSeq(:,1)', obj.iHandler.mMSeq(:,1), 'Aligner',obj.aligner_algorithm,...
                                      'Madgwick_Beta',obj.Madgwick_Beta);
            elseif obj.aligner_algorithm == Utils.AHRS_MAHONY
                obj.aligner = Aligner(obj.iHandler.mFSeq(:,1)', obj.iHandler.mMSeq(:,1), 'Aligner',obj.aligner_algorithm,...
                                      'Mahony_Kp',obj.Mahony_Kp, 'Mahony_Ki',obj.Mahony_Ki);
            elseif obj.aligner_algorithm == Utils.AHRS_EKF
                obj.aligner = Aligner(obj.iHandler.mFSeq(:,1)', obj.iHandler.mMSeq(:,1), 'Aligner',obj.aligner_algorithm,...
                                      'EKF_noiseW',obj.EKF_noiseW, 'EKF_noiseWb',obj.EKF_noiseWb,...
                                      'EKF_noiseF',obj.EKF_noiseF, 'EKF_noiseM',obj.EKF_noiseM);
            end
            %姿态对准器迭代
            for i = 2:Utils.ALIGNER_STAMP
                obj.q = obj.aligner.accurateAlignment(obj.iHandler.mFSeq(:,i), obj.iHandler.mWSeq(:,i), obj.iHandler.mMSeq(:,i));
                obj.phi  = Utils.getPhiFromQ(obj.q);
            end
            obj.stateInit(obj.phi);%状态初始化
%             obj.stateInit(obj.iHandler.phi0);%状态初始化
            obj.aligner = [];   %释放姿态对准器
            
            %序列化迭代计算
            for i = Utils.START_STAMP:length(obj.iHandler.mTSeq)
                %记录上一时刻状态量
                Cbn_prev = obj.Cbn;
                V_prev = obj.V;
                P_prev = obj.P;

                %计算当前时刻的状态量
                obj.calculateNextCbn(Cbn_prev,obj.iHandler.mWSeq(:,i));
                obj.calculateNextV(V_prev,obj.iHandler.mFSeq(:,i),obj.Cbn,obj.iHandler.mFSeq(:,i-1),Cbn_prev);
                obj.calculateNextP(P_prev,obj.V,V_prev);
                obj.Phi = Utils.getPhiFromCbn(obj.Cbn);
                obj.Q = Utils.getQFromCbn(obj.Cbn);
                obj.AccENU = obj.Cbn*obj.iHandler.mFSeq(:,i);

                %计算步态
                obj.GaitPhase = obj.iHandler.getPhase(i);

                %依据步态执行零速度更新
                %零速度修正
                if obj.GaitPhase == Utils.Phase_Swing 
                    %摆动相
                    obj.zuptKF.X = zeros(9,1); %误差项置零
                    if obj.sup_ctr ~= 0
                        obj.sup_ctr = 0;
                        obj.ahrs = [];
                    end   
                elseif obj.GaitPhase == Utils.Phase_Stance
                    %支撑相
                    %预测
                    F = obj.zuptKF.getF(obj,obj.Cbn,obj.iHandler.mFSeq(:,i));
                    G = obj.zuptKF.getG(obj,obj.Cbn);
                    obj.zuptKF.predict(F,G);
                    %更新
                    Z = obj.getZ(i);
                    obj.zuptKF.update(Z);
                    %修正
                    obj.zuptKF.reviseVn(obj,obj.zuptKF.X(4:6));
                    obj.zuptKF.revisePn(obj,obj.zuptKF.X(7:9));
                    obj.zuptKF.reviseCbn(obj,obj.zuptKF.X(1:3));
                    obj.Phi = Utils.getPhiFromCbn(obj.Cbn);
                    obj.Q = Utils.getQFromCbn(obj.Cbn);
                    obj.initARHS();%初始化姿态观测算法

                elseif  obj.GaitPhase == Utils.Phase_Unknown %计算错误

                end
                
                %将计算结果保存
                obj.saveStateToSeq(i);
            end

            %打标签
            obj.mStateSeq.Legend = gaitDetectType;
            %将结果序列作为返回值返回
            obj.stateSeqToMat();
            result = obj.mStateSeq; 
        end
        
        % @brief 姿态更新算法——二子样算法
        % @param Cbn_prev 前一时刻旋转矩阵Cbn  w角速度
        % @retval None
       
        function  calculateNextCbn(obj,Cbn_prev,w)
            w = w * pi /180;%弧度制转角度制
            I = eye(3);
            wx = Utils.getCrossMatrix(w);%角速度w构成的反对称矩阵[w×]
            obj.Cbn = Cbn_prev * ((2*I + wx* obj.delta_t)/(2*I - wx*obj.delta_t));
        end
        
        % @brief 速度更新算法
        % @param V_prev 前一时刻速度      f加速度     Cbn 当前旋转矩阵Cbn  
        % @param f_prev 前一时刻加速度    Cbn_prev 前一时刻旋转矩阵Cbn
        % @retval None
        function calculateNextV(obj,V_prev,f,Cbn,f_prev,Cbn_prev)
            obj.V = V_prev + (0.5*(Cbn*f+Cbn_prev*f_prev) - obj.gravityVec ) * obj.delta_t;
        end
        
        % @brief 位移更新算法
        % @param Pn_prev 前一时刻位移     Vn 当前速度     Vn_prev 前一时刻速度 
        % @retval None
        function calculateNextP(obj,Pn_prev,Vn,Vn_prev)
            obj.P = Pn_prev + 0.5*(Vn_prev + Vn) * obj.delta_t;
        end
        
        % @brief 获取观测矩阵Z
        % @param index 迭代器当前索引 
        % @retval Z 观测矩阵
        function Z = getZ(obj,index)
            Z = -obj.V;
            if obj.sup_ctr == 0
                obj.err_phi = [0, 0, 0]';
                if obj.ob_Attitude == Utils.WHOLE_ATTITUDE %如果是三轴姿态修正模式
                    Zyaw = [0, 0, 0]';
                    Z = [Zyaw; Z];
                elseif obj.ob_Attitude == Utils.YAW_ATTITUDE %如果是航向角修正模式
                    Zyaw = 0;
                    Z = [Zyaw; Z];
                end
                if obj.ob_Position == Utils.VERTICAL_POSITION
                    Z = [Z; 0];
                end
            else
                obj.sup_ctr = obj.sup_ctr+1;     
                obj.ahrs.UpdateMIMU(obj.iHandler.mFSeq(:,index), obj.iHandler.mWSeq(:,index), obj.iHandler.mMSeq(:,index));
                obj.q = obj.ahrs.returnQ();
                obj.phi = obj.ahrs.returnPhi();
                obj.err_phi = mod(obj.phi-obj.Phi + 180,360)-180;
                if obj.ob_Attitude == Utils.WHOLE_ATTITUDE %如果是三轴姿态修正模式
                    Zyaw = (mod(obj.phi-obj.Phi + 180,360)-180)*pi/180;
                    Zyaw = [Zyaw(1),Zyaw(2),-Zyaw(3)]';
                    Z = [Zyaw; Z];
                elseif obj.ob_Attitude == Utils.YAW_ATTITUDE %如果是航向角修正模式
                    Zyaw = (mod(obj.phi(3)-obj.Phi(3) + 180,360)-180)*pi/180;
                    Z = [Zyaw; Z];
                end
                if obj.ob_Position == Utils.VERTICAL_POSITION
                    Z = [Z; -obj.P(3)];
                end
            end        
        end
        
        % @brief 初始化ARHS算法
        % @param None 
        % @retval None
        function initARHS(obj)
            if obj.sup_ctr == 0
                obj.sup_ctr = obj.sup_ctr+1;
                %初始化姿态参考系统
                if obj.ahrs_algorithm == Utils.AHRS_MADGWICK
                    obj.ahrs = AHRS(obj.Q, 'AHRS',obj.ahrs_algorithm, 'Madgwick_Beta',obj.Madgwick_Beta);
                elseif obj.ahrs_algorithm == Utils.AHRS_MAHONY
                    obj.ahrs = AHRS(obj.Q, 'AHRS',obj.ahrs_algorithm, 'Mahony_Kp',obj.Mahony_Kp, 'Mahony_Ki',obj.Mahony_Ki);
                elseif obj.ahrs_algorithm == Utils.AHRS_EKF
                    obj.ahrs = AHRS(obj.Q, 'AHRS',obj.ahrs_algorithm, 'EKF_noiseW',obj.EKF_noiseW,...
                                    'EKF_noiseWb',obj.EKF_noiseWb, 'EKF_noiseF',obj.EKF_noiseF, 'EKF_noiseM',obj.EKF_noiseM);
                end
            end
        end
        
    end
end