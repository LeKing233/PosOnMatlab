classdef EKFStateCalculator <handle
    properties(Access = public)
        mStateSeq;      %状态向量序列
        iHandler;       %imu数据管理类
        pHandler;       %足底压力传感器数据管理类
    end
         
    properties(Access = private)
        %时间戳变量
        TimeStamp;      %时间戳
        SupTime;        %支撑相时刻
        
        %状态变量
        Acc;            %东北天坐标系下比力
        Wb;             %陀螺仪偏置
        Q;              %姿态四元数表示
        Phi;            %姿态欧拉角表示
        Cbn;            %姿态旋转矩阵表示
        V;              %航位推算速度
        P;              %航位推算位移
        Err_G;          %重力加速度观测误差
        Err_M;          %地磁场观测误差
        Err_V;          %速度观测误差
        Rk_G;           %重力加速度检测值
        Rk_M;           %地磁场检测值
        %步态划分
        GaitPhase;      %步态相位
   
        %常量
        delta_t = 0.006;        %更新间隔 (秒)
        gravity = 9.8;          %重力加速度数值(米每平方秒)
        gravityVec;             %重力加速度向量

        %核心组件
        zuptEKF;                %零速度修正扩展卡尔曼滤波器
        aligner;                %姿态初始对准器
        
        %初始对准算法设置
        aligner_algorithm;      %Aligner采用算法
        
        %ZUPEKF算法参数
        ZEKF_noiseW;            %预测噪声（陀螺仪噪声）
        ZEKF_noiseWb;           %预测噪声（陀螺仪偏置噪声）
        ZEKF_noiseF;            %预测噪声（加速度计噪声）
        ZEKF_noiseG;            %观测噪声噪声（互补滤波重力加速度噪声）
        ZEKF_noiseM;            %观测噪声（磁力计噪声）
        ZEKF_noiseV;            %观测噪声（速度噪声）
        ZEKF_noiseH;            %观测噪声（高度计计噪声）
        ZEKF_fadeQ;             %姿态渐消记忆因子
        ZEKF_fadeWb;            %陀螺仪偏置渐消记忆因子
        ZEKF_fadeV;             %速度渐消记忆因子
        ZEKF_fadeP;             %位移渐消记忆因子
        
        %EKF航位参考算法参数
        EKF_noiseW;             %预测噪声（陀螺仪噪声）
        EKF_noiseWb;            %预测噪声（陀螺仪偏置噪声）
        EKF_noiseF;             %观测噪声（加速度计噪声）
        EKF_noiseM;             %观测噪声（磁力计噪声）
        
        %Mahony航位参考算法参数
        Mahony_Kp;              %算法比例增益Kp
        Mahony_Ki;              %算法积分增益Ki
        
        %Madgwick航位参考算法参数
        Madgwick_Beta;          %算法增益beta
    end
    
    methods(Access = public)
        
        % @brief EKFStateCalculator构造函数 
        % @param imuHandler IMU数据处理示例   plantarHandler 压力传感器数据处理示例
        % @param varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = EKFStateCalculator(imuHandler,plantarHandler,varargin)          
            %常量初始化
            obj.gravityVec = [0;0;obj.gravity];
            %构建核心组件
            obj.iHandler = imuHandler;
            obj.pHandler = plantarHandler;
            %解析算法设置
            Algorithm_setting = inputParser;
            addParameter(Algorithm_setting, 'Aligner', Utils.AHRS_MADGWICK);              %默认启用梯度下降算法
            %读取ZUPEKF参数设置
            addParameter(Algorithm_setting, 'ZEKF_noiseW', 0.002);          %预测噪声（陀螺仪噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseWb', 0.00000002);    %预测噪声（陀螺仪偏置噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseF', 0.0006);         %预测噪声（加速度计噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseG', 0.016);          %观测噪声噪声（互补滤波重力加速度噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseM', 0.018);          %观测噪声（磁力计噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseV', 0.016);          %观测噪声（速度噪声）
            addParameter(Algorithm_setting, 'ZEKF_noiseH', 0.0005);         %观测噪声（高度计计噪声）
            addParameter(Algorithm_setting, 'ZEKF_fadeQ', 1.0005);          %姿态渐消记忆因子
            addParameter(Algorithm_setting, 'ZEKF_fadeWb', 1.000005);       %陀螺仪偏置渐消记忆因子
            addParameter(Algorithm_setting, 'ZEKF_fadeV', 1.000005);        %速度渐消记忆因子
            addParameter(Algorithm_setting, 'ZEKF_fadeP', 1.00005);         %位移渐消记忆因子
            %读取Mahony航位参考算法参数设置
            addParameter(Algorithm_setting, 'Mahony_Kp', 0.01);             %算法比例增益Kp
            addParameter(Algorithm_setting, 'Mahony_Ki', 0.0025);           %算法积分增益Ki
            %读取Madgwick航位参考算法参数设置
            addParameter(Algorithm_setting, 'Madgwick_Beta', 0.2);          %算法增益beta
            %解析varargin变量
            parse(Algorithm_setting, varargin{:});
            %初始化算法设置
            obj.aligner_algorithm = Algorithm_setting.Results.Aligner;
            %初始化ZUPEKF参数
            obj.ZEKF_noiseW = Algorithm_setting.Results.ZEKF_noiseW;
            obj.ZEKF_noiseWb = Algorithm_setting.Results.ZEKF_noiseWb;
            obj.ZEKF_noiseF = Algorithm_setting.Results.ZEKF_noiseF;
            obj.ZEKF_noiseG = Algorithm_setting.Results.ZEKF_noiseG;
            obj.ZEKF_noiseM = Algorithm_setting.Results.ZEKF_noiseM;
            obj.ZEKF_noiseV = Algorithm_setting.Results.ZEKF_noiseV;
            obj.ZEKF_noiseH = Algorithm_setting.Results.ZEKF_noiseH;
            obj.ZEKF_fadeQ = Algorithm_setting.Results.ZEKF_fadeQ;
            obj.ZEKF_fadeWb = Algorithm_setting.Results.ZEKF_fadeWb;
            obj.ZEKF_fadeV = Algorithm_setting.Results.ZEKF_fadeV;
            obj.ZEKF_fadeP = Algorithm_setting.Results.ZEKF_fadeP;
            %初始化Mahony航位参考算法参数
            obj.Mahony_Kp = Algorithm_setting.Results.Mahony_Kp;
            obj.Mahony_Ki = Algorithm_setting.Results.Mahony_Ki;
            %初始化Madgwick航位参考算法参数
            obj.Madgwick_Beta = Algorithm_setting.Results.Madgwick_Beta;
            %初始化序列
            obj.initStateSeq();
        end
        
        % @brief 状态解算
        % @param gaitDetectType
        % @retval None
        function result = solveState(obj,gaitDetectType)
            %初始姿态精对准
            init_q = obj.initAligner();
            %ZupEKF算法初始化
            obj.EKFStateinit(init_q, obj.iHandler.mFSeq(:,Utils.ALIGNER_STAMP));
            %序列化迭代计算
            for i = Utils.START_STAMP:length(obj.iHandler.mTSeq)
                %计算当前时刻的状态量
                obj.zuptEKF.predict(obj.iHandler.mFSeq(:,i), obj.iHandler.mWSeq(:,i));   %ekf预测
                
                %计算步态
                obj.GaitPhase = obj.iHandler.getPhase(i);

                %依据步态执行零速度更新
                %零速度修正
                if obj.GaitPhase == Utils.Phase_Swing 
                    
                elseif obj.GaitPhase == Utils.Phase_Stance
                    %支撑相
                    obj.zuptEKF.update(obj.iHandler.mFSeq(:,i), obj.iHandler.mMSeq(:,i));     %ekf更新
                    %获取观测误差
                    obj.Err_G = obj.zuptEKF.returnEg();
                    obj.Err_M = obj.zuptEKF.returnEm();
                    obj.Err_V = obj.zuptEKF.returnEv(); 
                    obj.Rk_G = obj.zuptEKF.returnRg();
                    obj.Rk_M = obj.zuptEKF.returnRm();
                    
                elseif  obj.GaitPhase == Utils.Phase_Unknown %计算错误
                    error('Invalid GaitPhase');
                end
                
                %获取计算结果
                obj.Acc = obj.zuptEKF.returnFc();
                obj.Wb = obj.zuptEKF.returnWb();
                obj.Q = obj.zuptEKF.returnQ();
                obj.Phi = obj.zuptEKF.returnPhi();
                obj.Cbn = Utils.getCbnFromQ(obj.Q);
                obj.V = obj.zuptEKF.returnV();
                obj.P = obj.zuptEKF.returnP();
                 
                %将计算结果保存
                obj.saveStateToSeq(i);
                
            end
            
            %打标签
            obj.mStateSeq.Legend = gaitDetectType;
            %将结果序列作为返回值返回
            obj.stateSeqToMat();
            result = obj.mStateSeq; 
        end
        
        
    end
    
    methods(Access = private)
        
        % @brief 状态初始化
        % @param q_init 初始姿态角       acc_init 初始加速度
        % @retval None
        function EKFStateinit(obj, q_init, acc_init)
            %初始化姿态
            obj.Q = q_init;
            obj.Phi = Utils.getPhiFromQ(obj.Q);
            obj.Cbn = Utils.getCbnFromQ(obj.Q);
            %初始化比力
            fc = Utils.quaternProd(Utils.quaternProd(q_init', [0; acc_init]'), Utils.quaternConj(q_init'))';
            obj.Acc = fc(2:4) - obj.gravityVec;
            %初始化ZupEKF及相关状态量
            obj.zuptEKF = ZupEKF(obj.Q, 'noiseW',obj.ZEKF_noiseW, 'noiseWb',obj.ZEKF_noiseWb, 'noiseF',obj.ZEKF_noiseF, 'noiseG',obj.ZEKF_noiseG, ...
                                'noiseM',obj.ZEKF_noiseM, 'noiseV',obj.ZEKF_noiseV, 'noiseH',obj.ZEKF_noiseH, ...
                                'fadeQ', obj.ZEKF_fadeQ, 'fadeWb', obj.ZEKF_fadeWb, 'fadeV', obj.ZEKF_fadeV, 'fadeP', obj.ZEKF_fadeP);
            obj.Wb = obj.zuptEKF.returnWb();
            obj.V = obj.zuptEKF.returnV();
            obj.P = obj.zuptEKF.returnP();
            obj.Err_G = zeros(3,1);
            obj.Err_M = zeros(3,1);
            obj.Err_V = zeros(3,1);
            obj.Rk_G = 0;
            obj.Rk_M = 0;
            obj.GaitPhase = Utils.Phase_Unknown;
            %保存初始状态到序列
            obj.saveStateToSeq(1);
        end
        
        % @brief 初始化姿态精对准
        % @param None 
        % @retval Q 初始姿态的四元数表示
        function Q = initAligner(obj)
           %初始化姿态对准器
            if obj.aligner_algorithm == Utils.AHRS_MADGWICK
                obj.aligner = Aligner(obj.iHandler.mFSeq(:,1)', obj.iHandler.mMSeq(:,1), 'Aligner',obj.aligner_algorithm,...
                                      'Madgwick_Beta',obj.Madgwick_Beta);
            elseif obj.aligner_algorithm == Utils.AHRS_MAHONY
                obj.aligner = Aligner(obj.iHandler.mFSeq(:,1)', obj.iHandler.mMSeq(:,1), 'Aligner',obj.aligner_algorithm,...
                                      'Mahony_Kp',obj.Mahony_Kp, 'Mahony_Ki',obj.Mahony_Ki);
            else
                error('Invalid Algorithm');
            end
            %姿态对准器迭代
            for i = 2:Utils.ALIGNER_STAMP
                obj.Q = obj.aligner.accurateAlignment(obj.iHandler.mFSeq(:,i), obj.iHandler.mWSeq(:,i), obj.iHandler.mMSeq(:,i));
                obj.Phi  = Utils.getPhiFromQ(obj.Q);
                obj.mStateSeq.Aligner_Q.addOne(obj.Q');
                obj.mStateSeq.Aligner_Phi.addOne(obj.Phi');
            end
            disp("初始姿态");
            disp(obj.Phi);
            Q = obj.Q;
        end
        
        % @brief 初始化储存序列
        % @param None
        % @retval None 
        function  initStateSeq(obj)          
            obj.mStateSeq = struct( 'Acc',MatList(), 'Wb',MatList(), 'Q',MatList(), 'Phi',MatList(), 'Cbn',MatList(), ...
                                    'V',MatList(),'P',MatList(), 'GaitPhase',MatList(),...
                                    'Err_G',MatList(), 'Err_M',MatList(), 'Err_V',MatList(), ...
                                    'SupTime',MatList(), 'Rk_G',MatList(), 'Rk_M',MatList(), ...
                                    'Aligner_Q',MatList(), 'Aligner_Phi',MatList() ...
                                    );
        end
        
        % @brief 将结果保存进序列
        % @param None
        % @retval None 
        function saveStateToSeq(obj, index)
            %保存状态量
            obj.mStateSeq.Acc.addOne(obj.Acc');
            obj.mStateSeq.Wb.addOne(obj.Wb');
            obj.mStateSeq.Q.addOne(obj.Q');
            obj.mStateSeq.Phi.addOne(obj.Phi');
            obj.mStateSeq.Cbn.addOne(obj.Cbn);
            obj.mStateSeq.V.addOne(obj.V');
            obj.mStateSeq.P.addOne(obj.P');
            %步态检测参数
            obj.mStateSeq.GaitPhase.addOne(obj.GaitPhase);
            if obj.GaitPhase == Utils.Phase_Stance
                %保存支撑相时间戳
                obj.mStateSeq.SupTime.addOne(obj.iHandler.mTSeq(index));
                %保存观测量误差
                obj.mStateSeq.Err_G.addOne(obj.Err_G');
                obj.mStateSeq.Err_M.addOne(obj.Err_M');
                obj.mStateSeq.Err_V.addOne(obj.Err_V');
                obj.mStateSeq.Rk_G.addOne(obj.Rk_G);
                obj.mStateSeq.Rk_M.addOne(obj.Rk_M);
            end 
        end
        
        % @brief 将结果保存为Mat
        % @param None
        % @retval None
        function stateSeqToMat(obj)          
            %保存状态量
            obj.mStateSeq.Acc = obj.mStateSeq.Acc.toMat();
            obj.mStateSeq.Wb = obj.mStateSeq.Wb.toMat();
            obj.mStateSeq.Q = obj.mStateSeq.Q.toMat();
            obj.mStateSeq.Phi = obj.mStateSeq.Phi.toMat();
            obj.mStateSeq.Cbn = obj.mStateSeq.Cbn.toMat() ;
            obj.mStateSeq.V = obj.mStateSeq.V.toMat();
            obj.mStateSeq.P = obj.mStateSeq.P.toMat();
            obj.mStateSeq.Aligner_Q = obj.mStateSeq.Aligner_Q.toMat();
            obj.mStateSeq.Aligner_Phi = obj.mStateSeq.Aligner_Phi.toMat();
            %步态检测参数
            obj.mStateSeq.GaitPhase =  obj.mStateSeq.GaitPhase.toMat();
            obj.mStateSeq.SupTime = obj.mStateSeq.SupTime.toMat();
            %保存观测量误差
            obj.mStateSeq.Err_G = obj.mStateSeq.Err_G.toMat();
            obj.mStateSeq.Err_M = obj.mStateSeq.Err_M.toMat();
            obj.mStateSeq.Err_V = obj.mStateSeq.Err_V.toMat();
            obj.mStateSeq.Rk_G = obj.mStateSeq.Rk_G.toMat();
            obj.mStateSeq.Rk_M = obj.mStateSeq.Rk_M.toMat();
        end

    end
    
end