 classdef Aligner
    %完成初始对准，包括粗对准和精对准
    properties(Access = private)
        %姿态参考算法
        MadgwickAHRS;           %梯度下降算法
        MahonyAHRS;             %互补滤波算法
        EKFAHRS;                %扩展卡尔曼滤波
        Delta_t;                %采样间隔
        
        %姿态观测算法设置
        Aligner_Algorithm;      %Aligner采用算法
        
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
        
        % @brief Aligner 对准器构造函数 
        % @param acc 加速度(列向量,单位:m/s²)    mag 磁力计(列向量,单位:uT)    varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = Aligner(acc, mag, varargin)
            %解析算法设置参数
            Aligner_setting = inputParser;
            %解析算法设置
            addParameter(Aligner_setting, 'Aligner', Utils.AHRS_MADGWICK);  %默认启用Madgwick算法
            addParameter(Aligner_setting, 'SamplePeriod', 0.006);           %默认采集间隔为0.006s
            %读取EKF航位参考算法参数设置
            addParameter(Aligner_setting, 'EKF_noiseW', 0.02);          %预测噪声（陀螺仪噪声）
            addParameter(Aligner_setting, 'EKF_noiseWb', 0.0001);       %预测噪声（陀螺仪偏置噪声）
            addParameter(Aligner_setting, 'EKF_noiseF', 50);            %观测噪声（加速度计噪声）
            addParameter(Aligner_setting, 'EKF_noiseM', 60);            %观测噪声（磁力计噪声）
            %读取Mahony航位参考算法参数设置
            addParameter(Aligner_setting, 'Mahony_Kp', 0.01);           %算法比例增益Kp
            addParameter(Aligner_setting, 'Mahony_Ki', 0.0025);         %算法积分增益Ki
            %读取Madgwick航位参考算法参数设置
            addParameter(Aligner_setting, 'Madgwick_Beta', 0.005);    %算法增益beta
            %解析varargin变量
            parse(Aligner_setting, varargin{:});
            %初始化算法设置
            obj.Aligner_Algorithm = Aligner_setting.Results.Aligner;
            obj.Delta_t = Aligner_setting.Results.SamplePeriod;
            %初始化EKF航位参考算法参数
            obj.EKF_noiseW = Aligner_setting.Results.EKF_noiseW;
            obj.EKF_noiseWb = Aligner_setting.Results.EKF_noiseWb;
            obj.EKF_noiseF = Aligner_setting.Results.EKF_noiseF;
            obj.EKF_noiseM = Aligner_setting.Results.EKF_noiseM;
            %初始化Mahony航位参考算法参数
            obj.Mahony_Kp = Aligner_setting.Results.Mahony_Kp;
            obj.Mahony_Ki = Aligner_setting.Results.Mahony_Ki;
            %初始化Madgwick航位参考算法参数
            obj.Madgwick_Beta = Aligner_setting.Results.Madgwick_Beta;
            %粗对准获取初始姿态
            q = obj.roughAlignment(acc, mag);
            %如果采用Madgwick算法
            if obj.Aligner_Algorithm == Utils.AHRS_MADGWICK
                obj.MadgwickAHRS = MadgwickAHRS(q, 'SamplePeriod',obj.Delta_t, 'Beta',obj.Madgwick_Beta);
            %如果采用Mahony算法
            elseif obj.Aligner_Algorithm == Utils.AHRS_MAHONY
                obj.MahonyAHRS = MahonyAHRS(q, 'SamplePeriod',obj.Delta_t, 'Kp',obj.Mahony_Kp, 'Ki',obj.Mahony_Ki);
            %如果采用扩展卡尔曼滤波算法
            elseif obj.Aligner_Algorithm == Utils.AHRS_EKF
                obj.EKFAHRS = EKFAHRS(q, 'SamplePeriod',obj.Delta_t, 'noiseW',obj.EKF_noiseW, 'noiseWb',obj.EKF_noiseWb,...
                                      'noiseF',obj.EKF_noiseF, 'noiseM',obj.EKF_noiseM);
            else
                error('Invalid argument');
            end
        end

        % @brief 姿态粗对准器 
        % @param acc 加速度(列向量,单位:m/s²)    mag 磁力计(列向量,单位:uT)
        % @retval Q 当前姿态四元数表示(列向量)
        function Q = roughAlignment(~, acc, mag)
            % 计算重力向量的倾斜角度
            roll = atan2(-acc(1), sqrt(acc(2)^2 + acc(3)^2));
            pitch = atan2(acc(2), acc(3));
            % 计算磁场向量的偏航角
            Mxb = mag(1)*cos(roll) + mag(2)*sin(pitch)*sin(roll) + mag(3)*cos(pitch)*sin(roll); 
            Myb = mag(2)*cos(pitch) - mag(3)*sin(pitch);
            yaw = atan2(double(Mxb), double(Myb));
            Q = Utils.getQFromPhi([pitch,roll,yaw]*180/pi);
        end
        
        
        % @brief 姿态精对准器更新 
        % @param acc 加速度(列向量,单位:m/s²)    gyro 角速度(列向量,单位:°/s)    mag 磁力计(列向量,单位:uT)
        % @retval Q 当前姿态四元数表示(列向量)
        function Q = accurateAlignment(obj, Acc, Gyro, Mag)
            %如果采用Madgwick算法
            if obj.Aligner_Algorithm == Utils.AHRS_MADGWICK
                obj.MadgwickAHRS.UpdateMIMU(Acc./9.8', Gyro*pi/180', Mag');
                Q = obj.MadgwickAHRS.returnQ();
            %如果采用Mahony算法
            elseif obj.Aligner_Algorithm == Utils.AHRS_MAHONY
                obj.MahonyAHRS.UpdateMIMU(Acc./9.8', Gyro*pi/180', Mag');
                Q = obj.MahonyAHRS.returnQ();
            %如果采用扩展卡尔曼滤波算法
            elseif obj.Aligner_Algorithm == Utils.AHRS_EKF
                obj.EKFAHRS.UpdateMIMU(Acc, Gyro, Mag);
                Q = obj.EKFAHRS.returnQ();
            end
            if Q(1) < 0
                Q = -Q;
            end
        end

    end
end