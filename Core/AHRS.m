classdef AHRS
    %航姿参考系统
    properties(Access = private)
        %姿态参考算法
        MadgwickAHRS;           %梯度下降算法
        MahonyAHRS;             %互补滤波算法
        EKFAHRS;                %扩展卡尔曼滤波
        Delta_t;                %采样间隔
        
        %姿态观测算法设置
        AHRS_Algorithm;  %AHRS采用算法
        
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
        
        % @brief AHRS构造函数 
        % @param varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = AHRS(q, varargin)
            %解析算法设置参数
            AHRS_setting = inputParser;
            %解析算法设置
            addParameter(AHRS_setting, 'AHRS', Utils.AHRS_EKF);         %默认启用EKF算法
            addParameter(AHRS_setting, 'SamplePeriod', 0.006);          %默认采集间隔为0.006s
            %读取EKF航位参考算法参数设置
            addParameter(AHRS_setting, 'EKF_noiseW', 0.02);             %预测噪声（陀螺仪噪声）
            addParameter(AHRS_setting, 'EKF_noiseWb', 0.0001);          %预测噪声（陀螺仪偏置噪声）
            addParameter(AHRS_setting, 'EKF_noiseF', 40);               %观测噪声（加速度计噪声）
            addParameter(AHRS_setting, 'EKF_noiseM', 50);               %观测噪声（磁力计噪声）
            %读取Mahony航位参考算法参数设置
            addParameter(AHRS_setting, 'Mahony_Kp', 0.01);              %算法比例增益Kp
            addParameter(AHRS_setting, 'Mahony_Ki', 0.0025);            %算法积分增益Ki
            %读取Madgwick航位参考算法参数设置
            addParameter(AHRS_setting, 'Madgwick_Beta', 0.005);         %算法增益beta
            %解析varargin变量
            parse(AHRS_setting, varargin{:});
            %初始化算法设置
            obj.AHRS_Algorithm = AHRS_setting.Results.AHRS;
            obj.Delta_t = AHRS_setting.Results.SamplePeriod;
            %初始化EKF航位参考算法参数
            obj.EKF_noiseW = AHRS_setting.Results.EKF_noiseW;
            obj.EKF_noiseWb = AHRS_setting.Results.EKF_noiseWb;
            obj.EKF_noiseF = AHRS_setting.Results.EKF_noiseF;
            obj.EKF_noiseM = AHRS_setting.Results.EKF_noiseM;
            %初始化Mahony航位参考算法参数
            obj.Mahony_Kp = AHRS_setting.Results.Mahony_Kp;
            obj.Mahony_Ki = AHRS_setting.Results.Mahony_Ki;
            %初始化Madgwick航位参考算法参数
            obj.Madgwick_Beta = AHRS_setting.Results.Madgwick_Beta;
            %如果采用Madgwick算法
            if obj.AHRS_Algorithm == Utils.AHRS_MADGWICK
                obj.MadgwickAHRS = MadgwickAHRS(q, 'SamplePeriod',obj.Delta_t, 'Beta',obj.Madgwick_Beta);
            %如果采用Mahony算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_MAHONY
                obj.MahonyAHRS = MahonyAHRS(q, 'SamplePeriod',obj.Delta_t, 'Kp',obj.Mahony_Kp, 'Ki',obj.Mahony_Ki);
            %如果采用扩展卡尔曼滤波算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_EKF
                obj.EKFAHRS = EKFAHRS(q, 'SamplePeriod',obj.Delta_t, 'noiseW',obj.EKF_noiseW, 'noiseWb',obj.EKF_noiseWb,...
                                      'noiseF',obj.EKF_noiseF, 'noiseM',obj.EKF_noiseM);
            else
                error('Invalid argument');
            end
                
        end
        
        % @brief AHRS更新 
        % @param obj, Gyro, Acc, Mag
        % @retval NONE
        function obj = UpdateMIMU(obj, Acc, Gyro, Mag)
            %如果采用Madgwick算法
            if obj.AHRS_Algorithm == Utils.AHRS_MADGWICK
                obj.MadgwickAHRS.UpdateMIMU(Acc./9.8', Gyro*pi/180', Mag');
            %如果采用Mahony算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_MAHONY
                obj.MahonyAHRS.UpdateMIMU(Acc./9.8', Gyro*pi/180', Mag');
            %如果采用扩展卡尔曼滤波算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_EKF  
                obj.EKFAHRS.UpdateMIMU(Acc, Gyro, Mag);
            else
                error('AHRS update error');
            end
        end
        
        % @brief 返回参考姿态Q四元数表示
        % @param obj 当前实例
        % @retval Q 四元数
        function Q = returnQ(obj)
            %如果采用Madgwick算法
            if obj.AHRS_Algorithm == Utils.AHRS_MADGWICK
                Q = obj.MadgwickAHRS.returnQ();
            %如果采用Mahony算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_MAHONY
                Q = obj.MahonyAHRS.returnQ();
            %如果采用扩展卡尔曼滤波算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_EKF
                Q = obj.EKFAHRS.returnQ();
            end
            if Q(1) < 0
                Q = -Q;
            end
        end
        
        % @brief 返回参考姿态Phi欧拉角表示
        % @param obj 当前实例
        % @retval Phi 欧拉角
        function Phi = returnPhi(obj)
            %如果采用Madgwick算法
            if obj.AHRS_Algorithm == Utils.AHRS_MADGWICK
                Phi = obj.MadgwickAHRS.returnPhi();
            %如果采用Mahony算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_MAHONY
                Phi = obj.MahonyAHRS.returnPhi();
            %如果采用扩展卡尔曼滤波算法
            elseif obj.AHRS_Algorithm == Utils.AHRS_EKF
                Phi = obj.EKFAHRS.returnPhi();
            end
        end
        
    end
    
    
end