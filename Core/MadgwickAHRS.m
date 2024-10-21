%MADGWICKAHRS Implementation of Madgwick's IMU and AHRS algorithms
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
%
%   Date          Author          Notes
%   28/09/2011    SOH Madgwick    Initial release

classdef MadgwickAHRS < handle
    %% Public properties
    properties (Access = private)
        SamplePeriod = 0.006;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Beta = 1;               	% algorithm gain
    end

    %% Public methods
    methods (Access = public)
        
        % @brief Madgwick 构造函数 
        % @param q 初始化姿态四元数表示(列向量)    varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = MadgwickAHRS(q, varargin)
            %读取设置
            Madgwick_setting = inputParser;
            addParameter(Madgwick_setting, 'SamplePeriod', 0.006);  %采集间隔
            addParameter(Madgwick_setting, 'Beta', 0.002);          %算法增益beta
            parse(Madgwick_setting, varargin{:});
            %初始化参数
            obj.Quaternion = q';
            obj.SamplePeriod = Madgwick_setting.Results.SamplePeriod;
            obj.Beta = Madgwick_setting.Results.Beta;
        end
        
        % @brief 更新姿态参考系统 
        % @param obj当前实例   Accelerometer 加速度(列向量，单位:g)   Gyroscope 角速度(列向量,单位:rad/s)   Magnetometer 磁力计(列向量,单位:uT)
        % @retval None
        function obj = UpdateMIMU(obj, Accelerometer, Gyroscope, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end	% handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);	% normalise magnitude

            % Reference direction of Earth's magnetic feild
            h = Utils.quaternProd(q, Utils.quaternProd([0 Magnetometer], Utils.quaternConj(q)));
            b = [0 0 norm([h(2) h(3)]) h(4)];

            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
                2*b(3)*(q(1)*q(4) + q(2)*q(3)) + 2*b(4)*(q(2)*q(4) - q(1)*q(3));
                2*b(3)*(0.5 - q(2)^2 - q(4)^2) + 2*b(4)*(q(1)*q(2) + q(3)*q(4));
                2*b(3)*(q(3)*q(4) - q(1)*q(2)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)];
            J = [-2*q(3),                       2*q(4),                         -2*q(1),                            2*q(2)
                2*q(2),                         2*q(1),                         2*q(4),                             2*q(3)
                0,                              -4*q(2),                        -4*q(3),                            0
                2*b(3)*q(4) - 2*b(4)*q(3),      2*b(3)*q(3) + 2*b(4)*q(4),      2*b(3)*q(2) - 2*b(4)*q(1),          2*b(3)*q(1) + 2*b(4)*q(2);
                2*b(4)*q(2),                    -4*b(3)*q(2) + 2*b(4)*q(1),     2*b(4)*q(4),                        -4*b(3)*q(4) + 2*b(4)*q(3);
                -2*b(3)*q(2),                   -2*b(3)*q(1) - 4*b(4)*q(2) ,    2*b(3)*q(4) - 4*b(4)*q(3),          2*b(3)*q(3)];
            
            step = (J'*F);
            step = step / norm(step);	% normalise step magnitude

            % Compute rate of change of quaternion
            qDot = 0.5 * Utils.quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
        % @brief 更新姿态参考系统(仅使用IMU)  
        % @param obj当前实例   Accelerometer 加速度(列向量，单位:g)   Gyroscope 角速度(列向量,单位:rad/s)
        % @retval None
        function obj = UpdateIMU(obj, Accelerometer, Gyroscope)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude

            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];
            J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
                2*q(2),     2*q(1),     2*q(4),	2*q(3)
                0,         -4*q(2),    -4*q(3),	0    ];
            step = (J'*F);
            step = step / norm(step);	% normalise step magnitude

            % Compute rate of change of quaternion
            qDot = 0.5 * Utils.quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
        % @brief 返回参考姿态Q四元数表示
        % @param obj当前实例
        % @retval Q 当前姿态四元数表示(列向量)
        function Q = returnQ(obj)
            Q = obj.Quaternion';
        end
        
        % @brief 返回参考姿态Phi欧拉角表示
        % @param obj当前实例
        % @retval Phi 当前姿态欧拉角表示(列向量)
        function Phi = returnPhi(obj)
            Phi = Utils.getPhiFromQ(obj.Quaternion);
        end
        
    end
end