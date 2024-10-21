%MAYHONYAHRS Madgwick's implementation of Mayhony's AHRS algorithm
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
%
%   Date          Author          Notes
%   28/09/2011    SOH Madgwick    Initial release

classdef MahonyAHRS < handle  
    %% Public properties
    properties (Access = private)
        SamplePeriod = 0.006;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Kp = 1;                     % algorithm proportional gain
        Ki = 0;                     % algorithm integral gain
        eInt = [0 0 0];             % integral error
    end    
 
    %% Public methods
    methods (Access = public)
        
        % @brief Mahony 构造函数 
        % @param q 初始化姿态四元数表示(列向量)    varargin 可变变量（未设置的变量按默认值初始化）
        % @retval obj
        function obj = MahonyAHRS(q, varargin)
            %读取设置
            Mahony_setting = inputParser;
            addParameter(Mahony_setting, 'SamplePeriod', 0.006);    %采集间隔
            addParameter(Mahony_setting, 'Kp', 0.1);                %算法比例增益Kp
            addParameter(Mahony_setting, 'Ki', 0.0025);             %算法积分增益Ki
            parse(Mahony_setting, varargin{:});
            %初始化参数
            obj.Quaternion = q';
            obj.SamplePeriod = Mahony_setting.Results.SamplePeriod;
            obj.Kp = Mahony_setting.Results.Kp;
            obj.Ki = Mahony_setting.Results.Ki;
        end
        
        % @brief 更新姿态参考系统 
        % @param obj当前实例   Accelerometer 加速度(列向量，单位:g)   Gyroscope 角速度(列向量,单位:rad/s)   Magnetometer 磁力计(列向量,单位:uT)
        % @retval None
        function obj = UpdateMIMU(obj, Accelerometer, Gyroscope, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability
 
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
 
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
 
            % Reference direction of Earth's magnetic feild
            h = Utils.quaternProd(q, Utils.quaternProd([0 Magnetometer], Utils.quaternConj(q)));
            b = [0 0 norm([h(2) h(3)]) h(4)];
            
            % Estimated direction of gravity and magnetic field
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
            w = [2*b(3)*(q(1)*q(4) + q(2)*q(3)) + 2*b(4)*(q(2)*q(4) - q(1)*q(3));
                2*b(3)*(0.5 - q(2)^2 - q(4)^2) + 2*b(4)*(q(1)*q(2) + q(3)*q(4));
                2*b(3)*(q(3)*q(4) - q(1)*q(2)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 
 
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v) + cross(Magnetometer, w); 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * Utils.quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
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
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
 
            % Estimated direction of gravity and magnetic flux
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
 
            % Error is sum of cross product between estimated direction and measured direction of field
            e = cross(Accelerometer, v); 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        
        % @brief 返回参考姿态Q四元数表示
        % @param obj当前实例
        % @retval Q 四元数
        function Q = returnQ(obj)
            Q = obj.Quaternion';
        end
        
        % @brief 返回参考姿态Phi欧拉角表示
        % @param obj当前实例
        % @retval Phi 欧拉角
        function Phi = returnPhi(obj)
            Phi = Utils.getPhiFromQ(obj.Quaternion);
        end
    end
end