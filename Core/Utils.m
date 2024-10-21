classdef Utils <handle
    

    %静态常量
    properties(Constant)
        %常量定义
        ALIGNER_STAMP = 50;
        START_STAMP = 51;
        
        %IMU用
        Phase_Static = 0.3;%静止
        Phase_Active = 0.7;%活动

        %plantar用
        Phase_HeelStrike = 0.6;%脚跟着地
        Phase_ToeOff = 0.4;%脚尖离地

        Phase_Landing = 0.2;%脚着地
        Phase_Floating = 0.8;%脚离地浮空

        Phase_Unknown = 0.5;%未知情况

        %最终的
        Phase_Stance = 0;%静止
        Phase_Swing = 1;%运动
        
        %算法设置定义
        WHOLE_ATTITUDE = 2;     %三轴姿态修正
        YAW_ATTITUDE = 1;       %仅航向修正
        NONE_ATTITUDE = 0;      %不启用姿态修正
        
        VERTICAL_POSITION = 1;  %启用高度修正
        NONE_POSITION = 0;      %不启用位移修正
           
        AHRS_MAHONY = 0;        %Mahony算法
        AHRS_MADGWICK = 1;      %Madgwick算法
        AHRS_EKF = 2;           %扩展卡尔曼滤波算法
        
    end

    methods(Static)
        function obj = Utils()
        end
        
        % @brief 计算三维向量的反对称矩阵 
        % @param v 3维向量
        % @retval crossMatrix 反对称矩阵
        function crossMatrix = getCrossMatrix(v)
            crossMatrix = [0, -v(3), v(2);
                           v(3), 0, -v(1);
                           -v(2), v(1), 0];    
        end
        
        % @brief 从姿态矩阵解算欧拉角 
        % @param Cbn 旋转矩阵
        % @retval Phi 欧拉角（角度制）
        function Phi = getPhiFromCbn(Cbn)
            pitch = asin(Cbn(3,2));
            if  abs(Cbn(3,2)) <= 0.999999 
                roll = -atan2(Cbn(3,1),Cbn(3,3));
                yaw = -atan2(Cbn(1, 2), Cbn(2, 2));     
            elseif abs(Cbn(3,2)) > 0.999999 
                roll = atan2(Cbn(1,3),Cbn(1,1));
                yaw = 0;
            end
            Phi = [pitch, roll, yaw]'*180/pi;
        end

        % @brief 从欧拉角计算姿态矩阵 
        % @param Phi 欧拉角（角度制）
        % @retval Cbn 旋转矩阵
        function  Cbn = getCbnFromPhi(phi)
            phi =  phi *pi/180;     %角度制转弧度制
            pitch = phi(1);         %横滚角roll
            roll = phi(2);         %俯仰角pitch
            yaw = phi(3);           %翻滚角yaw
            Cbn = [cos(yaw)*cos(roll)-sin(yaw)*sin(pitch)*sin(roll), -sin(yaw)*cos(pitch), cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
                sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll), cos(yaw)*cos(pitch), sin(yaw)*sin(roll)-cos(yaw)*sin(pitch)*cos(roll);
                -cos(pitch)*sin(roll), sin(pitch), cos(pitch)*cos(roll)];
        end
        
        % @brief 从欧拉角计算四元数
        % @param Phi 欧拉角（角度制）
        % @retval Q 四元数[w x y z]'
        function  Q = getQFromPhi(phi)
            phi =  phi *pi/180;     %角度制转弧度制   
            pitch = phi(1);         %俯仰角pitch
            roll = phi(2);         %横滚角roll
            yaw = phi(3);           %翻滚角yaw
            Q = [cos(yaw/2)*cos(pitch/2)*cos(roll/2)-sin(yaw/2)*sin(pitch/2)*sin(roll/2);
                cos(yaw/2)*sin(pitch/2)*cos(roll/2)-sin(yaw/2)*cos(pitch/2)*sin(roll/2);
                sin(yaw/2)*sin(pitch/2)*cos(roll/2)+cos(yaw/2)*cos(pitch/2)*sin(roll/2);
                sin(yaw/2)*cos(pitch/2)*cos(roll/2)+cos(yaw/2)*sin(pitch/2)*sin(roll/2)];
            if Q(1) < 0
                Q = -Q;
            end
            Q = Utils.normalizeQ(Q);
        end
        
        % @brief 从四元数计算欧拉角
        % @param Q 四元数[w x y z]'
        % @retval Phi 欧拉角（角度制）
         function  Phi = getPhiFromQ(Q)
            [q0, q1, q2, q3] = deal(Q(1), Q(2), Q(3), Q(4));
            pitch = asin(2*(q2*q3+q0*q1));
            if  abs(2*(q2*q3+q0*q1)) <= 0.999999 
                roll = -atan2(2*(q1*q3-q0*q2),q0^2-q1^2-q2^2+q3^2);
                yaw = -atan2(2*(q1*q2-q0*q3),q0^2-q1^2+q2^2-q3^2);
            elseif abs(2*(q2*q3+q0*q1)) > 0.999999 
                roll = atan2(2*(q1*q3+q0*q2),q0^2+q1^2-q2^2-q3^2);
                yaw = 0;
            end
            Phi = [pitch, roll, yaw]'*180/pi;
         end
         
         
        % @brief 从四元数计算旋转矩阵
        % @param Q 四元数[w x y z]'
        % @retval Cbn 旋转矩阵
        function  Cbn = getCbnFromQ(Q)
            [q0, q1, q2, q3] = deal(Q(1), Q(2), Q(3), Q(4));
            Cbn = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
                2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1); 
                2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
        end
        
        % @brief 从旋转矩阵计算四元数
        % @param Cbn 旋转矩阵
        % @retval Q 四元数[w x y z]'
        function  Q = getQFromCbn(Cbn)
            if (Cbn(1,1) >= Cbn(2,2)+Cbn(3,3))
                q1 = 0.5*sqrt(1+Cbn(1,1)-Cbn(2,2)-Cbn(3,3));
                q0 = (Cbn(3,2)-Cbn(2,3))/(4*q1);
                q2 = (Cbn(1,2)+Cbn(2,1))/(4*q1);
                q3 = (Cbn(1,3)+Cbn(3,1))/(4*q1);
            elseif (Cbn(2,2) >= Cbn(1,1)+Cbn(3,3))
                q2 = 0.5*sqrt(1-Cbn(1,1)+Cbn(2,2)-Cbn(3,3));
                q0 = (Cbn(1,3)-Cbn(3,1))/(4*q2);
                q1 = (Cbn(1,2)+Cbn(2,1))/(4*q2);
                q3 = (Cbn(2,3)+Cbn(3,2))/(4*q2);
            elseif (Cbn(3,3) >= Cbn(1,1)+Cbn(2,2))
                q3 = 0.5*sqrt(1-Cbn(1,1)-Cbn(2,2)+Cbn(3,3));
                q0 = (Cbn(2,1)-Cbn(1,2))/(4*q3);
                q1 = (Cbn(1,3)+Cbn(3,1))/(4*q3);
                q2 = (Cbn(2,3)+Cbn(3,2))/(4*q3);
            else
                q0 = 0.5*sqrt(1+Cbn(1,1)+Cbn(2,2)+Cbn(3,3));
                q1 = (Cbn(3,2)-Cbn(2,3))/(4*q0);
                q2 = (Cbn(1,3)-Cbn(3,1))/(4*q0);
                q3 = (Cbn(2,1)-Cbn(1,2))/(4*q0);
            end
            Q = [q0, q1, q2, q3]';
            if Q(1) < 0
                Q = -Q;
            end
            Q = Utils.normalizeQ(Q);
        end
        
        % @brief 返回四元数乘积
        % @param a b 
        % @retval ab 乘积
        function ab = quaternProd(a, b)
            ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
            ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
            ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
            ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
        end
        
        % @brief 返回四元数共轭
        % @param q 四元数
        % @retval qConj 四元数共轭
        function qConj = quaternConj(q)
            qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
        end
        
        % @brief 四元数单位化 
        % @param q 四元数
        % @retval q_norm 单位化四元数
        function q_norm = normalizeQ(q)
            q_norm = q / norm(q);
        end

    end
end