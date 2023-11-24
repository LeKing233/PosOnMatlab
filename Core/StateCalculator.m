classdef StateCalculator <handle
    
    properties
        %状态变量
        V;%速度向量
        P;%位移向量
        Cbn;%姿态矩阵
        Phi;%欧拉角
        GaitPhase;%步态相位
        mStateSeq;%状态向量序列
        
        %原始数据序列
        wSeq;%角速度序列
        fSeq;%加速度序列

        

        %常量
        delta_t = 0.006;    %更新间隔 (秒)
        gravity = 9.8;      %重力加速度数值(米每平方秒)
        gravityVec;      %重力加速度向量

        %核心组件
        iHandler;%imu数据管理类
        pHandler;%足底压力传感器数据管理类
        zuptKF;%卡尔曼滤波器（零速度修正特殊版本）
        aligner;%姿态初始对准器
        utils;%工具函数集合
    end

    methods
       %构造函数
       function obj = StateCalculator(imuHandler,plantarHandler)
            %常量初始化
            obj.gravityVec = [0;0;obj.gravity];
            %构建核心组件
            obj.iHandler = imuHandler;
            obj.pHandler = plantarHandler;
            obj.zuptKF  = ZuptKalmanFilter();  %卡尔曼滤波器
            obj.aligner = Aligner();           %姿态对准器
            obj.utils   = Utils();     %工具函数集
       end

       %状态初始化
       function stateInit(obj,phi0)
            obj.Phi = phi0;
            obj.V = zeros(3,1);
            obj.P = zeros(3,1);
            obj.Cbn = obj.getCbnFromPhi(obj.Phi);
            obj.zuptKF.init();%滤波器初始化 
            obj.GaitPhase = Utils.Phase_Unknown;
            obj.initStateSeq();
            obj.saveStateToSeq();

       end
       
        
       function  initStateSeq(obj)          
            obj.mStateSeq = struct(  'V',MatList(),'P',MatList(),'Cbn',MatList(),'Phi',MatList(),...
                                    'GaitPhase',MatList(), ...                                    
                                    'KF_X',MatList(), 'KF_P',MatList(),'KF_K',MatList()...
                                 );
       end


       %将结果保存进序列
       function saveStateToSeq(obj)
            %状态量
            obj.mStateSeq.V.addOne(obj.V);
            obj.mStateSeq.P.addOne(obj.P);
            obj.mStateSeq.Phi.addOne(obj.Phi);
            obj.mStateSeq.Cbn.addOne([zeros(1,3);obj.Cbn]);
            %步态检测参数
            obj.mStateSeq.GaitPhase.addOne(obj.GaitPhase);
            %kalman滤波器参数
            obj.mStateSeq.KF_X.addOne(obj.zuptKF.X);
            obj.mStateSeq.KF_P.addOne([9999*ones(1,9);obj.zuptKF.P]);
            obj.mStateSeq.KF_K.addOne([9999*ones(1,3);obj.zuptKF.K]);
       end
       
       %状态解算
       function solveState(obj) 

            %结果赋初值
            obj.wSeq = obj.iHandler.mWSeq;
            obj.fSeq = obj.iHandler.mFSeq;
            phi0 = [obj.iHandler.mRawData.AngleRoll(1);
                    obj.iHandler.mRawData.AnglePitch(1);
                    obj.iHandler.mRawData.AngleYaw(1)];
            obj.stateInit(phi0);%状态初始化
                
            %序列化迭代计算
            for i = 2:length(obj.wSeq)
                %记录上一时刻状态量
                Cbn_prev = obj.Cbn;
                V_prev = obj.V;
                P_prev = obj.P;

                %计算当前时刻的状态量
                obj.calculateNextCbn(Cbn_prev,obj.wSeq(:,i));
                obj.calculateNextV(V_prev,obj.fSeq(:,i),obj.Cbn,obj.fSeq(:,i-1),Cbn_prev);
                obj.calculateNextP(P_prev,obj.V,V_prev);
                obj.Phi = obj.getPhiFromCbn(obj.Cbn); 
                
                %计算步态
                obj.GaitPhase = obj.getGaitPhase(i);
                %依据步态执行零速度更新
                %零速度修正
                if obj.GaitPhase == Utils.Phase_Swing 
                     %摆动相
                    obj.zuptKF.X = zeros(9,1); %误差项置零       
                elseif obj.GaitPhase == Utils.Phase_Stance
                    %支撑相
                    %预测
                    F = obj.zuptKF.getF(obj,obj.Cbn,obj.fSeq(:,i));
                    G = obj.zuptKF.getG(obj,obj.Cbn);
                    obj.zuptKF.predict(F,G);
                    %更新
                    Z = -obj.V;
                    obj.zuptKF.update(Z);
                    %修正
                    obj.zuptKF.reviseVn(obj,obj.zuptKF.X(4:6));
                    obj.zuptKF.revisePn(obj,obj.zuptKF.X(7:9));
                    obj.zuptKF.reviseCbn(obj,obj.zuptKF.X(1:3));
                    obj.Phi = obj.getPhiFromCbn(obj.Cbn); 
            
                 elseif  obj.GaitPhase == Utils.Phase_Unknown %计算错误
                    
                end                
                %将计算结果保存
                obj.saveStateToSeq();
            end

       end


       %步态检测
       function phase = getGaitPhase(obj,index)
            phaseIMU = obj.iHandler.getPhase(index);
            phasePlantar = obj.pHandler.getGaitPhase(index);
            
            %方法一：将两者相位取"与"运算
%             if (phaseIMU==Utils.Phase_Static) && (phasePlantar == Utils.Phase_Landing)
%                 phase = Utils.Phase_Stance;
%             else
%                 phase = Utils.Phase_Swing;
%             end

            %方法二：以Plantar为准
            if (phasePlantar==Utils.Phase_Landing)
                phase = Utils.Phase_Stance;
            else
                phase = Utils.Phase_Swing;
            end
            %方法三：以IMU为准
%             if (phaseIMU==Utils.Phase_Static)
%                 phase = Utils.Phase_Stance;
%             else
%                 phase = Utils.Phase_Swing;
%             end

       end


       %从姿态矩阵解算欧拉角 
       function Phi = getPhiFromCbn(obj,Cbn)
            %横滚角Roll
            T33  = Cbn(3,3);
            T32 = Cbn(3,2);
            roll = atan(Cbn(3,2)/Cbn(3,3));
            %角度转换至[-pi,pi]
            if(T33>0)
                
            else 
                if(T32>0)
                    roll =  pi + roll;
                else 
                    roll =  -pi + roll;
                end 
            end
            Phi(1) = roll;
            %俯仰角Pitch  
            Phi(2) = asin(-Cbn(3,1));
            %航向角yaw
            yaw = atan(Cbn(2,1)/Cbn(1,1));
            T11 = Cbn(1,1);
            T21 = Cbn(2,1);   
            %角度转换至[-pi,pi]
            if(T11<0&&T21<0)
                yaw = yaw - pi;
            end
            if(T11<0&&T21>0)
                yaw =  yaw + pi;
            end
            Phi(3) = yaw;
            
            Phi = Phi'*180/pi;%转置成列向量
        end

       %从欧拉角计算姿态矩阵
       function  Cbn = getCbnFromPhi(obj,phi)
            phi =  phi *pi/180;%弧度制转角度值
            gamma = phi(1);%横滚角roll
            theta = phi(2);%俯仰角pitch
            psi = phi(3);%翻滚角yaw
            
            Cbn = [cos(theta)*cos(psi),-cos(gamma)*sin(psi)+sin(gamma)*sin(theta)*cos(psi),sin(gamma)*sin(psi)+cos(gamma)*sin(theta)*cos(psi);
                   cos(theta)*sin(psi),cos(gamma)*cos(psi)+sin(gamma)*sin(theta)*sin(psi),-sin(gamma)*cos(psi)+cos(gamma)*sin(theta)*sin(psi);
                   -sin(theta),sin(gamma)*cos(theta),cos(gamma)*cos(theta)];
        end
        
        %姿态矩阵计算——二子样算法
        function  calculateNextCbn(obj,Cbn_prev,w)
            w = w * pi /180;%弧度制转角度制
            I = eye(3);
            wx = obj.utils.getCrossMatrix(w);%角速度w构成的反对称矩阵[w×]
            obj.Cbn = Cbn_prev * ((2*I + wx* obj.delta_t)/(2*I - wx*obj.delta_t));
        end
        
        %计算下一时刻的速度
        function calculateNextV(obj,V_prev,f,Cbn,f_prev,Cbn_prev)
            obj.V = V_prev + (0.5*(Cbn*f+Cbn_prev*f_prev) - obj.gravityVec ) * obj.delta_t;
        end
        
        %计算下一时刻的位移
        function calculateNextP(obj,Pn_prev,Vn,Vn_prev)
            obj.P = Pn_prev + 0.5*(Vn_prev + Vn) * obj.delta_t;
        end
        


    end


    



        %% 绘图
   methods
        %三轴速度
        function plot_Velocity(obj)
            gcf = figure("Name","StateCalculator");
            Vel = obj.mStateSeq.V.toMat();
            plot(obj.iHandler.mRawData.Timestamp,Vel(1,:));hold on
            plot(obj.iHandler.mRawData.Timestamp,Vel(2,:));hold on
            plot(obj.iHandler.mRawData.Timestamp,Vel(3,:));hold on
            xlabel('时间戳'); % x轴注解
            ylabel('速度'); % y轴注解
            title('三轴速度曲线'); % 图形标题
            legend('X轴速度','Y轴速度','Z轴速度'); % 图形注解
            grid on; % 显示格线
        end

        %X、Y轴速度
        function plot_HorizontalVelXY(obj)
            gcf = figure("Name","StateCalculator");
            Vel = obj.mStateSeq.V.toMat();
            plot(obj.iHandler.mRawData.Timestamp,Vel(1,:));hold on
            plot(obj.iHandler.mRawData.Timestamp,Vel(2,:));
            xlabel('时间戳'); % x轴注解
            ylabel('速度'); % y轴注解
            title('XY方向速度曲线——修正后'); % 图形标题
            legend('X轴速度','Y轴速度'); % 图形注解
            grid on; % 显示格线
        end   



         %水平方向速度模长
         function plot_HorizontalVelNorm(obj)
            gcf = figure("Name","StateCalculator");
            Vel = obj.mStateSeq.V.toMat();
            velNormVec = zeros(size(Vel,2),1);
            for i = 1:size(Vel,2)
                velNormVec(i) = norm([Vel(1,i),Vel(2,i)]);%求取x方向和y方向速度模长
            end
            plot(obj.iHandler.mRawData.Timestamp,velNormVec);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('速度模长/m/s'); % y轴注解
            title('水平方向速度模长曲线'); % 图形标题
            grid on; % 显示格线
         end

         %IMU步态检测结果
         function plot_IMUGaitSeq(obj)
            gaitSeq = obj.mStateSeq.GaitPhase.toMat();
            gcf = figure("Name","StateCalculator");
            plot(obj.iHandler.mRawData.Timestamp,gaitSeq);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态'); % y轴注解
            title('步态检测结果'); % 图形标题
            grid on; % 显示格线
            legend('0-支撑相，1-摆动相，-1-未知结果');
         end

         %融合IMU步态检测结果和COP速度
         function plot_IMUGait_AND_COPVel(obj)
            gaitSeq = obj.mStateSeq.GaitPhase.toMat();
            COPSeq = obj.pHandler.mCOPSeq/100;
            COPVelSeq = obj.pHandler.mCOPVelSeq/100;
            gcf = figure("Name","StateCalculator");
            plot(obj.iHandler.mRawData.Timestamp,gaitSeq,LineWidth=2);hold on;
            plot(obj.iHandler.mRawData.Timestamp,COPSeq(:,1));hold on;
            plot(obj.iHandler.mRawData.Timestamp,COPSeq(:,2));hold on;
            plot(obj.iHandler.mRawData.Timestamp,COPVelSeq(:,1));hold on;
            plot(obj.iHandler.mRawData.Timestamp,COPVelSeq(:,2));hold on;
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('IMU和Plantar步态检测结果汇总图'); % 图形标题
            grid on; % 显示格线
            legend('0-支撑相，1-摆动相，-1-未知结果','COP_X','COP_Y','COPVel_X','COPVel_Y');
         end

         %IMU步态检测结果和Plantar共同显示
         function plot_IMUGait_AND_Plantar(obj)
            obj.iHandler.calculateGaitPhaseSeq();
            gaitSeq = obj.iHandler.mGaitDtr.gaitPhaseSeq;
            gcf = figure("Name","StateCalculator");
            plot(obj.iHandler.mRawData.Timestamp,gaitSeq,LineWidth=2);hold on;
            plot(obj.iHandler.mRawData.Timestamp,obj.pHandler.mGaitPhaseSeq);
            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('IMU和Plantar步态检测结果汇总图'); % 图形标题
            grid on; % 显示格线
            legend('0-支撑相，1-摆动相，-1-未知结果','2-脚跟着地，-2-脚尖离地');
         end


         %绘制融合相位检测、IMU相位检测、Plantar相位检测三者
         function plot_IMUGait_PlantarGait_Both(obj)            
            gaitSeq = obj.mStateSeq.GaitPhase.toMat();            
            gcf = figure("Name","StateCalculator");
            plot(obj.iHandler.mRawData.Timestamp,obj.iHandler.mGaitDtr.gaitPhaseSeq,LineWidth=2);hold on;
            plot(obj.iHandler.mRawData.Timestamp,obj.pHandler.mGaitPhaseSeq);hold on;
            plot(obj.iHandler.mRawData.Timestamp,gaitSeq,LineWidth=2);hold on;

            xlabel('时间戳/ms'); % x轴注解
            ylabel('步态检测结果'); % y轴注解
            title('融合相位检测、IMU相位检测、Plantar相位检测结果汇总图'); % 图形标题
            grid on; % 显示格线
            legend('IMU','Plantar','融合');
         end


        %二维轨迹图
        function plot_Track_2D(obj)            
            posSeq = obj.mStateSeq.P.toMat();            
            gcf = figure("Name","StateCalculator");           
            plot(posSeq(1,:),posSeq(2,:),LineWidth=2);hold on;

            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            title('二维轨迹图','FontSize', 16); % 图形标题
            grid on; % 显示格线
            axis equal;
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12          
        end


       %二维轨迹图
        function plot_Track_3D(obj)            
            posSeq = obj.mStateSeq.P.toMat();            
            gcf = figure("Name","StateCalculator");           
            plot3(posSeq(1,:),posSeq(2,:),posSeq(3,:),LineWidth=2);hold on;
            xlabel('X方向/米','FontSize', 16); % x轴注解
            ylabel('Y方向/米','FontSize', 16); % y轴注解
            zlabel('Z方向'); % z轴注解
            title('三维轨迹图'); % 图形标题
            grid on; % 显示格线
            axis equal;
            set(gca, 'FontSize', 12); % 设置刻度值字体大小为12      
%             % 调整视角为侧视图
%             view(0, 0);
        end

    end
end