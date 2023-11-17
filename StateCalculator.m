classdef StateCalculator <handle
    
    properties
        %状态变量
        V;%速度向量
        P;%位移向量
        Cbn;%姿态矩阵
        Phi;%欧拉角
        GaitPhase;%步态相位
        StateSeq;%状态向量序列
        
        %原始数据序列
        wSeq;%角速度序列
        fSeq;%加速度序列

        

        %常量
        delta_t = 0.006;    %更新间隔 (秒)
        gravity = 9.8;      %重力加速度数值(米每平方秒)
        gravityVec;      %重力加速度向量

        %核心组件
        imuHandler;%imu数据
        plantarHandler;%足底压力传感器数据
        gaitDtr;%步态检测器
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
            obj.imuHandler = imuHandler;
            obj.plantarHandler = plantarHandler;
            obj.gaitDtr = GaitPhaseDetector(); %步态检测器            
            obj.zuptKF  = ZuptKalmanFilter();  %卡尔曼滤波器
            obj.aligner = Aligner();           %姿态对准器
            obj.utils   = UtilContainer();     %工具函数集
       end

       %状态初始化
       function stateInit(obj,phi0)
            obj.Phi = phi0;
            obj.V = zeros(3,1);
            obj.P = zeros(3,1);
            obj.Cbn = obj.getCbnFromPhi(obj.Phi);
            obj.GaitPhase = obj.gaitDtr.PhaseUnknown;
            obj.gaitDtr.init();%步态检测初始化
            obj.zuptKF.init();%滤波器初始化 
            obj.initStateSeq();
            obj.saveStateToSeq();

       end
       
        
       function  initStateSeq(obj)
            obj.StateSeq = struct(  'V',MatList(),'P',MatList(),'Cbn',MatList(),'Phi',MatList(),...
                                    'GaitPhase',MatList(), ...                                    
                                    'KF_X',MatList(), 'KF_P',MatList(),'KF_K',MatList()...
                                 );
       end


       %将结果保存进序列
       function saveStateToSeq(obj)
            %状态量
            obj.StateSeq.V.addOne(obj.V);
            obj.StateSeq.P.addOne(obj.P);
            obj.StateSeq.Phi.addOne(obj.Phi);
            obj.StateSeq.Cbn.addOne([zeros(1,3);obj.Cbn]);
            %步态检测参数
            obj.StateSeq.GaitPhase.addOne(obj.GaitPhase);
            %kalman滤波器参数
            obj.StateSeq.KF_X.addOne(obj.zuptKF.X);
            obj.StateSeq.KF_P.addOne([9999*ones(1,9);obj.zuptKF.P]);
            obj.StateSeq.KF_K.addOne([9999*ones(1,3);obj.zuptKF.K]);
       end
       
       %状态解算
       function solveState(obj) 

            %结果赋初值
            obj.wSeq = [obj.imuHandler.mRawData.AngularVelX';
                        obj.imuHandler.mRawData.AngularVelY';
                        obj.imuHandler.mRawData.AngularVelZ'];
            obj.fSeq = [obj.imuHandler.mRawData.AccX';
                        obj.imuHandler.mRawData.AccY';
                        obj.imuHandler.mRawData.AccZ'];
            phi0 = [obj.imuHandler.mRawData.AngleRoll(1);
                    obj.imuHandler.mRawData.AnglePitch(1);
                    obj.imuHandler.mRawData.AngleYaw(1)];
            obj.stateInit(phi0);%状态初始化
                
%             %序列化迭代计算
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
                obj.GaitPhase = obj.gaitDtr.getPhase(obj.wSeq(:,1:i),...
                                                      obj.fSeq(:,1:i));
                %依据步态执行零速度更新
                %零速度修正
                if obj.GaitPhase == obj.gaitDtr.PhaseSwing 
                     %摆动相
                    obj.zuptKF.X = zeros(9,1); %误差项置零       
                elseif obj.GaitPhase == obj.gaitDtr.PhaseStance
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
            
                 elseif  obj.GaitPhase == obj.gaitDtr.PhaseUnknown %计算错误
                    
                end                
                %将计算结果保存
                obj.saveStateToSeq();
            end

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
end