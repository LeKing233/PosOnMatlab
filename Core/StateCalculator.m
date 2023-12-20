classdef StateCalculator <handle

    properties
        %状态变量
        AccENU;%东北天坐标系下的加速度向量
        V;%速度向量
        P;%位移向量
        Cbn;%姿态矩阵
        Phi;%欧拉角
        GaitPhase;%步态相位
        mStateSeq;%状态结果序列
        mSeqLength;%序列长度

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

            obj.utils   = Utils();     %工具函数集
        end

        %状态初始化
        function stateInit(obj,phi0)
            obj.Phi = phi0;
            obj.AccENU = zeros(3,1);
            obj.V = zeros(3,1);
            obj.P = zeros(3,1);
            obj.Cbn = obj.getCbnFromPhi(obj.Phi);
            obj.zuptKF.init();%滤波器初始化

            obj.GaitPhase = Utils.Phase_Unknown;
            obj.initStateSeq();
            obj.saveStateToSeq(1);

        end


        function  initStateSeq(obj)
            %            seqLength = obj.iHandler.mSeqLength;
            %            obj.mStateSeq.V = zeros(seqLength,3);
            %            obj.mStateSeq.P = zeros(seqLength,3);
            %            obj.mStateSeq.Cbn = zeros(seqLength*4,3);%每个Cbn之间要加一行间隔行
            %            obj.mStateSeq.CbnGap = zeros(1,3);%Cbn间隔行
            %            obj.mStateSeq.Phi = zeros(seqLength,3);
            %            obj.mStateSeq.GaitPhase = zeros(seqLength,1);
            %            obj.mStateSeq.KF_X = zeros(seqLength,9);%KF_X为9*1矩阵
            %            obj.mStateSeq.KF_P = zeros(seqLength*10,9);%KF_P为9*9矩阵，中间间隔一行
            %            obj.mStateSeq.KF_PGap = zeros(1,9);
            %            obj.mStateSeq.KF_K = zeros(seqLength*10,3);%KF_K为9*3矩阵，中间间隔一行
            %            obj.mStateSeq.KF_KGap = zeros(1,3);


            obj.mStateSeq = struct(  'V',MatList(),'P',MatList(),'Phi',MatList(),'AccENU',MatList(),'Cbn',MatList(),...
                'GaitPhase',MatList(), ...
                'KF_X',MatList(), 'KF_P',MatList(),'KF_K',MatList()...
                );
        end


        %将结果保存进序列
        function saveStateToSeq(obj,index)
            %             obj.mStateSeq.V(index,:) = obj.V;
            %             obj.mStateSeq.P(index,:) = obj.P;
            %             index_Cbn = index+4*(index-1);%Cbn保存开始索引
            %             obj.mStateSeq.Cbn(index_Cbn:(index_Cbn+4-1),:) = [obj.Cbn;obj.mStateSeq.CbnGap];
            %             obj.mStateSeq.Phi(index,:) = obj.Phi;
            %             obj.mStateSeq.GaitPhase(index,:) = obj.GaitPhase;
            %             obj.mStateSeq.KF_X(index,:) = obj.zuptKF.X';
            %             index_KF_P = index+10*(index-1);%KF_P保存索引
            %             obj.mStateSeq.KF_P(index_KF_P:(index_KF_P+10-1),:) = [obj.zuptKF.P;obj.mStateSeq.KF_PGap];
            %             index_KF_K = index + 10*(index-1);
            %             obj.mStateSeq.KF_K(index_KF_K:(index_KF_K+10-1),:) = [obj.zuptKF.K;obj.mStateSeq.KF_KGap];



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
            obj.mStateSeq.KF_P.addOne([obj.zuptKF.P;9999*ones(1,9)]);
            obj.mStateSeq.KF_K.addOne([obj.zuptKF.K;9999*ones(1,3)]);
        end



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
        end


        %状态解算
        function result = solveState(obj,gaitDetectType)

            % 在函数开始时初始化一个空的矩阵或结构体数组
            result = struct('Speeds',{},'StepDeltaTime',{},'StepLength', {}, 'cos_theta', {}, 'sin_theta', {}, 'PeXi', {}, 'PeYi', {});

            %结果赋初值
            obj.wSeq = obj.iHandler.mWSeq;
            obj.fSeq = obj.iHandler.mFSeq;
            phi0 = [obj.iHandler.mRawData.AngleX(1);
                obj.iHandler.mRawData.AngleY(1);
                obj.iHandler.mRawData.AngleZ(1)];
            obj.stateInit(phi0);%状态初始化

            %获取时间序列
            timestamps = obj.iHandler.mRawData.Timestamp;

            %Pe-根据步长估计的轨迹
            Pe = zeros(length(obj.pHandler.mSumSeqInArea.T_locations),2);
            counter = 1;
            lastEstiTime = timestamps(1);

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
                obj.AccENU = obj.Cbn*obj.fSeq(:,i);
                %计算步态
                obj.GaitPhase = obj.getGaitPhase(i,gaitDetectType);
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
                obj.saveStateToSeq(i);

                %记录步速
                speed = obj.pHandler.mWalkSpeedSeq(i);
                if(speed ~= 0)
                    counter  = counter + 1;
                    StepDeltaTime = (timestamps(i)-lastEstiTime)/1000;
                    StepLength = speed*StepDeltaTime;
                    v = obj.V;
                    meanV_XY = sqrt(v(1)^2+v(2)^2);%水平方向速度模长
                    cos_theta = v(1)./meanV_XY;
                    sin_theta = v(2)./meanV_XY;
                    PeXi = StepLength*cos_theta
                    PeYi = StepLength*sin_theta
                    Pe(counter,1) = Pe(counter-1,1) + StepLength;
                    Pe(counter,2) = Pe(counter-1,2) + StepLength;
                    lastEstiTime = timestamps(i);



                    % 将这些计算结果保存到结构体数组
                    result(end + 1).Speeds = speed;
                    result(end).StepDeltaTime = StepDeltaTime;
                    result(end).StepLength = StepLength;
                    result(end).cos_theta = cos_theta;
                    result(end).sin_theta = sin_theta;
                    result(end).PeXi = PeXi;
                    result(end).PeYi = PeYi;
                end

            end

        obj.mStateSeq.Pe = Pe;
        %打标签

        obj.mStateSeq.Legend = gaitDetectType;



        %将结果序列作为返回值返回

        obj.stateSeqToMat();
    

    end


    %步态检测
    function phase = getGaitPhase(obj,index,gaitDetectType)
        phaseIMU = obj.iHandler.getPhase(index);
        phasePlantar = obj.pHandler.getGaitPhase(index);
        switch gaitDetectType
            case "Both"
                %方法一：将两者相位取"与"运算
                if (phaseIMU==Utils.Phase_Static) && (phasePlantar == Utils.Phase_Landing)
                    phase = Utils.Phase_Stance;
                else
                    phase = Utils.Phase_Swing;
                end

            case "Imu"
                %方法二：以IMU为准
                if (phaseIMU==Utils.Phase_Static)
                    phase = Utils.Phase_Stance;
                else
                    phase = Utils.Phase_Swing;
                end

            case "Plantar"
                %方法三：以Plantar为准
                if (phasePlantar==Utils.Phase_Landing)
                    phase = Utils.Phase_Stance;
                else
                    phase = Utils.Phase_Swing;
                end
            otherwise
                error("未知类型");
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