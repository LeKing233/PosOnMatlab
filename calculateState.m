
%% ----------------------Main Function--------------------- 
function result = calculateState(w,f)
    %变量声明
    global WINDOW_LENGTH;    
    global Phi;
    global V;
    global P;
    global Cbn;
    global KF_X;
    global KF_K;
    global KF_P;

    %初始
    ConstInit();%常量赋值
    StateInit();%状态量初始化
    KF_Init();%卡尔曼滤波器初始化

    %结果赋初值
    result.Phi = Phi;
    result.V = V;
    result.P = P;
    result.GaitPhase = 0;
    result.KF_X = zeros(9,1);
    result.KF_K = zeros(9,3);
    result.KF_P = zeros(9,9);

    %序列化迭代    
     for  i = 2:length(w)
        %记录上一时刻状态量
        Cbn_prev = Cbn;
        V_prev = V;
        P_prev = P;
        Phi_prev = Phi;
        %计算 
        Cbn = getUpdateCbn(Cbn_prev,w(:,i));
        V = getUpdateVn(V_prev,f(:,i),Cbn,f(:,i-1),Cbn_prev);
        P = getUpdatePn(P_prev,V,V_prev);
        Phi = getPhiFromCbn(Cbn); 
        
        %步态检测
        if i<=WINDOW_LENGTH
            gaitPhase = -1;
        else
            gaitPhase = getGaitPhase(w(:,i-WINDOW_LENGTH:i),f(:,i-WINDOW_LENGTH:i));
        end

        %零速度修正
        if gaitPhase == 1       %摆动相
            KF_X = zeros(9,1);        
        elseif gaitPhase == 0   %支撑相
            %预测
            F = getF(Cbn,f(:,i));
            G = getG(Cbn);
            predict(F,G);
            %更新
            Z = -V;
            update(Z);
            %修正
            V = reviseVn(V);
            P = revisePn(P);
            Cbn = reviseCbn(Cbn);
    
         elseif  gaitPhase == -1 %计算错误
            
         end
        
        %将计算结果添加进返回值结构体
        result.Phi = [result.Phi,Phi];
        result.V = [result.V,V];
        result.P = [result.P,P];
        result.GaitPhase = [result.GaitPhase,gaitPhase];
        result.KF_X = [result.KF_X,KF_X];
        result.KF_K = [result.KF_K,KF_K];
        result.KF_P = [result.KF_P;zeros(1,9);KF_P];
     end
end
%% ----------------------变量初始化--------------------- 
function ConstInit()
    global NOISE_W ;
    global NOISE_F;
    global NOISE_R;
    global NOISE_W_GAIT;
    global NOISE_F_GAIT;
    global G;
    global GRAVITY;
    global DELTA_T;
    global WINDOW_LENGTH;
    global GAIT_THRESHOLD;

    
    NOISE_W = [0.007;0.007;0.007];
    NOISE_F = [0.001;0.001;0.001];
    
    NOISE_W_GAIT =  [20;20;20];
    NOISE_F_GAIT =  [10;10;10];

    NOISE_R = [0.00005;0.00005;0.00005];
    G = 10;
    GRAVITY  = [0;0;G];
    DELTA_T = 0.006;

    GAIT_THRESHOLD = 20;
    WINDOW_LENGTH = 10;
end

function  StateInit()
    global Phi;
    global V;
    global P;
    global Cbn;

    Phi = [0;0;0];
    V  = [0;0;0];
    P = [0;0;0];
    Cbn = getCbnFromPhi(Phi);    
end 

function KF_Init()
    global KF_P;
    global KF_X;
    global KF_Q;
    global KF_R;
    global KF_K;
    global KF_H;

    global NOISE_W;
    global NOISE_F;
    global NOISE_R;

    KF_K = zeros(9,3);
    KF_P = eye(9);
    KF_X = zeros(9,1);
    KF_Q = diag([NOISE_W(1)^2,NOISE_W(2)^2,NOISE_W(3)^2,...
                 NOISE_F(1)^2,NOISE_F(2)^2,NOISE_F(3)^2]);
    KF_R = diag([NOISE_R(1)^2,NOISE_R(2)^2,NOISE_R(3)^2]);

    KF_H = [zeros(3,3),eye(3,3),zeros(3,3)];

end 



%% ----------------------步态检测--------------------- 
function gaitPhase = getGaitPhase(wSeq,fSeq)
    global GAIT_THRESHOLD;
% 提取窗口数据,计算统计量
    T = calculateGaitStatics(wSeq,fSeq);      
% 根据统计量获取步相
    if T < GAIT_THRESHOLD
        gaitPhase = 0;%支撑相
    else
        gaitPhase = 1;%摆动相
    end
end

function T = calculateGaitStatics(wSeq,fSeq)
    global NOISE_W_GAIT;
    global NOISE_F_GAIT;
    global G;
    
    T = 0;    
    f_mean = mean(fSeq);
    w = length(fSeq);%窗口长度
    
    for i = 1:w
        T = T + 1/w * (1/norm(NOISE_W_GAIT)^2 * norm(wSeq(:,i))^2 + 1/norm(NOISE_F_GAIT)^2 * norm(fSeq(:,i) - G*f_mean/norm(f_mean))^2);
    end
    
end

function crossMatrix = getCrossMatrix(v)
    % 输入参数 v 是一个三维列向量
    % 计算反对称矩阵
    crossMatrix = [0, -v(3), v(2);
                   v(3), 0, -v(1);
                   -v(2), v(1), 0];
end

%% ----------------------状态更新预测--------------------- 
function  Cbn = getCbnFromPhi(phi)
    gamma = phi(1);%横滚角roll
    theta = phi(2);%俯仰角pitch
    psi = phi(3);%翻滚角yaw
    
    Cbn = [cos(theta)*cos(psi),-cos(gamma)*sin(psi)+sin(gamma)*sin(theta)*cos(psi),sin(gamma)*sin(psi)+cos(gamma)*sin(theta)*cos(psi);
           cos(theta)*sin(psi),cos(gamma)*cos(psi)+sin(gamma)*sin(theta)*sin(psi),-sin(gamma)*cos(psi)+cos(gamma)*sin(theta)*sin(psi);
           -sin(theta),sin(gamma)*cos(theta),cos(gamma)*cos(theta)];
end

function  Cbn = getUpdateCbn(Cbn_prev,w)
    global DELTA_T;
    I = eye(3);
    wx = getCrossMatrix(w);%角速度w构成的反对称矩阵[w×]
    Cbn = Cbn_prev * ((2*I + wx*DELTA_T)/(2*I - wx*DELTA_T));
end

function Vn = getUpdateVn(Vn_prev,fn,Cbn,fn_prev,Cbn_prev)
    global DELTA_T;
    global GRAVITY;
    Vn = Vn_prev + (0.5*(Cbn*fn+Cbn_prev*fn_prev) - GRAVITY)*DELTA_T;
end

function Pn = getUpdatePn(Pn_prev,Vn,Vn_prev)
    global DELTA_T;
    Pn = Pn_prev + 0.5*(Vn_prev + Vn) * DELTA_T;
end

function Phi = getPhiFromCbn(Cbn)
    threshold = 0.000001;
    %横滚角Roll
    T33  = Cbn(3,3);
    T32 = Cbn(3,2);
    roll = atan(Cbn(3,2)/Cbn(3,3));
    if(abs(T33) <= threshold)
        if(T32>=0)
            roll = -pi/2;
        else
            roll = pi/2;
        end
    elseif T33<0
        if(T32<0)
            roll = pi - roll;
        else %T32>0
            roll = -pi - roll;
        end
    else % T33>=0 && abs(T33> threshold) 
        roll = -roll;
    end
    Phi(1) = roll;
    %俯仰角Pitch  
    Phi(2) = asin(-Cbn(3,3));
    %航向角yaw
    yaw = atan(Cbn(2,1)/Cbn(1,1));
    T11 = Cbn(1,1);
    T21 = Cbn(2,1);    
    if(abs(T11) <= threshold)
        if(T21>=0)
            yaw = pi/2;
        else %T21<0
            yaw = 3/2*pi;
        end
    elseif T11>0
        if(T21<0)
            yaw = 2*pi + yaw;
        else %T21>=0
            yaw = yaw;
        end
    else % T11<=0 && abs(T11> threshold) 
        yaw = pi + yaw;
    end
    Phi(3) = yaw;
    
    Phi = Phi'*180/pi;%转置成列向量
end



%% ----------------------卡尔曼滤波器--------------------- 

function predict(F,G)
    global KF_P;
    global KF_Q;

    KF_P = F*KF_P*F' + G*KF_Q*G';   
end 

function update(Z)
    global KF_P;
    global KF_X;
    global KF_R;
    global KF_K;
    global KF_H;
    
    I = eye(9);
    KF_K = KF_P*KF_H'/(KF_H * KF_P * KF_H' + KF_R);
    KF_X = KF_K * Z;
    KF_P = (I - KF_K*KF_H)*KF_P*(I - KF_K*KF_H)' + KF_K*KF_R*KF_K';
    

end

function F = getF(Cbn,fn)
    global DELTA_T;
    I = eye(3);
    ZeroMatrix = zeros(3,3);
    F = [I,ZeroMatrix,ZeroMatrix;
         (getCrossMatrix(Cbn*fn))*DELTA_T,I,ZeroMatrix;
         ZeroMatrix,I*DELTA_T,I];
end

function G = getG(Cbn)
    global DELTA_T;
    ZeroMatrix = zeros(3,3);
    G = [-Cbn*DELTA_T,ZeroMatrix;
         ZeroMatrix,Cbn*DELTA_T;
         ZeroMatrix,ZeroMatrix];
end

%% ----------------------零速度修正---------------------  
function Vn = reviseVn(Vn_prev,KF_X) 
    global KF_X;
    Vn = Vn_prev + KF_X(4:6);
end 

function Pn = revisePn(Pn_prev)
    global KF_X;
    Pn = Pn_prev + KF_X(7:9);
end

function Cbn = reviseCbn(Cbn_prev) 
    global KF_X;
    I = eye(3);
    delta_phi_x = getCrossMatrix(KF_X(1:3));
    Cbn =  ((2*I + delta_phi_x)/(2*I - delta_phi_x)) * Cbn_prev;
end 




