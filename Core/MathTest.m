%%
clc
clear
% 模型推导
syms q0 q1 q2 q3;
syms gx gy gz bx by;
syms dt;
x = [q0;q1;q2;q3;bx;by];
% 过程模型
omega_x = gx - bx;
omega_y = gy - by;
omega_z = gz;
Omega = [0 -omega_x -omega_y -omega_z;
         omega_x 0 omega_z -omega_y;
         omega_y -omega_z 0 omega_x;
         omega_z omega_y -omega_x 0];
Q = [q0;q1;q2;q3];
f = [Q + 0.5*Omega*Q*dt;
    bx;
    by]

F = simplify(jacobian(f,x))

%%
clc
clear
% 模型推导
syms q0 q1 q2 q3;   %四元数
syms gx gy gz bx by bz;     %角速度及陀螺仪偏置
syms V [3 1];
syms P [3 1];
syms a [3 1];
syms dt;    %采样间隔

x = [q0;q1;q2;q3;bx;by;bz;V;P];
omega_x = gx - bx;
omega_y = gy - by;
omega_z = gz - bz;
Omega = [0 -omega_x -omega_y -omega_z;
         omega_x 0 omega_z -omega_y;
         omega_y -omega_z 0 omega_x;
         omega_z omega_y -omega_x 0];
Q = [q0;q1;q2;q3];
wb = [bx;by;bz];
g = [0;0;9.8];

% 将向量 a 转换为四元数
a1 = [0; a];

% 使用四元数 Q 旋转四元数 p
r = quatmultiply(quatmultiply(Q, a1), quatconj(Q));

% 提取旋转后的向量
a_rotated = r(2:4);

f = [Q + 0.5*Omega*Q*dt;
    wb;
    V + (a_rotated - g)*dt;
    P + V*dt]

F = simplify(jacobian(f,x))


%%
clc
clear
% 模型推导
syms q0 q1 q2 q3;   %四元数
syms gx gy gz bx by bz;     %角速度及陀螺仪偏置
syms vx vy vz;      %速度
syms px py pz;      %位移
syms dt;    %采样间隔
syms ax ay az;

x = [q0;q1;q2;q3;bx;by;bz;vx;vy;vz;px;py;pz];
omega_x = gx - bx;
omega_y = gy - by;
omega_z = gz - bz;
Omega = [0 -omega_x -omega_y -omega_z;
         omega_x 0 omega_z -omega_y;
         omega_y -omega_z 0 omega_x;
         omega_z omega_y -omega_x 0];
Q = [q0;q1;q2;q3];
V = [vx;vy;vz];
P = [px;py;pz];
wb = [bx;by;bz];
a = [ax;ay;az];
g = [0;0;9.8];

% 将向量 a 转换为四元数
a1 = [0; a];

% 使用四元数 Q 旋转四元数 p
r = quatmultiply(quatmultiply(Q, a1), quatconj(Q));

% 提取旋转后的向量
a_rotated = r(2:4);

f = [Q + 0.5*Omega*Q*dt;
    wb;
    V + (a_rotated - g)*dt;
    P + V*dt]

F = simplify(jacobian(f,x))


%%
clc
clear
syms q0 q1 q2 q3;   %四元数
syms gx gy gz bx by bz;     %角速度及陀螺仪偏置
syms V [3 1];
syms P [3 1];
syms a [3 1];
syms m [3 1];
syms dt;    %采样间隔


x = [q0;q1;q2;q3];
Q = [q0;q1;q2;q3];
wb = [bx;by;bz];
g = [0;0;9.8];

norm_m = quatmultiply(Q, quatmultiply([0;m],quatconj(Q)));
b = [0; 0; norm([norm_m(2) norm_m(3)]); norm_m(4)]

h = [2*(q1*q3 - q0*q2);
     2*(q2*q3 + q0*q1);
     q0^2 - q1^2 - q2^2 + q3^2;
     2*b(3)*(q0*q3 + q1*q2) + 2*b(4)*(q1*q3 - q0*q2);
     2*b(3)*(0.5 - 2*q1^2 - 2*q3^2) + 2*b(4)*(q0*q1 + q2*q3);
     2*b(3)*(q2*q3 - q0*q1) + 2*b(4)*(0.5 - q1^2 - q2^2);
     V]
 
H = simplify(jacobian(h,x))


%%
clc
clear
syms q0 q1 q2 q3;   %四元数
syms gx gy gz bx by bz;     %角速度及陀螺仪偏置
syms V [3 1];
syms P [3 1];
syms a [3 1];
% syms m [3 1];
syms b [4 1];
syms dt;    %采样间隔



Q = [q0;q1;q2;q3];
% x = [q0;q1;q2;q3];
x = [q0;q1;q2;q3;bx;by;bz;V;P];
g = [0;0;9.8];

hx = [2*(q1*q3 - q0*q2);
    2*(q0*q1 + q2*q3);
    2*(0.5 - q1^2 - q2^2);
    2*b(3)*(q0*q3 + q1*q2) + 2*b(4)*(q1*q3 - q0*q2);
    2*b(3)*(0.5 - q1^2 - q3^2) + 2*b(4)*(q0*q1 + q2*q3);
    2*b(3)*(q2*q3 - q0*q1) + 2*b(4)*(0.5 - q1^2 - q2^2)]

 
H = simplify(jacobian(hx,x))

%%
clc
clear
syms q0 q1 q2 q3;   %四元数
syms by bz; 
Q = [q0;q1;q2;q3];
b  = [0; 0; by; bz];
norm_m = quatmultiply(quatconj(Q), quatmultiply(b, Q));
simplify(norm_m)


%%
clc
clear
% 模型推导
syms q0 q1 q2 q3;   %四元数
syms gx gy gz bx by bz;     %角速度及陀螺仪偏置
syms V [3 1];
syms P [3 1];
syms a [3 1];
syms dt;    %采样间隔

Q = [q0;q1;q2;q3];

D = [ dt*(2*a1*q0 - 2*a2*q3 + 2*a3*q2),  dt*(2*a1*q1 + 2*a2*q2 + 2*a3*q3),  dt*(2*a2*q1 - 2*a1*q2 + 2*a3*q0), -dt*(2*a2*q0 + 2*a1*q3 - 2*a3*q1);
    dt*(2*a2*q0 + 2*a1*q3 - 2*a3*q1), -dt*(2*a2*q1 - 2*a1*q2 + 2*a3*q0),  dt*(2*a1*q1 + 2*a2*q2 + 2*a3*q3),  dt*(2*a1*q0 - 2*a2*q3 + 2*a3*q2);
    dt*(2*a2*q1 - 2*a1*q2 + 2*a3*q0),  dt*(2*a2*q0 + 2*a1*q3 - 2*a3*q1), -dt*(2*a1*q0 - 2*a2*q3 + 2*a3*q2),  dt*(2*a1*q1 + 2*a2*q2 + 2*a3*q3);]
d = D/(2*dt)

% % 将向量 a 转换为四元数
% a1 = [0; a];
% 
% % 使用四元数 Q 旋转四元数 p
% r = quatmultiply(quatmultiply(Q, a1), quatconj(Q))


%%
clc
clear
syms q0 q1 q2 q3;   %四元数
syms err_wx err_wy err_wz;
syms dt;    %采样间隔
syms a [3 1];
% syms err_q0 err_q1 err_q2 err_q3;
Q = [q0;q1;q2;q3];
err_q0 = sqrt((dt/2)*(q1^2*err_wx^2 + q2^2*err_wy^2 + q3^2*err_wz^2));
err_q1 = sqrt((dt/2)*(q0^2*err_wx^2 + q2^2*err_wz^2 + q3^2*err_wy^2));
err_q2 = sqrt((dt/2)*(q0^2*err_wy^2 + q1^2*err_wz^2 + q3^2*err_wx^2));
err_q3 = sqrt((dt/2)*(q0^2*err_wz^2 + q1^2*err_wy^2 + q2^2*err_wx^2));

% 将向量 a 转换为四元数
a1 = [0; a];
% 使用四元数 Q 旋转四元数 p
r = quatmultiply(quatmultiply(Q, a1), quatconj(Q));
% 提取旋转后的向量
a_rotated = r(2:4);
err_phi = [asin(2*(err_q2*err_q3+err_q0*err_q1));
           -atan2(2*(err_q1*err_q3-err_q0*err_q2),err_q0^2-err_q1^2-err_q2^2+err_q3^2);
           -atan2(2*(err_q1*err_q2-err_q0*err_q3),err_q0^2-err_q1^2+err_q2^2-err_q3^2)];

Ans = simplify(cross(a_rotated,err_phi))
dims = size(Ans)
% H = simplify(Ans)
% dims = size(H)

%%

% 定义四元数乘法函数
function res = quatmultiply(q, r)
    w1 = q(1); x1 = q(2); y1 = q(3); z1 = q(4);
    w2 = r(1); x2 = r(2); y2 = r(3); z2 = r(4);
    res = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
           w1*x2 + x1*w2 + y1*z2 - z1*y2;
           w1*y2 - x1*z2 + y1*w2 + z1*x2;
           w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

% 计算四元数的共轭
function res = quatconj(q)
    res = [q(1); -q(2:4)];
end




