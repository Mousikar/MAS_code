% nonlinear system 动态领导者时滞变化
clear
% 随机种子
rng(88);

% 迭代设置
t_sum = 20;
T = 0.001;
iter = floor(t_sum / T);
num_follower = 6;
num_leader = 4;

% 初值
x = 1.5 * ones(1, num_follower) + 2 * rand(1, num_follower);
y = 1.5 * ones(1, num_follower) + 2 * rand(1, num_follower);
dot_x = zeros(1, num_follower);
dot_y = zeros(1, num_follower);
theta = -pi + 2 * pi * rand(1, num_follower);
ddot_hat_theta_d = [0,0,0,0,0,0];
dot_hat_theta_d = [0,0,0,0,0,0];
hat_theta_d = [0,0,0,0,0,0];
rx = [2 3 3 2];
ry = [2 2 3 3];
dot_rx = 5 * (1 / T / iter) * ones(1, num_leader);
dot_ry = 5 * (1 / T / iter) * ones(1, num_leader);

% 系数
k1 = 2;
k2 = 2;
k3 = 4;
R = 100;

% 网络拓扑
A_F = [      0     0     0     0     0     0
             1     0     0     0     0     0
             0     1     0     1     0     0
             0     0     0     0     0     0
             0     1     0     0     0     0
             1     0     1     0     0     0]; % 跟随者邻接矩阵
A_LF = [     1     0     0     1
             0     1     0     0
             0     0     1     0
             0     0     1     1
             1     0     0     0
             0     0     0     0]; % 领导者和跟随者的耦合邻接矩阵
A = [A_F,A_LF];
B = diag(sum(A')); % 跟随者能接受到的领导者信息总和

L = -A_F;

L1 = L + B;
L2 = -A_LF;
xishu = inv(L1) * L2;

% 时延设置
global tau;
tau = 0.5;
dmax = tau/T;

% 初始化变量
ux = zeros(1, num_follower);
uy = zeros(1, num_follower);
theta_d = zeros(1, num_follower);
v = zeros(1, num_follower);
omega = zeros(1, num_follower);
dot_hat_theta_d = zeros(1, num_follower);

% 历史变量初始化
x_history = zeros(dmax, num_follower);
y_history = zeros(dmax, num_follower);
theta_history = zeros(dmax, num_follower);
dot_x_history = zeros(dmax, num_follower);
dot_y_history = zeros(dmax, num_follower);
dot_rx_history = zeros(dmax, num_leader);
dot_ry_history = zeros(dmax, num_leader);
v_history = zeros(dmax, num_follower);
omega_history = zeros(dmax, num_follower);
theta_d1 = [0,0,0,0,0,0];
kk = [0,0,0,0,0,0];
rx_history = zeros(dmax, num_leader);
ry_history = zeros(dmax, num_leader);
hat_ex_history = zeros(dmax, num_follower);
hat_ey_history = zeros(dmax, num_follower);
errx_actual_history = zeros(dmax, num_follower);
erry_actual_history = zeros(dmax, num_follower);
hat_evx_history = zeros(dmax, num_follower);
hat_evy_history = zeros(dmax, num_follower);

for i = 1:dmax
    x_history(i, :) = x;
    y_history(i, :) = y;
    theta_history(i, :) = theta;
    rx_history(i, :) = rx;
    ry_history(i, :) = ry;
end
%% 迭代
for k = 1:iter
    delta_v = 1/8*cos(2*pi/10*k*T)*2*pi/10;
    dot_rx = 5 * (1 / T / iter) * ones(1, num_leader) + ...
        [-delta_v delta_v delta_v -delta_v];
    dot_ry = 5 * (1 / T / iter) * ones(1, num_leader) + ...
        [-delta_v -delta_v delta_v delta_v];

    hat_ex = zeros(1, num_follower);
    hat_ey = zeros(1, num_follower);
    dot_dx = zeros(1, num_follower);
    dot_dy = zeros(1, num_follower);
    hat_evx = zeros(1, num_follower);
    hat_evy = zeros(1, num_follower);

    % 控制方程
    for i = 1:num_follower
        for j = 1:num_follower
            hat_ex(i) = hat_ex(i) + A_F(i, j) * (x_history(end - timedelay(i,j,k,'F') + 1, i) - x_history(end - timedelay(i,j,k,'F') + 1, j));
            hat_ey(i) = hat_ey(i) + A_F(i, j) * (y_history(end - timedelay(i,j,k,'F') + 1, i) - y_history(end - timedelay(i,j,k,'F') + 1, j));
            dot_dx(i) = dot_dx(i) + A_F(i, j)/L1(i,i) * dot_x_history(end - timedelay(i,j,k,'F') + 1, j);
            dot_dy(i) = dot_dy(i) + A_F(i, j)/L1(i,i) * dot_y_history(end - timedelay(i,j,k,'F') + 1, j);
        end
        for j = 1:num_leader
            hat_ex(i) = hat_ex(i) + A_LF(i, j) * (x_history(end - timedelay(i,j,k,'L') + 1, i) - rx_history(end - timedelay(i,j,k,'L') + 1, j));
            hat_ey(i) = hat_ey(i) + A_LF(i, j) * (y_history(end - timedelay(i,j,k,'L') + 1, i) - ry_history(end - timedelay(i,j,k,'L') + 1, j));
            dot_dx(i) = dot_dx(i) + A_LF(i, j)/L1(i,i) * dot_rx_history(end - timedelay(i,j,k,'L') + 1, j);
            dot_dy(i) = dot_dy(i) + A_LF(i, j)/L1(i,i) * dot_ry_history(end - timedelay(i,j,k,'L') + 1, j);        
        end
        ux(i) = dot_dx(i) - k1 * hat_ex(i);
        uy(i) = dot_dy(i) - k2 * hat_ey(i);

        theta_d(i) = atan2(uy(i), ux(i));

        if k == 0
            theta_d1(i) = theta_d(i);
            kk(i) = 0;
        end
        
        delta = - 0.7 * pi^2;
        
        if theta_d(i) * theta_d1(i) < delta
            if theta_d(i) < 0
                kk(i) = kk(i) + 1;
            else
                kk(i) = kk(i) - 1;
            end
        end
        
        theta_d1(i) = theta_d(i);
        theta_d(i) = theta_d(i) + 2 * pi * kk(i);
        
        v(i) = sqrt(uy(i)^2 + ux(i)^2);

        ddot_hat_theta_d(i) = - R^2 * (hat_theta_d(i) - theta_d(i)) - 2 * R * dot_hat_theta_d(i);  % 线性二阶微分器
        dot_hat_theta_d(i) = dot_hat_theta_d(i) + ddot_hat_theta_d(i) * T;
        hat_theta_d(i) = hat_theta_d(i) + dot_hat_theta_d(i) * T;
        
        omega(i) = dot_hat_theta_d(i) + k3 * (theta_d(i) - theta(i));  % 暂时不加上饱和函数

                
    end

    errx_actual = L1 * x' + L2 * rx';
    erry_actual = L1 * y' + L2 * ry';
    hat_evx = L1 * dot_x' + L2 * dot_rx';
    hat_evy = L1 * dot_y' + L2 * dot_ry';

    % 系统方程
    for i = 1:num_follower
        x(i) = x(i) + v(i) * T * cos(theta(i));    % v和x是N个智能体的速度和x坐标
        y(i) = y(i) + v(i) * T * sin(theta(i));    % y是N个智能体的y坐标
        theta(i) = theta(i) + omega(i) * T;
        dot_x(i) = v(i) * cos(theta(i));
        dot_y(i) = v(i) * sin(theta(i));

    end
    for i = 1:num_leader
        rx(i) = rx(i) + dot_rx(i) * T;
        ry(i) = ry(i) + dot_ry(i) * T;
    end

    x_history = [x_history; x];
    y_history = [y_history; y];
    theta_history = [theta_history; theta];
    v_history = [v_history; v];
    omega_history = [omega_history; omega];
    rx_history = [rx_history; rx];
    ry_history = [ry_history; ry];
    hat_ex_history = [hat_ex_history; -hat_ex];
    hat_ey_history = [hat_ey_history; -hat_ey];
    errx_actual_history = [errx_actual_history; errx_actual'];
    erry_actual_history = [erry_actual_history; erry_actual'];
    hat_evx_history = [hat_evx_history; hat_evx'];
    hat_evy_history = [hat_evy_history; hat_evy'];
    dot_x_history(k+dmax,:) = dot_x;
    dot_y_history(k+dmax,:) = dot_y;
    dot_rx_history(k+dmax,:) = dot_rx;
    dot_ry_history(k+dmax,:) = dot_ry;

    if mod(k,1000)==0
        % 计算进度
        progress = k / iter * 100;    
        % 打印进度信息
        fprintf('进度：%0.2f%%\n', progress);
    end
end

% 均匀时滞 
function result = timedelay(i,j,k,ForL)
    t=k*0.001;
    % t=1.5;
    global tau;
    tau = 0.5;
    result=0.001;
    if ForL=='L'
        j=j+6;
    end
    if i==1 && j==7
        result = tau * abs(cos(t));
    end
    if i==2 && j==8
        result = tau * abs(cos(t));
    end
    if i==1 && j==10
        result = tau * abs(cos(t));
    end
    if i==2 && j==1
        result = tau * abs(cos(t));
    end
    if i==3 && j==9
        result = tau * abs(cos(t));
    end
    if i==4 && j==10
        result = tau * abs(cos(t));
    end
    if i==3 && j==2
        result = tau * abs(cos(t));
    end
    if i==3 && j==4
        result = tau * abs(cos(t));
    end
    if i==5 && j==2
        result = tau * abs(cos(t));
    end
    if i==6 && j==5
        result = tau * abs(cos(t));
    end
    if i==4 && j==8
        result = tau * abs(cos(t));
    end
    if i==5 && j==9
        result = tau * abs(cos(t));
    end
    if i==6 && j==3
        result = tau * abs(cos(t));
    end
    result = floor(result / 0.001);
    if result==0
        result=1;
    end
end