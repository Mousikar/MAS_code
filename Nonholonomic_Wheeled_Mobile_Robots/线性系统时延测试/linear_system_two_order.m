% linear system 二阶系统
clear
% 随机种子
rng(88);

% 迭代设置
t_sum = 50;
T = 0.001;
iter = floor(t_sum / T);
num_follower = 6;
num_leader = 4;

% 初值
x = zeros(1, num_follower) + 2 * rand(1, num_follower);
y = zeros(1, num_follower) + 2 * rand(1, num_follower);
theta = -pi + 2 * pi * rand(1, num_follower);
dot_x = zeros(1, num_follower);
dot_y = zeros(1, num_follower);

rx = 1 + 2 * rand(1, num_leader);
ry = 1 + 2 * rand(1, num_leader);
dot_rx = 0.25* ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);
dot_ry = 0.25* ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);

% 系数
k1 = 3.8;
k2 = 1.1;
k3 = 0.12;
R = 100;

% 网络拓扑
A_F = [0,1,0,0,0,0;
       0,0,0,0,0,0;
       0,0,0,0,0,0;
       0,1,0,0,0,0;
       1,0,0,0,0,1;
       0,0,1,0,1,0]; % 跟随者邻接矩阵

B = [1,0,0,0,0,0;
     0,2,0,0,0,0;
     0,0,1,0,0,0;
     0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,0]; % 跟随者能接受到的领导者信息总和

A_LF = [1,0,0,0;
         0,1,1,0;
         0,0,0,1;
         1,0,0,0;
         0,1,0,0;
         0,0,0,0]; % 领导者和跟随者的耦合邻接矩阵

L = -A_F;
for i = 1:num_follower
    L(i,i) = sum(A_F(i,:));
end

L1 = L + B;
L2 = -A_LF;
xishu = inv(L1) * L2;

% 时延设置
tau_actual = [10, 10, 10, 10, 10, 10]; % 根据实际情况填充
tau = repmat(tau_actual, num_follower, 1) / 1000; % 初始化时延
d = floor(tau / T);
dmax = max(max(d));

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
dot_x_history = zeros(dmax, num_follower);
dot_y_history = zeros(dmax, num_follower);
theta_history = zeros(dmax, num_follower);
v_history = zeros(iter, num_follower);
omega_history = zeros(iter, num_follower);
rx_history = zeros(dmax, num_leader);
ry_history = zeros(dmax, num_leader);
hat_ex_history = zeros(iter, num_follower);
hat_ey_history = zeros(iter, num_follower);
errx_actual_history = zeros(iter, num_follower);
erry_actual_history = zeros(iter, num_follower);
hat_evx_history = zeros(iter, num_follower);
hat_evy_history = zeros(iter, num_follower);

for i = 1:dmax
    x_history(i, :) = x;
    y_history(i, :) = y;
    theta_history(i, :) = theta;
    rx_history(i, :) = rx;
    ry_history(i, :) = ry;
end

for k = 1:iter
    hat_ex = zeros(1, num_follower);
    hat_ey = zeros(1, num_follower);
    dot_dx = zeros(1, num_follower);
    dot_dy = zeros(1, num_follower);
    hat_evx = zeros(1, num_follower);
    hat_evy = zeros(1, num_follower);

    % 控制方程
    for i = 1:num_follower
        for j = 1:num_follower
            % hat_ex(i) = hat_ex(i) - A_F(i, j) * ( (x(i) - x(j)) );
            % hat_ey(i) = hat_ey(i) - A_F(i, j) * ( (y(i) - y(j)) );
            % hat_evx(i) = hat_evx(i) - A_F(i, j) * ( k2 * (dot_x(i) - dot_x(j)) );
            % hat_evy(i) = hat_evy(i) - A_F(i, j) * ( k2 * (dot_y(i) - dot_y(j)) );
            hat_ex(i) = hat_ex(i) - A_F(i, j) * (x(i) - x_history(end - d(i, j) + 1, j));
            hat_ey(i) = hat_ey(i) - A_F(i, j) * (y(i) - y_history(end - d(i, j) + 1, j));
            hat_evx(i) = hat_evx(i) - A_F(i, j) * ( k2 * (dot_x(i) - dot_x_history(end - d(i, j) + 1, j)) );
            hat_evy(i) = hat_evy(i) - A_F(i, j) * ( k2 * (dot_y(i) - dot_y_history(end - d(i, j) + 1, j)) );
        end
        for j = 1:num_leader
            % hat_ex(i) = hat_ex(i) - A_LF(i, j) * ( (x(i) - rx(j)) );
            % hat_ey(i) = hat_ey(i) - A_LF(i, j) * ( (y(i) - ry(j)) );
            % hat_evx(i) = hat_evx(i) - A_LF(i, j) * ( k2 * (dot_x(i) - dot_rx(j)) );
            % hat_evy(i) = hat_evy(i) - A_LF(i, j) * ( k2 * (dot_y(i) - dot_ry(j)) );
            hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x(i) - rx_history(end - d(i, j) + 1, j));
            hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y(i) - ry_history(end - d(i, j) + 1, j));
            hat_evx(i) = hat_evx(i) - A_LF(i, j) * ( k2 * (dot_x(i) - dot_rx(j)) );
            hat_evy(i) = hat_evy(i) - A_LF(i, j) * ( k2 * (dot_y(i) - dot_ry(j)) );
        end
        ux(i) = hat_ex(i) + hat_evx(i);
        uy(i) = hat_ey(i) + hat_evy(i);
    end

    errx_actual = L1 * x' + L2 * rx';
    erry_actual = L1 * y' + L2 * ry';
    % 系统方程
    dot_x = dot_x + ux * T;
    dot_y = dot_y + uy * T;
    x = x + dot_x * T;
    y = y + dot_y * T;
    rx = rx + dot_rx * T;
    ry = ry + dot_ry * T;

    x_history(k+dmax,:) = x;
    y_history(k+dmax,:) = y;
    dot_x_history(k+dmax,:) = dot_x;
    dot_y_history(k+dmax,:) = dot_y;
    theta_history(k+dmax,:) = theta;
    rx_history(k+dmax,:) = rx;
    ry_history(k+dmax,:) = ry;
    hat_ex_history(k,:) = -hat_ex;
    hat_ey_history(k,:) = -hat_ey;
    errx_actual_history(k,:) = errx_actual';
    erry_actual_history(k,:) = erry_actual';
    hat_evx_history(k,:) = hat_evx;
    hat_evy_history(k,:) = hat_evy;

    if mod(k,1000)==0
        % 计算进度
        progress = k / iter * 100;    
        % 打印进度信息
        fprintf('进度：%0.2f%%\n', progress);
    end
end

