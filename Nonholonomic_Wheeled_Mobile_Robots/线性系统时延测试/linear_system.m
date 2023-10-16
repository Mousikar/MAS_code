% linear system
clear
% 随机种子
rng(87);

% 迭代设置
t_sum = 2;
T = 0.0001;
iter = floor(t_sum / T);
num_follower = 6;
num_leader = 4;
num_leader = 6;

% 初值
x = zeros(1, num_follower) + 2 * rand(1, num_follower);
y = zeros(1, num_follower) + 2 * rand(1, num_follower);
theta = -pi + 2 * pi * rand(1, num_follower);

rx = 1 + 2 * rand(1, num_leader);
ry = 1 + 2 * rand(1, num_leader);
dot_rx = 0.25 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);
dot_ry = 0.25 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);

% 系数
k1 = 3.8;
k2 = 3.8;
k3 = 0.12;
R = 100;

% % 网络拓扑
% A_F = [0,1,0,0,0,0;
%        0,0,0,0,0,0;
%        0,0,0,0,0,0;
%        0,1,0,0,0,0;
%        1,0,0,0,0,1;
%        0,0,1,0,1,0]; % 跟随者邻接矩阵
% 
% B = [1,0,0,0,0,0;
%      0,2,0,0,0,0;
%      0,0,1,0,0,0;
%      0,0,0,1,0,0;
%      0,0,0,0,1,0;
%      0,0,0,0,0,0]; % 跟随者能接受到的领导者信息总和
% 
% A_LF = [1,0,0,0;
%          0,1,1,0;
%          0,0,0,1;
%          1,0,0,0;
%          0,1,0,0;
%          0,0,0,0]; % 领导者和跟随者的耦合邻接矩阵
A_F = [0,1,0,1,0,0;
       1,0,1,0,0,0;
       0,1,0,0,0,1;
       1,0,0,0,1,0;
       0,0,0,1,0,1;
       0,0,1,0,1,0]; % 跟随者邻接矩阵

B = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,1,0,0,0;
     0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,1]; % 跟随者能接受到的领导者信息总和

A_LF = [1,0,0,0,0,0;
         0,1,0,0,0,0;
         0,0,1,0,0,0;
         0,0,0,1,0,0;
         0,0,0,0,1,0;
         0,0,0,0,0,1]; % 领导者和跟随者的耦合邻接矩阵


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
theta_history = zeros(dmax, num_follower);
v_history = zeros(dmax, num_follower);
omega_history = zeros(dmax, num_follower);
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
            % hat_ex(i) = hat_ex(i) - A_F(i, j) * (x(i) - x(j));
            % hat_ey(i) = hat_ey(i) - A_F(i, j) * (y(i) - y(j));
            hat_ex(i) = hat_ex(i) - A_F(i, j) * (x(i) - x_history(end - d(i, j) + 1, j));
            hat_ey(i) = hat_ey(i) - A_F(i, j) * (y(i) - y_history(end - d(i, j) + 1, j));
            % hat_ex(i) = hat_ex(i) - A_F(i, j) * (x_history(end - d(i, j) + 1, i) - x_history(end - d(i, j) + 1, j));
            % hat_ey(i) = hat_ey(i) - A_F(i, j) * (y_history(end - d(i, j) + 1, i) - y_history(end - d(i, j) + 1, j));
        end
        for j = 1:num_leader
            hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x(i) - rx(j));
            hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y(i) - ry(j));
            % hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x(i) - rx_history(end - d(i, j) + 1, j));
            % hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y(i) - ry_history(end - d(i, j) + 1, j));
            % hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x_history(end - d(i, j) + 1, i) - rx_history(end - d(i, j) + 1, j));
            % hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y_history(end - d(i, j) + 1, i) - ry_history(end - d(i, j) + 1, j));
            dot_dx(i) = dot_dx(i) + A_LF(i, j) * dot_rx(1);
            dot_dy(i) = dot_dy(i) + A_LF(i, j) * dot_ry(1);
        end
        % dot_dx(i) = dot_rx(1);
        % dot_dy(i) = dot_ry(1);
        if B(i,i) ~= 0
            dot_dx(i) = 1/B(i,i)* dot_rx(1);
            dot_dy(i) = 1/B(i,i)* dot_ry(1);
        end
        hat_evx(i) = dot_rx(1) - v(i) * sin(theta(i));
        hat_evy(i) = dot_ry(1) - v(i) * cos(theta(i));
        ux(i) = dot_dx(i) + k1 * hat_ex(i);
        uy(i) = dot_dy(i) + k2 * hat_ey(i);
    end

    errx_actual = L1 * x' + L2 * rx';
    erry_actual = L1 * y' + L2 * ry';

    % 系统方程
    x = x + ux * T;
    y = y + uy * T;
    rx = rx + dot_rx * T;
    ry = ry + dot_ry * T;

    x_history = [x_history; x];
    y_history = [y_history; y];
    theta_history = [theta_history; theta];
    rx_history = [rx_history; rx];
    ry_history = [ry_history; ry];
    hat_ex_history = [hat_ex_history; -hat_ex];
    hat_ey_history = [hat_ey_history; -hat_ey];
    errx_actual_history = [errx_actual_history; errx_actual'];
    erry_actual = [erry_actual_history; erry_actual'];
    hat_evx_history = [hat_evx_history; hat_evx];
    hat_evy_history = [hat_evy_history; hat_evy];


    if mod(k,1000)==0
        % 计算进度
        progress = k / iter * 100;    
        % 打印进度信息
        fprintf('进度：%0.2f%%\n', progress);
    end
end

