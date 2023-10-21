% leader formation control and follower containment
clear
% 随机种子
rng(100);

% 迭代设置
t_sum = 60;
T = 0.001;
iter = floor(t_sum / T);
num_follower = 6;
num_leader = 4;

% 初值
rx_star = [2 3 3 2];
ry_star = [2 2 3 3];
rx = 1 + 2 * rand(1, num_leader);
ry = 1 + 2 * rand(1, num_leader);
rtheta =  -pi + 2 * pi * rand(1, num_leader);
u_rx = zeros(1,num_leader);
u_ry = zeros(1,num_leader);
theta_Ld = zeros(1, num_leader);
theta_d1 = zeros(1, num_leader);
theta_d2 = zeros(1, num_follower);
kk = zeros(1, num_leader);
kkkk = zeros(1, num_follower);
v_L = zeros(1, num_leader);
omega_L = zeros(1, num_leader);
ddot_hat_theta_d1 = [0,0,0,0];
dot_hat_theta_d1 = [0,0,0,0];
hat_theta_d1 = [0,0,0,0];
ddot_hat_theta_d2 = [0,0,0,0,0,0];
dot_hat_theta_d2 = [0,0,0,0,0,0];
hat_theta_d2 = [0,0,0,0,0,0];

v = zeros(1, num_follower);
omega = zeros(1, num_follower);

x = zeros(1, num_follower) + 2 * rand(1, num_follower);
y = zeros(1, num_follower) + 2 * rand(1, num_follower);
theta = -pi + 2 * pi * rand(1, num_follower);
theta_d = zeros(1, num_follower);

ux = zeros(1,num_follower);
uy = zeros(1,num_follower);
% 时延
tau_F = [10, 40, 20, 30, 50, 60]; % 根据实际情况填充
tau = repmat(tau_F, num_follower, 1) / 1000; % 初始化时延
d_F = floor(tau / T);
d_Fmax = max(max(d_F));
% 历史变量初始化
rx_star_his = zeros(iter+d_Fmax,num_leader);
ry_star_his = zeros(iter+d_Fmax,num_leader);
dot_rx_star_his = zeros(iter+d_Fmax,num_leader);
dot_ry_star_his = zeros(iter+d_Fmax,num_leader);
rx_his = zeros(iter+d_Fmax,num_leader);
ry_his = zeros(iter+d_Fmax,num_leader);
rtheta_his = zeros(iter+d_Fmax,num_leader);
dot_rx_his = zeros(iter+d_Fmax,num_leader);
dot_ry_his = zeros(iter+d_Fmax,num_leader);
u_rx_his = zeros(iter+d_Fmax,num_leader);
u_yx_his = zeros(iter+d_Fmax,num_leader);
v_L_his = zeros(iter+d_Fmax,num_leader);
omega_L_his = zeros(iter+d_Fmax,num_leader);
errorx_L_his = zeros(iter+d_Fmax,num_leader);
errory_L_his = zeros(iter+d_Fmax,num_leader);

x_his = zeros(iter+d_Fmax,num_follower);
y_his = zeros(iter+d_Fmax,num_follower);
theta_his = zeros(iter+d_Fmax,num_follower);
v_his = zeros(iter+d_Fmax,num_follower);
omega_his = zeros(iter+d_Fmax,num_follower);
errorx_F_his = zeros(iter+d_Fmax,num_follower);
errory_F_his = zeros(iter+d_Fmax,num_follower);

for k = 1:d_Fmax
    rx_star_his(k, :) = rx_star;
    ry_star_his(k, :) = ry_star;
    rx_his(k, :) = rx;
    ry_his(k, :) = ry;
    x_his(k, :) = x;
    y_his(k, :) = y;
end
% leader期望轨迹，在20——25秒有个变化，队形缩小，在40转弯
for k =  1 + d_Fmax:iter + d_Fmax
    if k <= iter/3 + d_Fmax
        dot_rx_star = 0.25 * ones(1,num_leader);
        dot_ry_star = 0.5 * ones(1,num_leader);
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3 + d_Fmax && k<= iter/3+5/T + d_Fmax
        dot_rx_star = 0.25 * ones(1,num_leader) + [-0.1,+0.1,+0.1,-0.1];
        dot_ry_star = 0.5 * ones(1,num_leader) + [-0.1,-0.1,+0.1,+0.1];
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3+5/T + d_Fmax && k<= iter/3*2 + d_Fmax
        dot_rx_star = 0.25 * ones(1,num_leader);
        dot_ry_star = 0.5 * ones(1,num_leader);
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3*2 + d_Fmax
        dot_rx_star = 0.25 * ones(1,num_leader);
        dot_ry_star = zeros(1,num_leader);
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    rx_star_his(k,:) = rx_star;
    ry_star_his(k,:) = ry_star;
    dot_rx_star_his(k,:) = dot_rx_star;
    dot_ry_star_his(k,:) = dot_ry_star;
end
%% leader formation control
% 拓扑
A_L = [0,0,0,1;
       1,0,0,0;
       0,1,0,0;
       0,0,1,0;];
L_L = -A_L;
for i = 1:num_leader
    L_L(i,i) = sum(A_L(i,:));
end
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
L_ = L1\L2;
% 系数
k_L = 2;
k_F = k_L;
k_theta = 3;
R=100;
%% 迭代
for k = 1 + d_Fmax:iter + d_Fmax
    % leader 编队控制
    for i = 1:num_leader
        u_rx(i) = dot_rx_star_his(k,i) - k_L * ( rx(i) - rx_star_his(k,i) );
        u_ry(i) = dot_ry_star_his(k,i) - k_L * ( ry(i) - ry_star_his(k,i) );

        theta_Ld(i) = atan2(u_ry(i), u_rx(i));

        if k == 1
            theta_d1(i) = theta_Ld(i);
            kk(i) = 0;
        end

        delta = - 0.9 * pi^2;

        if theta_Ld(i) * theta_d1(i) < delta
            if theta_Ld(i) < 0
                kk(i) = kk(i) + 1;
            else
                kk(i) = kk(i) - 1;
            end
        end

        theta_d1(i) = theta_Ld(i);
        theta_Ld(i) = theta_Ld(i) + 2 * pi * kk(i);
        
        v_L(i) = sqrt(u_ry(i)^2 + u_rx(i)^2);

        ddot_hat_theta_d1(i) = - R^2 * (hat_theta_d1(i) - theta_Ld(i)) - 2 * R * dot_hat_theta_d1(i);  % 线性二阶微分器
        dot_hat_theta_d1(i) = dot_hat_theta_d1(i) + ddot_hat_theta_d1(i) * T;
        hat_theta_d1(i) = hat_theta_d1(i) + dot_hat_theta_d1(i) * T;
        
        % omega_L(i) = dot_hat_theta_d(i) + k_theta * (theta_Ld(i) - rtheta(i));  % 暂时不加上饱和函数

        % --------------------------------加上饱和函数-----------------------------------------
        de = 0.1;
        kkk = 1/de;
        temp = rtheta(i) - theta_Ld(i);
        temp = nthroot(temp^2,3);
        if abs(temp)>de
           sats = sign(temp);
        else
           sats = kkk * (temp);
        end
        omega_L(i) = dot_hat_theta_d1(i) - k_theta * sign(rtheta(i) - theta_Ld(i))*norm(rtheta(i) - theta_Ld(i))^(2/3); %- 0.01 * sats;      % 加上饱和函数
    end
    errorx_L = L_L * (rx-rx_star_his(k,:))';
    errory_L = L_L * (ry-ry_star_his(k,:))';

    % follow 合围控制
    for i = 1:num_follower
        ux(i) = - L_(i,:) * ( dot_rx_star_his(k,:) + k_L * rx_star_his(k,:) )' - k_F * x(i);
        uy(i) = - L_(i,:) * ( dot_ry_star_his(k,:) + k_L * ry_star_his(k,:) )' - k_F * y(i);
        for j = 1:num_follower
            ux(i) = ux(i) - k_F * A_F(i, j) * (x_his(k - d_F(i, j), i) - x_his(k - d_F(i, j), j));
            uy(i) = uy(i) - k_F * A_F(i, j) * (y_his(k - d_F(i, j), i) - y_his(k - d_F(i, j), j));
        end
        for j = 1:num_leader
            ux(i) = ux(i) - k_F * A_LF(i, j) * (x_his(k - d_F(i, j), i) - rx_his(k - d_F(i, j), j));
            uy(i) = uy(i) - k_F * A_LF(i, j) * (y_his(k - d_F(i, j), i) - ry_his(k - d_F(i, j), j));
        end

        theta_d(i) = atan2(uy(i), ux(i));

        if k == 1
            theta_d2(i) = theta_d(i);
            kkkk(i) = 0;
        end

        delta = - 0.9 * pi^2;

        if theta_d(i) * theta_d2(i) < delta
            if theta_d(i) < 0
                kkkk(i) = kkkk(i) + 1;
            else
                kkkk(i) = kkkk(i) - 1;
            end
        end

        theta_d2(i) = theta_d(i);
        theta_d(i) = theta_d(i) + 2 * pi * kkkk(i);
        
        v(i) = sqrt(uy(i)^2 + ux(i)^2);

        ddot_hat_theta_d2(i) = - R^2 * (hat_theta_d2(i) - theta_d(i)) - 2 * R * dot_hat_theta_d2(i);  % 线性二阶微分器
        dot_hat_theta_d2(i) = dot_hat_theta_d2(i) + ddot_hat_theta_d2(i) * T;
        hat_theta_d2(i) = hat_theta_d2(i) + dot_hat_theta_d2(i) * T;
        
        % omega(i) = dot_hat_theta_d(i) + k_theta * (theta_d(i) - theta(i));  % 暂时不加上饱和函数

        % --------------------------------加上饱和函数-----------------------------------------
        de = 0.1;
        kkk = 1/de;
        temp = theta(i) - theta_d(i);
        temp = nthroot(temp^2,3);
        if abs(temp)>de
           sats = sign(temp);
        else
           sats = kkk * (temp);
        end
        omega(i) = dot_hat_theta_d2(i) - k_theta * sign(theta(i) - theta_d(i))*norm(theta(i) - theta_d(i))^(2/3); %- 0.01 * sats;      % 加上饱和函数

    end
    errorx_F = x' + L_ * rx';
    errory_F = y' + L_ * ry';

    % 系统方程
    for i = 1:num_leader
        rtheta(i) = rtheta(i) + omega_L(i) * T;
        rx(i) = rx(i) + v_L(i) * cos(rtheta(i)) * T;
        ry(i) = ry(i) + v_L(i) * sin(rtheta(i)) * T;
    end
    for i = 1:num_follower
        theta(i) = theta(i) + omega(i) * T;
        x(i) = x(i) + v(i) * T * cos(theta(i));    % v和x是N个智能体的速度和x坐标
        y(i) = y(i) + v(i) * T * sin(theta(i));    % y是N个智能体的y坐标
    end
    % 记录历史变量
    rx_his(k,:) = rx;
    ry_his(k,:) = ry;
    rtheta_his(k,:) = rtheta;
    v_L_his(k,:) = v_L;
    omega_L_his(k,:) = omega_L;
    errorx_L_his(k,:) = errorx_L;
    errory_L_his(k,:) = errory_L;
    x_his(k,:) = x;
    y_his(k,:) = y;
    theta_his(k,:) = theta;
    v_his(k,:) = v;
    omega_his(k,:) = omega;
    errorx_F_his(k,:) = errorx_F;
    errory_F_his(k,:) = errory_F;
    if mod(k,1000)==0
        % 计算进度
        progress = k / iter * 100;    
        % 打印进度信息
        fprintf('进度：%0.2f%%\n', progress);
    end
end

%% 画图验证
plot(rx_star_his,ry_star_his,'r-')
hold on
plot(rx_his,ry_his,'b--')
plot(x_his,y_his,'g--')
axis equal