% leader formation control
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
kk = zeros(1, num_leader);
v_L = zeros(1, num_leader);
omega_L = zeros(1, num_leader);
ddot_hat_theta_d = [0,0,0,0];
dot_hat_theta_d = [0,0,0,0];
hat_theta_d = [0,0,0,0];
% 时延
tau_L = [10, 40, 20, 30]; % 根据实际情况填充
tau = repmat(tau_L, num_leader, 1) / 1000; % 初始化时延
d_L = floor(tau / T);
d_Lmax = max(max(d_L));
% 历史变量初始化
rx_star_his = zeros(iter+d_Lmax,num_leader);
ry_star_his = zeros(iter+d_Lmax,num_leader);
dot_rx_star_his = zeros(iter+d_Lmax,num_leader);
dot_ry_star_his = zeros(iter+d_Lmax,num_leader);
rx_his = zeros(iter+d_Lmax,num_leader);
ry_his = zeros(iter+d_Lmax,num_leader);
rtheta_his = zeros(iter+d_Lmax,num_leader);
dot_rx_his = zeros(iter+d_Lmax,num_leader);
dot_ry_his = zeros(iter+d_Lmax,num_leader);
u_rx_his = zeros(iter+d_Lmax,num_leader);
u_yx_his = zeros(iter+d_Lmax,num_leader);
v_L_his = zeros(iter+d_Lmax,num_leader);
omega_L_his = zeros(iter+d_Lmax,num_leader);
errorx_L_his = zeros(iter+d_Lmax,num_leader);
errory_L_his = zeros(iter+d_Lmax,num_leader);
for k = 1:d_Lmax
    rx_star_his(k, :) = rx_star;
    ry_star_his(k, :) = ry_star;
    rx_his(k, :) = rx;
    ry_his(k, :) = ry;
    rtheta_his(k, :) = rtheta;
end
% leader期望轨迹，在20——25秒有个变化，队形缩小，在40转弯
for k =  1 + d_Lmax:iter + d_Lmax
    if k <= iter/3 + d_Lmax
        dot_rx_star = 0.25 * ones(1,num_leader);
        dot_ry_star = 0.5 * ones(1,num_leader);
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3 + d_Lmax && k<= iter/3+5/T + d_Lmax
        dot_rx_star = 0.25 * ones(1,num_leader) + [-0.1,+0.1,+0.1,-0.1];
        dot_ry_star = 0.5 * ones(1,num_leader) + [-0.1,-0.1,+0.1,+0.1];
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3+5/T + d_Lmax && k<= iter/3*2 + d_Lmax
        dot_rx_star = 0.25 * ones(1,num_leader);
        dot_ry_star = 0.5 * ones(1,num_leader);
        rx_star = rx_star + dot_rx_star * T;
        ry_star = ry_star + dot_ry_star * T;
    end
    if k > iter/3*2 + d_Lmax
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
% 系数
k_L = 1;
k_theta = 3;
R=100;
%% 迭代
for k = 1 + d_Lmax:iter + d_Lmax
    for i = 1:num_leader
        u_rx(i) = dot_rx_star_his(k,i) - k_L * rx(i) + k_L * rx_star_his(k,i);
        u_ry(i) = dot_ry_star_his(k,i) - k_L * ry(i) + k_L * ry_star_his(k,i);
        for j = 1:num_leader
        u_rx(i) = u_rx(i) - k_L * A_L(i, j) * ( (rx_his(k - d_L(i, j), i) - rx_star_his(k - d_L(i, j), i) ) - ( rx_his(k - d_L(i, j), j) - rx_star_his(k - d_L(i, j), j) ));
        u_ry(i) = u_ry(i) - k_L * A_L(i, j) * ( (ry_his(k - d_L(i, j), i) - ry_star_his(k - d_L(i, j), i) ) - ( ry_his(k - d_L(i, j), j) - ry_star_his(k - d_L(i, j), j) ));
        end

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

        ddot_hat_theta_d(i) = - R^2 * (hat_theta_d(i) - theta_Ld(i)) - 2 * R * dot_hat_theta_d(i);  % 线性二阶微分器
        dot_hat_theta_d(i) = dot_hat_theta_d(i) + ddot_hat_theta_d(i) * T;
        hat_theta_d(i) = hat_theta_d(i) + dot_hat_theta_d(i) * T;
        
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
        omega_L(i) = dot_hat_theta_d(i) - k_theta * sign(rtheta(i) - theta_Ld(i))*norm(rtheta(i) - theta_Ld(i))^(2/3); %- 0.01 * sats;      % 加上饱和函数
    end
    errorx_L = L_L * (rx-rx_star_his(k,:))';
    errory_L = L_L * (ry-ry_star_his(k,:))';
    for i = 1:num_leader
        rtheta(i) = rtheta(i) + omega_L(i) * T;
        rx(i) = rx(i) + v_L(i) * cos(rtheta(i)) * T;
        ry(i) = ry(i) + v_L(i) * sin(rtheta(i)) * T;
    end
    rx_his(k,:) = rx;
    ry_his(k,:) = ry;
    v_L_his(k,:) = v_L;
    omega_L_his(k,:) = omega_L;
    errorx_L_his(k,:) = errorx_L;
    errory_L_his(k,:) = errory_L;
end

%% 画图验证
plot(rx_star_his,ry_star_his,'r-')
hold on
plot(rx_his,ry_his,'b--')
axis equal