% nonlinear system
clear
% 随机种子
rng(88);

% 迭代设置
t_sum = 10;%50
T = 0.001;
iter = floor(t_sum / T);
num_follower = 6;
num_leader = 4;
% num_leader = 6;

% 初值
x = zeros(1, num_follower) + 2 * rand(1, num_follower);
y = zeros(1, num_follower) + 2 * rand(1, num_follower);
theta = -pi + 2 * pi * rand(1, num_follower);
dot_x = zeros(1, num_follower);
dot_y = zeros(1, num_follower);
ddot_hat_theta_d = [0,0,0,0,0,0];
dot_hat_theta_d = [0,0,0,0,0,0];
hat_theta_d = [0,0,0,0,0,0];
rx = 1 + 2 * rand(1, num_leader);
ry = 1 + 2 * rand(1, num_leader);
rx = [2 4 4 2];
ry = [2 2 4 4];
dot_rx = 0.25 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);
dot_ry = 0.25 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);
x = zeros(1, num_follower) + 4 * rand(1, num_follower);
y = zeros(1, num_follower) + 4 * rand(1, num_follower);

% leader静止
% dot_rx = 0 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);
% dot_ry = 0 * ones(1, num_leader);%5 * (1 / T / iter) * ones(1, num_leader);

% 系数
k1 = 1;
k2 = 1;
k3 = 2;
R = 100;
% follower不会和leader碰撞的网络拓扑
% L1 = [2,  0,  0,  0,  0,  0;
%      -1,  2,  0,  0,  0,  0;
%       0, -1,  3, -1,  0,  0;
%       0,  0,  0,  2,  0,  0;
%       0, -1,  0,  0,  2,  0;
%       0,  0, -1,  0, -1,  2 ]; % 跟随者邻接矩阵
L1 = [2,  0,  0,  0,  0,  0;
     -1,  2,  0,  0,  0,  0;
      0, -1,  3, -1,  0,  0;
      0,  0,  0,  2,  0,  0;
      0, -1,  0,  0,  2,  0;
      -1,  0, -1,  0, 0,  2 ]; % 跟随者邻接矩阵
B = diag(diag(L1));
A_F = B-L1;
% L2 =[ -1,  0,  0, -1;
%         0, -1,  0,  0;
%         0,  0, -1,  0;
%         0, -1,  0, -1;
%         0,  0, -1,  0;
%         0,  0,  0,  0];
L2 =[ -1,  0,  0, -1;
		0, -1,  0,  0;
		0,  0, -1,  0;
		0,  0,  -1, -1;
		-1,  0, 0,  0;
		0,  0,  0,  0];
A_LF = -L2;
xishu = inv(L1) * L2;
%%
% 时延设置
dmax = 500;

% 初始化变量
ux = zeros(1, num_follower);
uy = zeros(1, num_follower);
theta_d = zeros(1, num_follower);
theta_d1 = zeros(1, num_follower);
kk = zeros(1, num_follower);
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

for k = 1:iter%/2
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
            % hat_ex(i) = hat_ex(i) + A_F(i, j)/L1(i,i) * (x(i) - x_history(end - d(i, j) + 1, j));
            % hat_ey(i) = hat_ey(i) + A_F(i, j)/L1(i,i) * (y(i) - y_history(end - d(i, j) + 1, j));
            % 都有时延，包括自身
            hat_ex(i) = hat_ex(i) + A_F(i, j)/L1(i,i) * (x_history(end - timedelay(i,j,k,'F') + 1, i) - x_history(end - timedelay(i,j,k,'F') + 1, j));
            hat_ey(i) = hat_ey(i) + A_F(i, j)/L1(i,i) * (y_history(end - timedelay(i,j,k,'F') + 1, i) - y_history(end - timedelay(i,j,k,'F') + 1, j));
            % 用计算的位置来代替现在的位置
            % hat_ex(i) = hat_ex(i) - A_F(i, j) * (x(i) - x_history(end - d(i, j) + 1, j) - ...
            %     v_history(end - d(i, j) + 1, j) * cos(theta_history(end - d(i, j) + 1, j)) * d(i, j) * T);
            % hat_ey(i) = hat_ey(i) - A_F(i, j) * (y(i) - y_history(end - d(i, j) + 1, j) - ...
            %     v_history(end - d(i, j) + 1, j) * sin(theta_history(end - d(i, j) + 1, j)) * d(i, j) * T);

            dot_dx(i) = dot_dx(i) + A_F(i, j)/L1(i,i) * dot_x_history(end - timedelay(i,j,k,'F') + 1, j);
            dot_dy(i) = dot_dy(i) + A_F(i, j)/L1(i,i) * dot_y_history(end - timedelay(i,j,k,'F') + 1, j);
        end
        for j = 1:num_leader
            % hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x(i) - rx(j));
            % hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y(i) - ry(j));
            % hat_ex(i) = hat_ex(i) + A_LF(i, j)/L1(i,i) * (x(i) - rx_history(end - d(i, j) + 1, j));
            % hat_ey(i) = hat_ey(i) + A_LF(i, j)/L1(i,i) * (y(i) - ry_history(end - d(i, j) + 1, j));
            % 都有时延，包括自身
            hat_ex(i) = hat_ex(i) + A_LF(i, j)/L1(i,i) * (x_history(end - timedelay(i,j,k,'L') + 1, i) - rx_history(end - timedelay(i,j,k,'L') + 1, j));
            hat_ey(i) = hat_ey(i) + A_LF(i, j)/L1(i,i) * (y_history(end - timedelay(i,j,k,'L') + 1, i) - ry_history(end - timedelay(i,j,k,'L') + 1, j));
            % 用计算的位置来代替现在的位置
            % hat_ex(i) = hat_ex(i) - A_LF(i, j) * (x(i) - rx_history(end - d(i, j) + 1, j) - dot_rx(j) * d(i, j) * T);
            % hat_ey(i) = hat_ey(i) - A_LF(i, j) * (y(i) - ry_history(end - d(i, j) + 1, j) - dot_ry(j) * d(i, j) * T);
            dot_dx(i) = dot_dx(i) + A_LF(i, j)/L1(i,i) * dot_rx(j);
            dot_dy(i) = dot_dy(i) + A_LF(i, j)/L1(i,i) * dot_ry(j);
        end
        % hat_ex(i) = 1/sum(A_F(i, :)) * hat_ex(i);
        % hat_ey(i) = 1/sum(A_F(i, :)) * hat_ey(i);
        % dot_dx(i) = mean(dot_rx);
        % dot_dy(i) = mean(dot_ry);
        % if B(i,i) ~= 0
        %     dot_dx(i) = 1/B(i,i)* dot_dx(i);
        %     dot_dy(i) = 1/B(i,i)* dot_dy(i);
        % end
        hat_evx(i) = dot_rx(1) - v(i) * sin(theta(i));
        hat_evy(i) = dot_ry(1) - v(i) * cos(theta(i));
        % ux(i) = dot_dx(i) + k1 * hat_ex(i);
        % uy(i) = dot_dy(i) + k2 * hat_ey(i);
        % int_x = sum(hat_ex_history);
        % int_y = sum(hat_ey_history);
        ux(i) = dot_dx(i) - k1 * hat_ex(i);% - T * int_x(i);
        uy(i) = dot_dy(i) - k2 * hat_ey(i);% - T * int_y(i);
        % ux(i) = - k1 * hat_ex(i) - T * int_x(i);
        % uy(i) = - k2 * hat_ey(i) - T * int_y(i);

        theta_d(i) = atan2(uy(i), ux(i));

        if k == 1
            theta_d1(i) = theta_d(i);
            kk(i) = 0;
        end

        delta = - 0.9 * pi^2;

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

        % --------------------------------加上饱和函数-----------------------------------------
        % de = 0.1;
        % kkk = 1/de;
        % temp = theta(i) - theta_d(i);
        % temp = nthroot(temp^2,3);
        % if abs(temp)>de
        %    sats = sign(temp);
        % else
        %    sats = kkk * (temp);
        % end
        % omega(i) = dot_hat_theta_d(i) - k3 * sign(theta(i) - theta_d(i))*norm(theta(i) - theta_d(i))^(2/3); %- 0.01 * sats;      % 加上饱和函数
    end

    % errx_actual = L1 * x' + L2 * rx';
    % erry_actual = L1 * y' + L2 * ry';
    errx_actual = x' + xishu * rx';
    erry_actual = y' + xishu * ry';
    dot_errx_actual = zeros(1, num_follower);
    dot_erry_actual = zeros(1, num_follower);
    for i = 1:num_follower
        for j = 1:num_follower
            dot_errx_actual(i) = dot_errx_actual(i) - A_F(i, j) * (ux(i) - ux(j));
            dot_erry_actual(i) = dot_erry_actual(i) - A_F(i, j) * (uy(i) - uy(j));
        end
        for j = 1:num_leader
            dot_errx_actual(i) = dot_errx_actual(i) - A_LF(i, j) * (ux(i) - dot_rx(j));
            dot_erry_actual(i) = dot_erry_actual(i) - A_LF(i, j) * (uy(i) - dot_ry(j));
        end
    end
    lyax(k) = errx_actual' * errx_actual + dmax*T*sum(sum(errx_actual_history(end - dmax + 1:end,:).^6));
    lyay(k) = errx_actual' * erry_actual + dmax*T*sum(sum(erry_actual_history(end - dmax + 1:end,:).^2));
    dotlyax(k) = dot_errx_actual * errx_actual;
    dotlyay(k) = dot_erry_actual * erry_actual;
    % 系统方程
    for i = 1:num_follower
        theta(i) = theta(i) + omega(i) * T;
        x(i) = x(i) + v(i) * T * cos(theta(i));    % v和x是N个智能体的速度和x坐标
        y(i) = y(i) + v(i) * T * sin(theta(i));    % y是N个智能体的y坐标
        dot_x(i) = v(i) * cos(theta(i));
        dot_y(i) = v(i) * sin(theta(i));
    end
    for i = 1:num_leader
        rx(i) = rx(i) + dot_rx(i) * T;
        ry(i) = ry(i) + dot_ry(i) * T;
    end

    x_history(k+dmax,:) = x;
    y_history(k+dmax,:) = y;
    dot_x_history(k+dmax,:) = dot_x;
    dot_y_history(k+dmax,:) = dot_y;
    theta_history(k+dmax,:) = theta;
    rx_history(k+dmax,:) = rx;
    ry_history(k+dmax,:) = ry;
    v_history(k,:) = v;
    omega_history(k,:) = omega;
    hat_ex_history(k,:) = hat_ex;
    hat_ey_history(k,:) = hat_ey;
    errx_actual_history(k,:) = errx_actual';
    erry_actual_history(k,:) = erry_actual';
    hat_evx_history(k,:) = hat_evx;
    hat_evy_history(k,:) = hat_evy;

    if mod(k,1000)==0
        % 计算进度
        progress = k / iter * 100;    
        % 打印进度信息
        fprintf('进度：%0.2f%%\n', progress);
        disp(timedelay(1,7,k,'F'))
    end
end
%% test
 timedelay(i,j,k,'F')
%% 函数时滞
function result = timedelay(i,j,k,ForL)
    t=k*0.001;
    result=0.001;
    if ForL=='L'
        j=j+6;
    end
    if i==1 && j==7
        result = 0.5 * abs(sin(t));
    end
    if i==2 && j==8
        result = 0.5 * abs(sin(t));
    end
    if i==1 && j==10
        result = exp(-1-t);
    end
    if i==2 && j==1
        result = exp(-1-t);
    end
    if i==3 && j==9
        result = 0.5/(1+t);
    end
    if i==4 && j==10
        result = 0.5/(1+t);
    end
    if i==3 && j==2
        result = 0.4 * abs(cos(t));
    end
    if i==3 && j==4
        result = 0.4 * abs(cos(t));
    end
    if i==5 && j==2
        result = 0.3/(1+t^2);
    end
    if i==6 && j==1
        result = 0.3/(1+t^2);
    end
    if i==4 && j==9
        result = 0.2/(1+log(1+t));
    end
    if i==5 && j==7
        result = 0.2/(1+log(1+t));
    end
    if i==6 && j==3
        result = 0.2/(1+log(1+t));
    end
    result = floor(result / 0.001);
    if result==0
        result=1;
    end
end

% % 均匀时滞 
% function result = timedelay(i,j,k,ForL)
%     t=k*0.001;
%     result=0.001;
%     if ForL=='L'
%         j=j+6;
%     end
%     if i==1 && j==7
%         result = 0.5 * abs(cos(t));
%     end
%     if i==2 && j==8
%         result = 0.5 * abs(cos(t));
%     end
%     if i==1 && j==10
%         result = 0.5 * abs(cos(t));
%     end
%     if i==2 && j==1
%         result = 0.5 * abs(cos(t));
%     end
%     if i==3 && j==9
%         result = 0.5 * abs(cos(t));
%     end
%     if i==4 && j==10
%         result = 0.5 * abs(cos(t));
%     end
%     if i==3 && j==2
%         result = 0.5 * abs(cos(t));
%     end
%     if i==3 && j==4
%         result = 0.5 * abs(cos(t));
%     end
%     if i==5 && j==2
%         result = 0.5 * abs(cos(t));
%     end
%     if i==6 && j==5
%         result = 0.5 * abs(cos(t));
%     end
%     if i==4 && j==8
%         result = 0.5 * abs(cos(t));
%     end
%     if i==5 && j==9
%         result = 0.5 * abs(cos(t));
%     end
%     if i==6 && j==3
%         result = 0.5 * abs(cos(t));
%     end
%     result = floor(result / 0.001);
%     if result==0
%         result=1;
%     end
% end