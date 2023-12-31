clear 
% close all
clc
A = [-2, 2;
     -1, 1];
B = [1;
     0];
K = [-1, 2];
L = [3 0 0 -1 -1 -1;
    -1 1 0 0 0 0;
    -1 -1 2 0 0 0;
    -1 0 0 1 0 0;
    0 0 0 -1 1 0;
    0 0 0 0 -1 1];  % laplace矩阵
dt = 0.01;
Ts = 0:dt:5;
G = expm(A*dt);
syms s
H = int(expm(A*s),0,dt)*B;
H = eval(H);

c = 1.1;
c1 = 0.5;
alpha = 0.9;
x0 = [0.4, 0.3, 0.5, 0.2, 0.6, 0.1, 0.7, 0, 0.8, -0.1, 0.4, -0.2];  % 初始状态
xtri0 = x0;

T = []; % 记录时间和触发机器人索引
x_his = []; % 记录理论状态值
xtri_his = []; % 记录触发状态值
u = zeros(1,6);
U = []; % 记录控制输入
xi_ = zeros(1,6);
t_ki = zeros(1,6);
flag1=zeros(1,6);
xi_ini=[0.1;0.1;0.1;0.1;0.1;0.1];
for t = Ts
    matrix_x = reshape(x0, 2, 6);
    matrix_xtri = reshape(xtri0, 2, 6);
    flag2=flag1;
    flag1=zeros(1,6);
    for i = 1:length(L)
        x = matrix_x(:,i);
        xtri = matrix_xtri(:,i);
        u(i) = c * K(1) * L(i,:) * matrix_x(1,:)' + ...
            c * K(2) * L(i,:) * matrix_x(2,:)';
        x = G * x + H * u(i);  % 理论值更新

        % 解方程
        if t == 0 || isempty(T)
            t_ki(i) = 0;
        else
            % temp=T(T(:,2)==i,1);
            % t_ki(i) = temp(end);
            t_ki(i) = T(end,1);
        end
        if abs(t-(t_ki(i)+xi_(i))) < 5*dt
        % if abs(t-(t_ki(i)+xi_(i))) < 3*dt
            fi = @(xi) norm(expm(A*xi)*xtri - x) - c1 * exp(- alpha * (t_ki + xi));
            xi_ini = 0.1;
            xi_(i) = fsolve(fi, xi_ini);
            matrix_xtri(:,i) = x;   % 触发值更新
            T = [T; t, i];
            flag1(i)=1;
        end
        if xi_(i) <= 0 || sum(flag2) ~= 0
            fi = @(xi) norm(expm(A*xi)*xtri - x) - c1 * exp(- alpha * (t_ki + xi));
            xi_ini = 0.1;
            xi_(i) = fsolve(fi, xi_ini)
        end


        % % 解方程
        % if (t_ki(i)+xi_(i))-t <= -0.5*dt && sum(flag2) == 0 %0.7*dt
        %     fi = @(xi) norm(expm(A*xi)*xtri - x) - c1 * exp(- alpha * (t_ki(i) + xi));
        %     xi_(i) = fsolve(fi, xi_ini(i));
        %     matrix_xtri(:,i) = x;   % 触发值更新
        %     T = [T; t, i];
        %     flag1(i)=1;
        %     t_ki(i) = T(end,1);
        % end
        % if xi_(i) < 0 || sum(flag2) ~= 0
        %     fi = @(xi) norm(expm(A*xi)*xtri - x) - c1 * exp(- alpha * (t_ki(i) + xi));
        %     xi_(i) = fsolve(fi, xi_ini(i));
        %     t_ki(i) = T(end,1)+dt;
        % end

        matrix_x(:,i) = x;
    end
    x0 = reshape(matrix_x, 1, 12);
    xtri0 = reshape(matrix_xtri, 1, 12);
    U = [U; u];
    x_his = [x_his; x0];
    xtri_his = [xtri_his; xtri0];
end
T
%% 画图
figure(1)
plot(Ts,x_his(:,1:2:12),'r--', LineWidth=2)
hold on
plot(Ts,xtri_his(:,1:2:12),'k', LineWidth=2)
xlabel('time');ylabel('x_1');

figure(2)
plot(Ts,x_his(:,2:2:12),'r--', LineWidth=2)
hold on
plot(Ts,xtri_his(:,2:2:12),'k', LineWidth=2)
xlabel('time');ylabel('x_2');

figure(3)
plot(T(:,1),T(:,2),'ro',MarkerFaceColor='r')

% figure(4)
% plot(0:dt:5,U, LineWidth=2)
% legend('agent1', 'agent2', 'agent3', 'agent4', 'agent5', 'agent6')
        %% 解方程
        % syms xi
        % % assume(xi, 'real');
        % e = norm(expm(A*xi)*xtri - xtri);
        % if t == 0 || isempty(T)
        %     t_ki = 0;
        % else
        %     t_ki = T(end,1);
        % end
        % m = c1 * exp(- alpha * (t_ki + xi));
        % fi = e - m == 0;
        % solutions = solve(fi, xi)
        % xi_ = eval(solutions);