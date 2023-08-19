clear 
% close all

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
Ts = 0:dt:10;
G = expm(A*dt);
syms s
H = int(expm(A*s),0,dt)*B;
H = eval(H);

c = 1.1;
c1 = 0.5;
alpha = 0.9;
x0 = [0.4, 0.3, 0.5, 0.2, 0.6, 0.1, 0.7, 0, 0.8, -0.1, 0.4, -0.2];  % 初始状态
xtri0 = x0;

M = []; % 记录阈值
e = zeros(1,6);
E = []; % 记录误差
T = []; % 记录时间和触发机器人索引
x_his = []; % 记录理论状态值
xtri_his = []; % 记录触发状态值
u = zeros(1,6);
U = []; % 记录控制输入
for t = 0:dt:5
    matrix_x = reshape(x0, 2, 6);
    matrix_xtri = reshape(xtri0, 2, 6);
    m = c1 * exp(- alpha * t); M = [M; m];
    for i = 1:length(L)
        x = matrix_x(:,i);
        xtri = matrix_xtri(:,i);
        u(i) = c * K(1) * L(i,:) * matrix_xtri(1,:)' + ...
            c * K(2) * L(i,:) * matrix_xtri(2,:)';
        x = G * x + H * u(i);  % 理论值更新
        e(i) = norm(x - xtri);
        fi = e(i) - m;
        if fi > 0
            matrix_xtri(:,i) = x;   % 触发值更新
            T = [T; t, i];
        end
        matrix_x(:,i) = x;
    end
    x0 = reshape(matrix_x, 1, 12);
    xtri0 = reshape(matrix_xtri, 1, 12);
    E = [E; e];
    U = [U; u];
    x_his = [x_his; x0];
    xtri_his = [xtri_his; xtri0];
end

%% 画图
figure(1)
plot(0:dt:5,x_his(:,1:2:12),'r--', LineWidth=2)
hold on
plot(0:dt:5,xtri_his(:,1:2:12),'k', LineWidth=2)
xlabel('time');ylabel('x_1');

figure(2)
plot(0:dt:5,x_his(:,2:2:12),'r--', LineWidth=2)
hold on
plot(0:dt:5,xtri_his(:,2:2:12),'k', LineWidth=2)
xlabel('time');ylabel('x_2');

figure(3)
plot(T(:,1),T(:,2),'ro',MarkerFaceColor='r')

figure(4)
for i = 1:length(L)
    subplot(3,2,i)
    plot(0:dt:5,M,'r--', LineWidth=2)
    hold on
    plot(0:dt:5,E(:,i),'k', LineWidth=2)
end

figure(5)
plot(0:dt:5,U, LineWidth=2)
legend('agent1', 'agent2', 'agent3', 'agent4', 'agent5', 'agent6')