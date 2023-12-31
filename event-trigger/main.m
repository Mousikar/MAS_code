clear all
close all

x = [1 2 3 4 5 6];  % 初始状态
L = [3 0 0 -1 -1 -1;
    -1 1 0 0 0 0;
    -1 -1 2 0 0 0;
    -1 0 0 1 0 0;
    0 0 0 -1 1 0;
    0 0 0 0 -1 1];  % laplace矩阵
dt = 0.01;
Ts = 0:dt:10;
xhat = x;
x1 = [];    % 记录所有的正常状态
x1hat = []; % 记录所有的经过触发之后的状态
s = 1;
E = [];     % 记录所有的误差
alpha = 0.9;
c1 = 1.1;
ef = [];    % 记录所有的fi
M = [];     % 记录每次的m = c1 * exp(-alpha * t)
U = [];     % 记录每次的控制输入
T1 = [];    % 记录事件出发的时刻
for t = 0:dt:10
    ef1 = [];% 记录一次的fi
    % 储存数据
    x1 = [x1;x];
    x1hat = [x1hat;xhat];
    e = [];
    % 1. 计算测量误差 e = xhat - x
    for i = 1:length(x)
        e1 = x1hat(s,i) - x1(s,i);
        e = [e e1];
    end
    % 储存误差
    E = [E;e];
    % 2. 计算事件触发函数
    m = c1 * exp(-alpha * t);
    for i = 1:length(x)        
        f1 = norm(e(i)) - m;
        ef1 = [ef1 f1];
    % 4. 判断是否发生事件触发，如果是，更新状态
        if f1>=0
            xhat=x;
            num1=s;
            T1=[T1;num1]
        end
    end
    ef = [ef;ef1];
    M = [M m];
    % 3. 更新状态
    u = -L * x1hat(s,:)';
    u=u.';
    x=x+dt*u;
    U = [U;u];
    s=s+1;
end
%% 画图
subplot(311)
plot(x1hat,'k','LineWidth',2)
hold on;
plot(x1,'r-.','LineWidth',2)
title('状态变量')

subplot(312)
plot(U,'LineWidth',2)
title('控制输入')

subplot(313)
plot(M,'r','LineWidth',2)
hold on;
plot(ef,'b','LineWidth',2)
plot(E,'k','LineWidth',2)