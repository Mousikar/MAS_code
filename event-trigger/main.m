clear all
close all

x = [0 0 0 0 0 6];  % 初始状态
L = [3 0 0 -1 -1 -1;
    -1 1 0 0 0 0;
    -1 -1 2 0 0 0;
    -1 0 0 1 0 0;
    0 0 0 -1 1 0;
    0 0 0 0 -1 1];  % laplace矩阵
dt = 0.01;
Ts = 0:dt:10;
xhat = x;
x1 = [];
x1hat = [];
s = 1;
E = [];
alpha = 0.9;
c1 = 1.1;
ef = [];
M = [];
U = [];
T1 = [];
for t = 0:dt:10
    ef1 = [];
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
end