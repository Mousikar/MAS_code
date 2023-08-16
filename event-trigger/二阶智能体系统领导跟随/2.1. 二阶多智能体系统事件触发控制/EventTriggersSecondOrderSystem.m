%% 基于事件触发机制的二阶多智能体的平均一致性仿真
%FileName: EventTriggersSecondOrderSystem
%Author  : 
%Date    : 2017年10月31日 15:17:07
%Version ：V1.1.0

%% EventTriggersSecondOrderSystem
function EventTriggersSecondOrderSystem()

clc;
clear;
close all;

%% 初始化系统参数
global  A
A= [0  1  0  0 1  1;
    1 0  1  0  0  0;
    0 1  0  1  1  0;
    0  0  1  0 1  0;
    1  0  1  1 0  1;
    1  0  0  0 1  0];%邻接矩阵

L = diag(sum(A)) - A;%图G的拉普拉斯矩阵

[~,D]=eig(L);%求拉普拉斯矩阵的特征值，其中D是由特征值构成的对角矩阵
tempArray = sort(D*ones(length(L),1));%tempArray是排好序的向量
lambda2 = tempArray(2, 1);%求得代数连通度(也就是次小特征值)

%初始状态更换
x0=[-4; 6; 1; 16.2; 2.4; 8];%四个智能体的位置初始状态
v0=[-0.5; 1.3; -3.2; 8.4; 3; 2.2];%四个智能体的位置初始状态

%求位置状态和速度状态的平均值
aver_x0 = sum(x0) / length(x0);
aver_v0 = sum(v0) / length(v0);

%完成一致性分量的提取以及初始化
consistentComponent = zeros(12, 1);%系统的一致性分量分配空间，以及完成初始化
consistentComponent(1:6,:) = aver_x0.*ones(6, 1);
consistentComponent(7:12,:) = aver_v0.*ones(6, 1);

y0(1:6, :) = x0;
y0(7:12, :) = v0;

alpha = 1;
beta = 1;

g = @(t, x)[%二阶系统的微分方程模型
    x(7);%x(7)就是v(1)
    x(8);%x(8)就是v(2)
    x(9);%x(9)就是v(3)
    x(10);%x(10)就是v(4)
    x(11);%x(11)就是v(5)
    x(12);%x(12)就是v(6)
    
    alpha*(-A(1,1)*(x(1)-x(1))-A(1,2)*(x(1) - x(2))-A(1,3)* (x(1)-x(3))-A(1,4)*(x(1) - x(4))-A(1,5)*(x(1) - x(5))-A(1,6)*(x(1) - x(6)))+...
    beta*(-A(1,1)*(x(7)-x(7))-A(1,2)*(x(7) - x(8))-A(1,3)* (x(7)-x(9))-A(1,4)*(x(7) - x(10))-A(1,5)*(x(7) - x(11))-A(1,6)*(x(7) - x(12)));
    
    alpha*(-A(2,1)*(x(2)-x(1))-A(2,2)*(x(2) - x(2))-A(2,3)* (x(2)-x(3))-A(2,4)*(x(2) - x(4))-A(2,5)*(x(2) - x(5))-A(2,6)*(x(2) - x(6)))+...
    beta*(-A(2,1)*(x(8)-x(7))-A(2,2)*(x(8) - x(8))-A(2,3)* (x(8)-x(9))-A(2,4)*(x(8) - x(10))-A(2,5)*(x(8) - x(11))-A(2,6)*(x(8) - x(12)));
    
    alpha*(-A(3,1)*(x(3)-x(1))-A(3,2)*(x(3) - x(2))-A(3,3)* (x(3)-x(3))-A(3,4)*(x(1) - x(4))-A(3,5)*(x(3) - x(5))-A(3,6)*(x(3) - x(6)))+...
    beta*(-A(3,1)*(x(9)-x(7))-A(3,2)*(x(9) - x(8))-A(3,3)* (x(9)-x(9))-A(3,4)*(x(9) - x(10))-A(3,5)*(x(9) - x(11))-A(3,6)*(x(9) - x(12)));
    
    alpha*(-A(4,1)*(x(4)-x(1))-A(4,2)*(x(4) - x(2))-A(4,3)* (x(4)-x(3))-A(4,4)*(x(4) - x(4))-A(4,5)*(x(4) - x(5))-A(4,6)*(x(4) - x(6)))+...
    beta*(-A(4,1)*(x(10)-x(7))-A(4,2)*(x(10) - x(8))-A(4,3)* (x(10)-x(9))-A(4,4)*(x(10) - x(10))-A(4,5)*(x(10) - x(11))-A(4,6)*(x(10) - x(12)));
    
    alpha*(-A(5,1)*(x(5)-x(1))-A(5,2)*(x(5) - x(2))-A(5,3)* (x(5)-x(3))-A(5,4)*(x(5) - x(4))-A(5,5)*(x(5) - x(5))-A(5,6)*(x(5) - x(6)))+...
    beta*(-A(5,1)*(x(11)-x(7))-A(5,2)*(x(11) - x(8))-A(5,3)* (x(11)-x(9))-A(5,4)*(x(11) - x(10))-A(5,5)*(x(11) - x(11))-A(5,6)*(x(11) - x(12)));
    
    alpha*(-A(6,1)*(x(6)-x(1))-A(6,2)*(x(1) - x(2))-A(6,3)* (x(6)-x(3))-A(6,4)*(x(6) - x(4))-A(6,5)*(x(6) - x(5))-A(6,6)*(x(6) - x(6)))+...
    beta*(-A(6,1)*(x(12)-x(7))-A(6,2)*(x(12) - x(8))-A(6,3)* (x(12)-x(9))-A(6,4)*(x(12) - x(10))-A(6,5)*(x(12) - x(11))-A(6,6)*(x(12) - x(12)));
    ];

%下面是单步运行的微分方程的解
timeStart = 0;%仿真的起始时间
timeEnd = 10;%仿真的结束时间
stepLength = 0.05;%在解微分方程组的时候用到的步长
stepNum=floor((timeEnd-timeStart)/stepLength);%求步数floor的作用是将小数转化为整数
X(1)=timeStart;%时间起点
Y(:,1)=y0;%赋初值，可以是向量，但是要注意维数

EX(1) = norm(consistentComponent(1:6, 1));%位置误差变量赋初值
EV(1) = norm(consistentComponent(7:12, 1));%速度误差变量赋初值

inPut = g(1, y0);%初始化控制输入
U(:, 1) = inPut(7:12);

triggerState = Y(:,1);%在系统没有遇到触发事件的时候的暂存系统状态的临时变量
aa = Y( 1 , 1);
for i = 2 : stepNum
    
    X(i)=X(i -1)+stepLength;%更新时间轴
    
    %提取非一致性分量
    delt(1:6,:) = Y(1:6, i - 1) - (aver_x0.*ones(6, 1) + (aver_v0 * X(i)).*ones(6, 1));
    delt(7:12,:) = Y(7:12, i - 1) - consistentComponent(7:12);
    
    e(1:6,:) = Y(1:6, i - 1) - triggerState(1:6, :);%提取X(i)时刻的位置状态误差
    ex = Y(1:6, i - 1) - aa.*ones(6, 1);   
    e(7:12,:) = Y(7:12, i - 1) - triggerState(7:12, :);%提取X(i)时刻的速度状态误差
    
    if EventTriggering(e, delt, L) >=0%如果触发函数满足触发条件，就进行系统的更新
        triggerState = Y( : , i -1);%保存触发状态
        %e(1:6,:) = zeros(6,1);%x状态误差清零
        aa = Y(1, i - 1);
        ex = zeros(6,1);
        e(7:12,:) =  zeros(6,1);%v状态误差清零

    end
   
    
    EX(i) = norm(ex);
    EV(i) = norm(e(7:12));
    
    inPut =g(1, triggerState);%这两步是为了保存控制输入
    U(:, i) = inPut(7:12);
    
    Y( : , i) = eventTriggersRK4(g, stepLength, X(i -1), Y( : , i-1), triggerState);%求解下一时刻的状态值
    %1.控制器阶段性地起作用（U(i)是在[0, 无穷}上的分段函数）
    %2.其实这是普通的解微分方程的步骤，当中并没有涉及与“平均一致性”相关的东西！！！
end

figure('name', 'system status：x');
plot(X, Y(1:6, : ));%grid;
xlabel('time s');
ylabel('position m');
legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('second order system position status');

figure('name', 'system status：v');
plot(X, Y(7:12, :));%grid;
xlabel('time s');
ylabel('velocity  m/s');
legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('second order system velocity status');

figure('name', 'X(position) Error function：e');
plot(X, EX);%grid on;
xlabel('time s');
ylabel('||e||');
legend('||e||');
title('second order system position error information');


figure('name', 'V(velocity) Error function：e');
plot(X, EV);%grid on;
xlabel('time s');
ylabel('||e||');
legend('||e||');
title('second order system velocity error information');


figure('name', 'Control input：u');
plot(X, U);%grid on;
xlabel('time s');
ylabel('Control Input');
legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('Control inputs for second order system');

function sign = EventTriggering(e, delta, L)
Alpha = 1;
Beta = 1;
A = [zeros(6,6), eye(6); -Alpha.*L, -Beta.*L];
B = [zeros(6,6),zeros(6,6); -Alpha.*L, -Beta.*L];
P = [(Alpha+Beta).*(L+eye(6)), eye(6); eye(6), eye(6)];

M = B'*P + P*B;

[~,D]=eig(L);%求拉普拉斯矩阵的特征值
tempArray = sort(D*ones(length(L),1));
lambda2 = tempArray(2, 1);%求得代数连通度

Rho = 0.1;
Xi = 2.197;
k = Rho *((Xi - 2)/2*norm(M));

sign = norm(e) - k*norm(delta);

%% 事件触发用到的ODE
% functionName:传入的微分方程的名称
% h:计算所用到的步长
% x0:其实时间点
% y0:初值
% u0:触发时刻的状态
function y = eventTriggersRK4(functionName, h, x0, y0, u0)

k1 = functionName(x0,           u0);
k2 = functionName(x0 + h/2,  u0 + h*k1/2);
k3 = functionName(x0 + h/2,  u0 + h*k2/2);
k4 = functionName(x0 + h,     u0 + h*k3);

y= y0 + h*(k1+2*k2+2*k3+k4)/6;