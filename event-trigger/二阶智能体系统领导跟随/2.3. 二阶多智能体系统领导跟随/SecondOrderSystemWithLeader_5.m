%% 事件触发下带有领导者的二阶多智能体的领导跟随一致性仿真
%FileName: EventTriggersSecondOrderSystemWithLeader
%Author    :  
%Date        : 2018年4月15日14:51:09
%Version  ：V1.1.0 

%% SecondOrderSystemWithLeader
function SecondOrderSystemWithLeader_5()
clc;
clear;
close all;

%% 1.初始化系统参数，在解系统方程之前的准备工作
global  A
A= [0  0  1  0  0  0;
      0  0  1  0  0  0;
      1  1  0  1  0  0;
      0  0  1  0  1  0;
      0  0  0  1  0  1;
      0  0  0  0 1  0];%系统跟随者的邻接矩阵

L = diag(sum(A)) - A;%图G的拉普拉斯矩阵
[~,D]=eig(L);%求拉普拉斯矩阵的特征值，其中D是由特征值构成的对角矩阵
tempArray = sort(D*ones(length(L),1));%tempArray是排好序的向量
lambda2 = tempArray(2, 1);%求得代数连通度(也就是) 

H = diag([1 0 0 1 1 0]);%系统的领导跟随矩阵

%初始化系统的初始状态
X_x0=[0; 0; 0; 0.2; 13; 8];%四个智能体的位置初始状态
X_v0=[1.5; 1.3; 1.2; 2.4; 3; 2.2];%四个智能体的位置初始状态
X_y0(1:6, :) = X_x0;
X_y0(7:12, :) = X_v0;
X_y0(13:14, :) = [0.5; 1.5];%加入领导者的初始状态


Y_x0=[6; 2; 13; 0; 0; 0];%四个智能体的位置初始状态
Y_v0=[2.5; 1.3; -2.2; 3.4; 1; 0.2];%四个智能体的位置初始状态
Y_y0(1:6, :) = Y_x0;
Y_y0(7:12, :) = Y_v0;
Y_y0(13:14, :) = [1; 1];%加入领导者的初始状态


alpha = 1.5;
beta = 1;
lambda = 10;
% little=0.53864     big=0.67713
% r=10     alpha=1.5     beta=1

%% 2.创建系统函数
g = @(t, x)[%二阶系统的微分方程模型
x(7);
x(8);
x(9);
x(10);
x(11);
x(12);

0+...
 alpha*(-A(1,1)*(x(1)-x(1))-A(1,2)*(x(1) - x(2))-A(1,3)* (x(1)-x(3))-A(1,4)*(x(1) - x(4))-A(1,5)*(x(1) - x(5))-A(1,6)*(x(1) - x(6)))+...
 beta*(-A(1,1)*(x(7)-x(7))-A(1,2)*(x(7) - x(8))-A(1,3)* (x(7)-x(9))-A(1,4)*(x(7) - x(10))-A(1,5)*(x(7) - x(11))-A(1,6)*(x(7) - x(12)))+...
 lambda*-H(1,1)*((x(1)-x(13))+(x(7) - x(14)));
 
 0+...
  alpha*(-A(2,1)*(x(2)-x(1))-A(2,2)*(x(2) - x(2))-A(2,3)* (x(2)-x(3))-A(2,4)*(x(2) - x(4))-A(2,5)*(x(2) - x(5))-A(2,6)*(x(2) - x(6)))+...
 beta*(-A(2,1)*(x(8)-x(7))-A(2,2)*(x(8) - x(8))-A(2,3)* (x(8)-x(9))-A(2,4)*(x(8) - x(10))-A(2,5)*(x(8) - x(11))-A(2,6)*(x(8) - x(12)))+...
 lambda*-H(2,2)*((x(2)-x(13))+(x(8) - x(14)));
 
 0+...
  alpha*(-A(3,1)*(x(3)-x(1))-A(3,2)*(x(3) - x(2))-A(3,3)* (x(3)-x(3))-A(3,4)*(x(1) - x(4))-A(3,5)*(x(3) - x(5))-A(3,6)*(x(3) - x(6)))+...
 beta*(-A(3,1)*(x(9)-x(7))-A(3,2)*(x(9) - x(8))-A(3,3)* (x(9)-x(9))-A(3,4)*(x(9) - x(10))-A(3,5)*(x(9) - x(11))-A(3,6)*(x(9) - x(12)))+...
 lambda*-H(3,3)*((x(3)-x(13))+(x(9) - x(14)));

 0+...
  alpha*(-A(4,1)*(x(4)-x(1))-A(4,2)*(x(4) - x(2))-A(4,3)* (x(4)-x(3))-A(4,4)*(x(4) - x(4))-A(4,5)*(x(4) - x(5))-A(4,6)*(x(4) - x(6)))+...
 beta*(-A(4,1)*(x(10)-x(7))-A(4,2)*(x(10) - x(8))-A(4,3)* (x(10)-x(9))-A(4,4)*(x(10) - x(10))-A(4,5)*(x(10) - x(11))-A(4,6)*(x(10) - x(12)))+...
 lambda*-H(4,4)*((x(4)-x(13))+(x(10) - x(14)));
 
 0+...
  alpha*(-A(5,1)*(x(5)-x(1))-A(5,2)*(x(5) - x(2))-A(5,3)* (x(5)-x(3))-A(5,4)*(x(5) - x(4))-A(5,5)*(x(5) - x(5))-A(5,6)*(x(5) - x(6)))+...
 beta*(-A(5,1)*(x(11)-x(7))-A(5,2)*(x(11) - x(8))-A(5,3)* (x(11)-x(9))-A(5,4)*(x(11) - x(10))-A(5,5)*(x(11) - x(11))-A(5,6)*(x(11) - x(12)))+...
 lambda*-H(5,5)*((x(5)-x(13))+(x(11) - x(14)));
 
 0+...
  alpha*(-A(6,1)*(x(6)-x(1))-A(6,2)*(x(1) - x(2))-A(6,3)* (x(6)-x(3))-A(6,4)*(x(6) - x(4))-A(6,5)*(x(6) - x(5))-A(6,6)*(x(6) - x(6)))+...
 beta*(-A(6,1)*(x(12)-x(7))-A(6,2)*(x(12) - x(8))-A(6,3)* (x(12)-x(9))-A(6,4)*(x(12) - x(10))-A(6,5)*(x(12) - x(11))-A(6,6)*(x(12) - x(12)))+...
 lambda*-H(6,6)*((x(6)-x(13))+(x(12) - x(14)));
 
 x(14);
 0;%此时的领导者的控制协议u0
 ]; 

%% 下面是单步运行的微分方程的解
timeStart = 0;%仿真的起始时间
timeEnd = 20;%仿真的结束时间
stepLength = 0.0025;%在解微分方程组的时候用到的步长
stepNum=floor((timeEnd-timeStart)/stepLength);%求步数floor的作用是将小数转化为整数
X(1)=timeStart;%时间起点

Y1(:, 1)=X_y0;%赋初值，可以是向量，但是要注意维数
triggerState_1 = Y1(:, 1);%在系统没有遇到触发事件的时候的暂存系统状态的临时变量

Y2(:, 1)=Y_y0;%赋初值，可以是向量，但是要注意维数
triggerState_2 = Y2(:, 1);%在系统没有遇到触发事件的时候的暂存系统状态的临时变量

for i = 1 : 6
    triggerMatrix_1(:, i) = triggerState_1;
    errorMatrix_1(:, i) = zeros(12,1);
    
    triggerMatrix_2(:, i) = triggerState_2;
    errorMatrix_2(:, i) = zeros(12,1);
    
    vectorialNorm(i, 1) = sqrt(X_v0(i, :)^2 + Y_v0(i, :)^2);
    vectorDirection(i, 1) = atan(X_v0(i, :) / Y_v0(i, :));
end
E( :, 1) = zeros(6,1);%速度误差变量赋初值


inPut = g(1, X_y0);%初始化控制输入
U(:, 1) = inPut(7:12);

for i = 2 : stepNum
    %1.更新时间轴
    X(i)=X(i -1)+stepLength;%更新时间轴

    %2.解第一维度
    for j = 1 : 6
        errorMatrix_1(1:12, j) = Y1(1:12, i - 1) - triggerMatrix_1(1:12, j);%获取位置和速度误差

          %提取非一致性分量
         delt(1:6, 1) = Y1(1:6, i - 1) - Y1(13, i - 1).*ones(6, 1);
         delt(7:12, 1) = Y1(7:12, i - 1) - Y1(14, i - 1).*ones(6, 1);

        if(singleAgentEventTriggering(A, H, j, delt, errorMatrix_1(1:12, j)) >= 0)
            %disp(['时间为：',num2str(X(i)), '   Agent',num2str(j),'触发！']);
            triggerMatrix_1(:, j) = Y1(:, i -1);%保存位置触发状态
            errorMatrix_1(7:12, j) = zeros(6,1);%保存位置触发状态
        end

        %在这里获取控制输入数值
        inPut =g(1, triggerMatrix_1(:, j));%这两步是为了保存控制输入
        U(j, i) = inPut(j+6);
        
        
        tempState_1 = eventTriggersRK4(g, stepLength, X(i -1), Y1(: , i-1), triggerMatrix_1(:, j));
        Y1( j , i) = tempState_1(j, 1);
        Y1( j+6 , i) = tempState_1(j+6, 1);  
    end
    
         Y1(13:14 , i) = tempState_1(13:14 , 1);  
         
     %3.解第二维度     
    for j = 1 : 6
        errorMatrix_2(1:12, j) = Y2(1:12, i - 1) - triggerMatrix_2(1:12, j);%获取位置和速度误差

        if(singleAgentEventTriggering(A, H, j, Y2(1:12, i - 1), errorMatrix_2(1:12, j)) >= 0)
            %disp(['时间为：',num2str(X(i)), '   Agent',num2str(j),'触发！']);
            triggerMatrix_2(:, j) = Y2(:, i -1);%保存位置触发状态
            errorMatrix_2(j+6, j) = 0;%保存位置触发状态
        end

        tempState_2 = eventTriggersRK4(g, stepLength, X(i -1), Y2(: , i-1), triggerMatrix_2(:, j));
        Y2( j , i) = tempState_2(j, 1);
        Y2( j+6 , i) = tempState_2(j+6, 1);  
    end
    
         Y2(13:14 , i) = tempState_2(13:14 , 1);  
        
         for k = 1 : 6
             vectorialNorm(k, i) = sqrt(Y1(k+6, i)^2 + Y2(k+6, i)^2);
             vectorDirection(k, i) = atan(Y1(k+6, i) / Y2(k+6, i));
             %E(k, i) = norm(errorMatrix_1(7:12, k));%速度误差变量赋初值
             E(k, i) = getErrorSum(A, H, k, errorMatrix_1(1:12, j));
         end
end

%% 绘制图像(第一维度)
figure('name', '多智能体系统X方向的位置轨迹');
subplot(221)
plot(X, Y1(1:6, : ));
xlabel('time s');
ylabel('position m');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status');

subplot(222)
%figure('name', '多智能体系统的速度轨迹');
plot(X, Y1(7:12, :));
xlabel('time s');
ylabel('velocity  m/s');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status');

subplot(223)
%figure('name', '位置差变化趋势');
plot(X, Y1(1:6, : ) - ones(6,1)*Y1(13, :));
xlabel('time s');
ylabel('position m');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status change');

subplot(224)
%figure('name', '速度差变化趋势');
plot(X, Y1(7:12, :) - ones(6,1)*Y1(14, :));% axis([0, 20, -10, 10]);% 坐标轴的显示范围
xlabel('time s');
ylabel('velocity  m/s');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status change');

%% 绘制图像（第二维度）
figure('name', '多智能体系统Y方向的位置轨迹');
subplot(221)
plot(X, Y2(1:6, : ));
xlabel('time s');
ylabel('position m');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status');

subplot(222)
%figure('name', '多智能体系统的速度轨迹');
plot(X, Y2(7:12, :));
xlabel('time s');
ylabel('velocity  m/s');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status');

subplot(223)
%figure('name', '位置差变化趋势');
plot(X, Y2(1:6, : ) - ones(6,1)*Y2(13, :));
xlabel('time s');
ylabel('position m');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status change');

subplot(224)
%figure('name', '速度差变化趋势');
plot(X, Y2(7:12, :) - ones(6,1)*Y2(14, :));% axis([0, 20, -10, 10]);% 坐标轴的显示范围
xlabel('time s');
ylabel('velocity  m/s');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status change');


figure('name', '二维空间的位置分布');
plot(Y2(1, : ), Y1(1, : ));
hold on;
plot(Y2(2, : ), Y1(2, : ));
hold on;
plot(Y2(3, : ), Y1(3, : ));
hold on;
plot(Y2(4, : ), Y1(4, : ));
hold on;
plot(Y2(5, : ), Y1(5, : ));
hold on;
plot(Y2(6, : ), Y1(6, : ));
hold on;
scatter(Y2(1, 1), Y1(1, 1),'o');
scatter(Y2(2, 1), Y1(2, 1),'o');
scatter(Y2(3, 1), Y1(3, 1),'o');
scatter(Y2(4, 1), Y1(4, 1),'o');
scatter(Y2(5, 1), Y1(5, 1),'o');
scatter(Y2(6, 1), Y1(6, 1),'o');
hold off;
%axis([0, 43, 0, 35]);% 坐标轴的显示范围


% figure('name', 'Control input：u');
% plot(X, U);grid on;
% xlabel('time s');
% ylabel('Control Input');
% legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
% title('Control inputs for second order system');


% figure('name', 'vectorialNorm');
% plot(X, vectorialNorm);grid on;
% 
% figure('name', 'vectorDirection');
% plot(X, vectorDirection);grid on;

figure('name', 'vectorDirection');
plot(X, E(6, :));
axis([0, 6, 0, 0.012])
xlabel('time s');
ylabel('||e||');
title('The error change trend of the agent 3');
%% 分布式事件触发函数
% A:邻接矩阵
% H:领航者智能体的邻接矩阵
% state:智能体的位置状态和速度状态
% error:智能体的位置误差和速度误差
function flag= singleAgentEventTriggering(A,H,i,state,error)
   
    %下面的是系统的三个参数
    alpha = 1.5;
    beta = 1;
    lambda = 10;
    xita = 0.001518;
    delta = 0.59125;
    
% r=10     alpha=1.5     beta=1
% little=0.53651     big=0.67944

    xi = state(i, :);%取出第i个智能体的位置状态
    vi = state(i+6, :);%取出第i个智能体的速度状态
    
    exi  = error(i, :);%取出第i个智能体的位置误差信息
    evi  = error(i+6, :);%取出第i个智能体的速度误差信息
     
    tempRow = A(i,:);%取出邻接矩阵的第i行
    
    errorSum = 0;%申请临时误差求和变量
    for num = 1 : length(tempRow)%从第一个到第六个进行遍历
        if(tempRow(num)>0)
            errorSum = errorSum + alpha*(norm(exi).^2 +norm(error(num, :)).^2)+...
                                              + beta*(norm(evi).^2 +norm(error(num+6, :)).^2)+...
                                              + lambda*H(i,i)*(norm(exi).^2 + norm(evi).^2);
        end
    end
    
    errorSum = errorSum*0.5;
    flag = errorSum - xita*(delta-0.5)*(norm(xi).^2 + norm(vi).^2);

function errorSum = getErrorSum(A,H,i,error) 
    alpha = 1.5;
    beta = 1;
    lambda = 10;
    % little=0.74868     big=0.79445
% r=12     alpha=2.5     beta=1

    exi  = error(i, :);%取出第i个智能体的位置误差信息
    evi  = error(i+6, :);%取出第i个智能体的速度误差信息
     
    tempRow = A(i,:);%取出邻接矩阵的第i行
    
    errorSum = 0;%申请临时误差求和变量
    for num = 1 : length(tempRow)%从第一个到第六个进行遍历
        if(tempRow(num)>0)
            errorSum = errorSum + 0*(norm(exi).^2 +norm(error(num, :)).^2)+...
                                              + beta*(norm(evi).^2 +norm(error(num+6, :)).^2)+...
                                              + lambda*H(i,i)*(norm(evi).^2);
        end
    end
    
%% 改写的ODE函数（Runge-Kutta法）
% funcName:微分方程的函数名
% h:每一步的步长
% tspan: 起始时间和结尾时间
% X_y0:微分方程的初始值
function [x, y] = SolvingDifferentialEquation(funcName, h, tspan, X_y0)

    startTime = tspan(1);
    endTime = tspan(2);
    n=floor((endTime-startTime)/h);%求步数floor的作用是将小数转化为整数
    x(1)=startTime;%时间起点
    y(:,1)=X_y0;%赋初值，可以是向量，但是要注意维数
    
    for i = 1 : n
        x(i+1) = x(i) + h;
        y( : , i + 1)= stepRK4(funcName, h, x(i), y( : , i));
    end
    
%% 事件触发用到的ODE
% functionName:传入的微分方程的名称
% h:计算所用到的步长
% x0:其实时间点
% X_y0:初值
% u0:触发时刻的状态
function y = eventTriggersRK4(functionName, h, x0, X_y0, u0)

    k1 = functionName(x0,           u0);
    k2 = functionName(x0 + h/2,  u0 + h*k1/2);
    k3 = functionName(x0 + h/2,  u0 + h*k2/2);
    k4 = functionName(x0 + h,     u0 + h*k3);

    y= X_y0 + h*(k1+2*k2+2*k3+k4)/6;

%% 连续时间下的单步的RK4
% functionName:传入的微分方程的名称
% h:计算所用到的步长
% x0:其实时间点
% X_y0:初值
function y = stepRK4(functionName, h, x0, X_y0)

    k1 = functionName(x0,            X_y0);
    k2 = functionName(x0 + h/2,  X_y0 + h*k1/2);
    k3 = functionName(x0 + h/2,  X_y0 + h*k2/2);
    k4 = functionName(x0 + h,     X_y0 + h*k3);

    y= X_y0 + h*(k1+2*k2+2*k3+k4)/6;
