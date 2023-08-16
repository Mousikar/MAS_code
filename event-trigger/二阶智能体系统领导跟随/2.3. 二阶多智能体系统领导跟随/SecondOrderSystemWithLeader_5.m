%% �¼������´����쵼�ߵĶ��׶���������쵼����һ���Է���
%FileName: EventTriggersSecondOrderSystemWithLeader
%Author    :  
%Date        : 2018��4��15��14:51:09
%Version  ��V1.1.0 

%% SecondOrderSystemWithLeader
function SecondOrderSystemWithLeader_5()
clc;
clear;
close all;

%% 1.��ʼ��ϵͳ�������ڽ�ϵͳ����֮ǰ��׼������
global  A
A= [0  0  1  0  0  0;
      0  0  1  0  0  0;
      1  1  0  1  0  0;
      0  0  1  0  1  0;
      0  0  0  1  0  1;
      0  0  0  0 1  0];%ϵͳ�����ߵ��ڽӾ���

L = diag(sum(A)) - A;%ͼG��������˹����
[~,D]=eig(L);%��������˹���������ֵ������D��������ֵ���ɵĶԽǾ���
tempArray = sort(D*ones(length(L),1));%tempArray���ź��������
lambda2 = tempArray(2, 1);%��ô�����ͨ��(Ҳ����) 

H = diag([1 0 0 1 1 0]);%ϵͳ���쵼�������

%��ʼ��ϵͳ�ĳ�ʼ״̬
X_x0=[0; 0; 0; 0.2; 13; 8];%�ĸ��������λ�ó�ʼ״̬
X_v0=[1.5; 1.3; 1.2; 2.4; 3; 2.2];%�ĸ��������λ�ó�ʼ״̬
X_y0(1:6, :) = X_x0;
X_y0(7:12, :) = X_v0;
X_y0(13:14, :) = [0.5; 1.5];%�����쵼�ߵĳ�ʼ״̬


Y_x0=[6; 2; 13; 0; 0; 0];%�ĸ��������λ�ó�ʼ״̬
Y_v0=[2.5; 1.3; -2.2; 3.4; 1; 0.2];%�ĸ��������λ�ó�ʼ״̬
Y_y0(1:6, :) = Y_x0;
Y_y0(7:12, :) = Y_v0;
Y_y0(13:14, :) = [1; 1];%�����쵼�ߵĳ�ʼ״̬


alpha = 1.5;
beta = 1;
lambda = 10;
% little=0.53864     big=0.67713
% r=10     alpha=1.5     beta=1

%% 2.����ϵͳ����
g = @(t, x)[%����ϵͳ��΢�ַ���ģ��
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
 0;%��ʱ���쵼�ߵĿ���Э��u0
 ]; 

%% �����ǵ������е�΢�ַ��̵Ľ�
timeStart = 0;%�������ʼʱ��
timeEnd = 20;%����Ľ���ʱ��
stepLength = 0.0025;%�ڽ�΢�ַ������ʱ���õ��Ĳ���
stepNum=floor((timeEnd-timeStart)/stepLength);%����floor�������ǽ�С��ת��Ϊ����
X(1)=timeStart;%ʱ�����

Y1(:, 1)=X_y0;%����ֵ������������������Ҫע��ά��
triggerState_1 = Y1(:, 1);%��ϵͳû�����������¼���ʱ����ݴ�ϵͳ״̬����ʱ����

Y2(:, 1)=Y_y0;%����ֵ������������������Ҫע��ά��
triggerState_2 = Y2(:, 1);%��ϵͳû�����������¼���ʱ����ݴ�ϵͳ״̬����ʱ����

for i = 1 : 6
    triggerMatrix_1(:, i) = triggerState_1;
    errorMatrix_1(:, i) = zeros(12,1);
    
    triggerMatrix_2(:, i) = triggerState_2;
    errorMatrix_2(:, i) = zeros(12,1);
    
    vectorialNorm(i, 1) = sqrt(X_v0(i, :)^2 + Y_v0(i, :)^2);
    vectorDirection(i, 1) = atan(X_v0(i, :) / Y_v0(i, :));
end
E( :, 1) = zeros(6,1);%�ٶ�����������ֵ


inPut = g(1, X_y0);%��ʼ����������
U(:, 1) = inPut(7:12);

for i = 2 : stepNum
    %1.����ʱ����
    X(i)=X(i -1)+stepLength;%����ʱ����

    %2.���һά��
    for j = 1 : 6
        errorMatrix_1(1:12, j) = Y1(1:12, i - 1) - triggerMatrix_1(1:12, j);%��ȡλ�ú��ٶ����

          %��ȡ��һ���Է���
         delt(1:6, 1) = Y1(1:6, i - 1) - Y1(13, i - 1).*ones(6, 1);
         delt(7:12, 1) = Y1(7:12, i - 1) - Y1(14, i - 1).*ones(6, 1);

        if(singleAgentEventTriggering(A, H, j, delt, errorMatrix_1(1:12, j)) >= 0)
            %disp(['ʱ��Ϊ��',num2str(X(i)), '   Agent',num2str(j),'������']);
            triggerMatrix_1(:, j) = Y1(:, i -1);%����λ�ô���״̬
            errorMatrix_1(7:12, j) = zeros(6,1);%����λ�ô���״̬
        end

        %�������ȡ����������ֵ
        inPut =g(1, triggerMatrix_1(:, j));%��������Ϊ�˱����������
        U(j, i) = inPut(j+6);
        
        
        tempState_1 = eventTriggersRK4(g, stepLength, X(i -1), Y1(: , i-1), triggerMatrix_1(:, j));
        Y1( j , i) = tempState_1(j, 1);
        Y1( j+6 , i) = tempState_1(j+6, 1);  
    end
    
         Y1(13:14 , i) = tempState_1(13:14 , 1);  
         
     %3.��ڶ�ά��     
    for j = 1 : 6
        errorMatrix_2(1:12, j) = Y2(1:12, i - 1) - triggerMatrix_2(1:12, j);%��ȡλ�ú��ٶ����

        if(singleAgentEventTriggering(A, H, j, Y2(1:12, i - 1), errorMatrix_2(1:12, j)) >= 0)
            %disp(['ʱ��Ϊ��',num2str(X(i)), '   Agent',num2str(j),'������']);
            triggerMatrix_2(:, j) = Y2(:, i -1);%����λ�ô���״̬
            errorMatrix_2(j+6, j) = 0;%����λ�ô���״̬
        end

        tempState_2 = eventTriggersRK4(g, stepLength, X(i -1), Y2(: , i-1), triggerMatrix_2(:, j));
        Y2( j , i) = tempState_2(j, 1);
        Y2( j+6 , i) = tempState_2(j+6, 1);  
    end
    
         Y2(13:14 , i) = tempState_2(13:14 , 1);  
        
         for k = 1 : 6
             vectorialNorm(k, i) = sqrt(Y1(k+6, i)^2 + Y2(k+6, i)^2);
             vectorDirection(k, i) = atan(Y1(k+6, i) / Y2(k+6, i));
             %E(k, i) = norm(errorMatrix_1(7:12, k));%�ٶ�����������ֵ
             E(k, i) = getErrorSum(A, H, k, errorMatrix_1(1:12, j));
         end
end

%% ����ͼ��(��һά��)
figure('name', '��������ϵͳX�����λ�ù켣');
subplot(221)
plot(X, Y1(1:6, : ));
xlabel('time s');
ylabel('position m');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status');

subplot(222)
%figure('name', '��������ϵͳ���ٶȹ켣');
plot(X, Y1(7:12, :));
xlabel('time s');
ylabel('velocity  m/s');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status');

subplot(223)
%figure('name', 'λ�ò�仯����');
plot(X, Y1(1:6, : ) - ones(6,1)*Y1(13, :));
xlabel('time s');
ylabel('position m');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status change');

subplot(224)
%figure('name', '�ٶȲ�仯����');
plot(X, Y1(7:12, :) - ones(6,1)*Y1(14, :));% axis([0, 20, -10, 10]);% ���������ʾ��Χ
xlabel('time s');
ylabel('velocity  m/s');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status change');

%% ����ͼ�񣨵ڶ�ά�ȣ�
figure('name', '��������ϵͳY�����λ�ù켣');
subplot(221)
plot(X, Y2(1:6, : ));
xlabel('time s');
ylabel('position m');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status');

subplot(222)
%figure('name', '��������ϵͳ���ٶȹ켣');
plot(X, Y2(7:12, :));
xlabel('time s');
ylabel('velocity  m/s');
%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status');

subplot(223)
%figure('name', 'λ�ò�仯����');
plot(X, Y2(1:6, : ) - ones(6,1)*Y2(13, :));
xlabel('time s');
ylabel('position m');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('position status change');

subplot(224)
%figure('name', '�ٶȲ�仯����');
plot(X, Y2(7:12, :) - ones(6,1)*Y2(14, :));% axis([0, 20, -10, 10]);% ���������ʾ��Χ
xlabel('time s');
ylabel('velocity  m/s');%legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('velocity status change');


figure('name', '��ά�ռ��λ�÷ֲ�');
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
%axis([0, 43, 0, 35]);% ���������ʾ��Χ


% figure('name', 'Control input��u');
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
%% �ֲ�ʽ�¼���������
% A:�ڽӾ���
% H:�캽����������ڽӾ���
% state:�������λ��״̬���ٶ�״̬
% error:�������λ�������ٶ����
function flag= singleAgentEventTriggering(A,H,i,state,error)
   
    %�������ϵͳ����������
    alpha = 1.5;
    beta = 1;
    lambda = 10;
    xita = 0.001518;
    delta = 0.59125;
    
% r=10     alpha=1.5     beta=1
% little=0.53651     big=0.67944

    xi = state(i, :);%ȡ����i���������λ��״̬
    vi = state(i+6, :);%ȡ����i����������ٶ�״̬
    
    exi  = error(i, :);%ȡ����i���������λ�������Ϣ
    evi  = error(i+6, :);%ȡ����i����������ٶ������Ϣ
     
    tempRow = A(i,:);%ȡ���ڽӾ���ĵ�i��
    
    errorSum = 0;%������ʱ�����ͱ���
    for num = 1 : length(tempRow)%�ӵ�һ�������������б���
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

    exi  = error(i, :);%ȡ����i���������λ�������Ϣ
    evi  = error(i+6, :);%ȡ����i����������ٶ������Ϣ
     
    tempRow = A(i,:);%ȡ���ڽӾ���ĵ�i��
    
    errorSum = 0;%������ʱ�����ͱ���
    for num = 1 : length(tempRow)%�ӵ�һ�������������б���
        if(tempRow(num)>0)
            errorSum = errorSum + 0*(norm(exi).^2 +norm(error(num, :)).^2)+...
                                              + beta*(norm(evi).^2 +norm(error(num+6, :)).^2)+...
                                              + lambda*H(i,i)*(norm(evi).^2);
        end
    end
    
%% ��д��ODE������Runge-Kutta����
% funcName:΢�ַ��̵ĺ�����
% h:ÿһ���Ĳ���
% tspan: ��ʼʱ��ͽ�βʱ��
% X_y0:΢�ַ��̵ĳ�ʼֵ
function [x, y] = SolvingDifferentialEquation(funcName, h, tspan, X_y0)

    startTime = tspan(1);
    endTime = tspan(2);
    n=floor((endTime-startTime)/h);%����floor�������ǽ�С��ת��Ϊ����
    x(1)=startTime;%ʱ�����
    y(:,1)=X_y0;%����ֵ������������������Ҫע��ά��
    
    for i = 1 : n
        x(i+1) = x(i) + h;
        y( : , i + 1)= stepRK4(funcName, h, x(i), y( : , i));
    end
    
%% �¼������õ���ODE
% functionName:�����΢�ַ��̵�����
% h:�������õ��Ĳ���
% x0:��ʵʱ���
% X_y0:��ֵ
% u0:����ʱ�̵�״̬
function y = eventTriggersRK4(functionName, h, x0, X_y0, u0)

    k1 = functionName(x0,           u0);
    k2 = functionName(x0 + h/2,  u0 + h*k1/2);
    k3 = functionName(x0 + h/2,  u0 + h*k2/2);
    k4 = functionName(x0 + h,     u0 + h*k3);

    y= X_y0 + h*(k1+2*k2+2*k3+k4)/6;

%% ����ʱ���µĵ�����RK4
% functionName:�����΢�ַ��̵�����
% h:�������õ��Ĳ���
% x0:��ʵʱ���
% X_y0:��ֵ
function y = stepRK4(functionName, h, x0, X_y0)

    k1 = functionName(x0,            X_y0);
    k2 = functionName(x0 + h/2,  X_y0 + h*k1/2);
    k3 = functionName(x0 + h/2,  X_y0 + h*k2/2);
    k4 = functionName(x0 + h,     X_y0 + h*k3);

    y= X_y0 + h*(k1+2*k2+2*k3+k4)/6;
