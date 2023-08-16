%% �����¼��������ƵĶ��׶��������ƽ��һ���Է���
%FileName: EventTriggersSecondOrderSystem
%Author  : 
%Date    : 2017��10��31�� 15:17:07
%Version ��V1.1.0

%% EventTriggersSecondOrderSystem
function EventTriggersSecondOrderSystem()

clc;
clear;
close all;

%% ��ʼ��ϵͳ����
global  A
A= [0  1  0  0 1  1;
    1 0  1  0  0  0;
    0 1  0  1  1  0;
    0  0  1  0 1  0;
    1  0  1  1 0  1;
    1  0  0  0 1  0];%�ڽӾ���

L = diag(sum(A)) - A;%ͼG��������˹����

[~,D]=eig(L);%��������˹���������ֵ������D��������ֵ���ɵĶԽǾ���
tempArray = sort(D*ones(length(L),1));%tempArray���ź��������
lambda2 = tempArray(2, 1);%��ô�����ͨ��(Ҳ���Ǵ�С����ֵ)

%��ʼ״̬����
x0=[-4; 6; 1; 16.2; 2.4; 8];%�ĸ��������λ�ó�ʼ״̬
v0=[-0.5; 1.3; -3.2; 8.4; 3; 2.2];%�ĸ��������λ�ó�ʼ״̬

%��λ��״̬���ٶ�״̬��ƽ��ֵ
aver_x0 = sum(x0) / length(x0);
aver_v0 = sum(v0) / length(v0);

%���һ���Է�������ȡ�Լ���ʼ��
consistentComponent = zeros(12, 1);%ϵͳ��һ���Է�������ռ䣬�Լ���ɳ�ʼ��
consistentComponent(1:6,:) = aver_x0.*ones(6, 1);
consistentComponent(7:12,:) = aver_v0.*ones(6, 1);

y0(1:6, :) = x0;
y0(7:12, :) = v0;

alpha = 1;
beta = 1;

g = @(t, x)[%����ϵͳ��΢�ַ���ģ��
    x(7);%x(7)����v(1)
    x(8);%x(8)����v(2)
    x(9);%x(9)����v(3)
    x(10);%x(10)����v(4)
    x(11);%x(11)����v(5)
    x(12);%x(12)����v(6)
    
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

%�����ǵ������е�΢�ַ��̵Ľ�
timeStart = 0;%�������ʼʱ��
timeEnd = 10;%����Ľ���ʱ��
stepLength = 0.05;%�ڽ�΢�ַ������ʱ���õ��Ĳ���
stepNum=floor((timeEnd-timeStart)/stepLength);%����floor�������ǽ�С��ת��Ϊ����
X(1)=timeStart;%ʱ�����
Y(:,1)=y0;%����ֵ������������������Ҫע��ά��

EX(1) = norm(consistentComponent(1:6, 1));%λ������������ֵ
EV(1) = norm(consistentComponent(7:12, 1));%�ٶ�����������ֵ

inPut = g(1, y0);%��ʼ����������
U(:, 1) = inPut(7:12);

triggerState = Y(:,1);%��ϵͳû�����������¼���ʱ����ݴ�ϵͳ״̬����ʱ����
aa = Y( 1 , 1);
for i = 2 : stepNum
    
    X(i)=X(i -1)+stepLength;%����ʱ����
    
    %��ȡ��һ���Է���
    delt(1:6,:) = Y(1:6, i - 1) - (aver_x0.*ones(6, 1) + (aver_v0 * X(i)).*ones(6, 1));
    delt(7:12,:) = Y(7:12, i - 1) - consistentComponent(7:12);
    
    e(1:6,:) = Y(1:6, i - 1) - triggerState(1:6, :);%��ȡX(i)ʱ�̵�λ��״̬���
    ex = Y(1:6, i - 1) - aa.*ones(6, 1);   
    e(7:12,:) = Y(7:12, i - 1) - triggerState(7:12, :);%��ȡX(i)ʱ�̵��ٶ�״̬���
    
    if EventTriggering(e, delt, L) >=0%��������������㴥���������ͽ���ϵͳ�ĸ���
        triggerState = Y( : , i -1);%���津��״̬
        %e(1:6,:) = zeros(6,1);%x״̬�������
        aa = Y(1, i - 1);
        ex = zeros(6,1);
        e(7:12,:) =  zeros(6,1);%v״̬�������

    end
   
    
    EX(i) = norm(ex);
    EV(i) = norm(e(7:12));
    
    inPut =g(1, triggerState);%��������Ϊ�˱����������
    U(:, i) = inPut(7:12);
    
    Y( : , i) = eventTriggersRK4(g, stepLength, X(i -1), Y( : , i-1), triggerState);%�����һʱ�̵�״ֵ̬
    %1.�������׶��Ե������ã�U(i)����[0, ����}�ϵķֶκ�����
    %2.��ʵ������ͨ�Ľ�΢�ַ��̵Ĳ��裬���в�û���漰�롰ƽ��һ���ԡ���صĶ���������
end

figure('name', 'system status��x');
plot(X, Y(1:6, : ));%grid;
xlabel('time s');
ylabel('position m');
legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('second order system position status');

figure('name', 'system status��v');
plot(X, Y(7:12, :));%grid;
xlabel('time s');
ylabel('velocity  m/s');
legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6');
title('second order system velocity status');

figure('name', 'X(position) Error function��e');
plot(X, EX);%grid on;
xlabel('time s');
ylabel('||e||');
legend('||e||');
title('second order system position error information');


figure('name', 'V(velocity) Error function��e');
plot(X, EV);%grid on;
xlabel('time s');
ylabel('||e||');
legend('||e||');
title('second order system velocity error information');


figure('name', 'Control input��u');
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

[~,D]=eig(L);%��������˹���������ֵ
tempArray = sort(D*ones(length(L),1));
lambda2 = tempArray(2, 1);%��ô�����ͨ��

Rho = 0.1;
Xi = 2.197;
k = Rho *((Xi - 2)/2*norm(M));

sign = norm(e) - k*norm(delta);

%% �¼������õ���ODE
% functionName:�����΢�ַ��̵�����
% h:�������õ��Ĳ���
% x0:��ʵʱ���
% y0:��ֵ
% u0:����ʱ�̵�״̬
function y = eventTriggersRK4(functionName, h, x0, y0, u0)

k1 = functionName(x0,           u0);
k2 = functionName(x0 + h/2,  u0 + h*k1/2);
k3 = functionName(x0 + h/2,  u0 + h*k2/2);
k4 = functionName(x0 + h,     u0 + h*k3);

y= y0 + h*(k1+2*k2+2*k3+k4)/6;