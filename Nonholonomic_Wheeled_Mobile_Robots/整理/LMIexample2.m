% LMI example
clear;clc
L11 = diag([2,2,1,2,3,2]);
n = 6;
tau_l = 0.002;
tau_u = 0.03;
K = 30000*eye(6);%diag([-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]);
Lambda = kron(K, eye(2));

lambdaNi = inv(Lambda);
A_F = [0,1,0,0,0,0;
       0,0,0,0,0,0;
       0,0,0,0,0,0;
       0,1,0,0,0,0;
       1,0,0,0,0,1;
       0,0,1,0,1,0]; % 跟随者邻接矩阵
A_LF = [1,0,0,0;
        0,1,1,0;
        0,0,0,1;
        1,0,0,0;
        0,1,0,0;
        0,0,0,0]; % 领导者和跟随者的耦合邻接矩阵
A = [A_F,A_LF];
B = [1,0,0,0,0,0;
     0,2,0,0,0,0;
     0,0,1,0,0,0;
     0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,0]; % 跟随者能接受到的领导者信息总和
d = sum(sum(A));
L = -A_F;
num_follower = 6;
for i = 1:num_follower
    L(i,i) = sum(A_F(i,:));
end

L1 = L + B;
L2 = -A_LF;
L1Ni =  kron(inv(L1), eye(2));
%% 求k的版本
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);
RR1 = lmivar(1, [2*n 1]);
RR2 = lmivar(1, [2*n 1]);


lmiterm([1 1 1 RR1], tau_l^2, 1)
lmiterm([1 1 1 RR2], (tau_u - tau_l)^2, 1)
lmiterm([1 1 1 P], -1, 1, 's')
lmiterm([1 1 1 Q1], 1, 1)
lmiterm([1 1 1 Q2], 1, 1)
lmiterm([1 1 1 R1], -1, 1)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 Q1], -1, 1)
lmiterm([1 1 2 Q2], -1, 1)
lmiterm([1 1 2 R1], 1, 1)

lmiterm([1 1 3 P], 1, 1)
lmiterm([1 1 3 Q1], -1, 1)
lmiterm([1 1 3 Q2], -1, 1)
lmiterm([1 1 3 R1], 1, 1)

lmiterm([1 1 4 R1], -1, 1)

lmiterm([1 2 2 Q1], 1, 1)
lmiterm([1 2 2 Q2], 1, 1)
lmiterm([1 2 2 R1], -1, 1)

lmiterm([1 2 3 Q1], 1, 1)
lmiterm([1 2 3 Q2], 1, 1)
lmiterm([1 2 3 R1], -1, 1)

lmiterm([1 2 4 R1], 1, 1)

lmiterm([1 3 3 Q1], 1, 1)
lmiterm([1 3 3 Q2], 1, 1)
lmiterm([1 3 3 R1], -1, 1)

lmiterm([1 3 4 R1], 1, 1)

lmiterm([1 4 4 Q1], -1, 1)
lmiterm([1 4 4 R1], -1, 1)
lmiterm([1 4 4 R2], -1, 1)

lmiterm([1 4 5 R2], 1, 1)

lmiterm([1 5 5 Q2], -1, 1)
lmiterm([1 5 5 R2], -1, 1)

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q1],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,Q2],1,1)
lmiterm([4,1,1,0],0)

lmiterm([-5,1,1,R1],1,1)
lmiterm([5,1,1,0],0)

lmiterm([-6,1,1,R2],1,1)
lmiterm([6,1,1,0],0)


lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-1);
%% 不求k的版本
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);

lmiterm([1 1 1 R1], tau_l^2, 1)
lmiterm([1 1 1 R2], (tau_u - tau_l)^2, 1)
lmiterm([1 1 1 P], -lambdaNi, 1, 's')
lmiterm([1 1 1 Q1], lambdaNi, lambdaNi)
lmiterm([1 1 1 Q2], lambdaNi, lambdaNi)
lmiterm([1 1 1 R1], -lambdaNi, lambdaNi)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 Q1], -lambdaNi, 1)
lmiterm([1 1 2 Q2], -lambdaNi, 1)
lmiterm([1 1 2 R1], lambdaNi, 1)

lmiterm([1 1 3 P], 1, lambdaNi)
lmiterm([1 1 3 Q1], -lambdaNi, lambdaNi)
lmiterm([1 1 3 Q2], -lambdaNi, lambdaNi)
lmiterm([1 1 3 R1], lambdaNi, lambdaNi)

lmiterm([1 1 4 R1], -lambdaNi, 1)

lmiterm([1 2 2 Q1], 1, 1)
lmiterm([1 2 2 Q2], 1, 1)
lmiterm([1 2 2 R1], -1, 1)

lmiterm([1 2 3 Q1], 1, lambdaNi)
lmiterm([1 2 3 Q2], 1, lambdaNi)
lmiterm([1 2 3 R1], -1, lambdaNi)

lmiterm([1 2 4 R1], 1, 1)

lmiterm([1 3 3 Q1], lambdaNi, lambdaNi)
lmiterm([1 3 3 Q2], lambdaNi, lambdaNi)
lmiterm([1 3 3 R1], -lambdaNi, lambdaNi)

lmiterm([1 3 4 R1], lambdaNi, 1)

lmiterm([1 4 4 Q1], -1, 1)
lmiterm([1 4 4 R1], -1, 1)
lmiterm([1 4 4 R2], -1, 1)

lmiterm([1 4 5 R2], 1, 1)

lmiterm([1 5 5 Q2], -1, 1)
lmiterm([1 5 5 R2], -1, 1)


lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q1],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,Q2],1,1)
lmiterm([4,1,1,0],0)

lmiterm([-5,1,1,R1],1,1)
lmiterm([5,1,1,0],0)

lmiterm([-6,1,1,R2],1,1)
lmiterm([6,1,1,0],0)

lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-1);
%% 将所有的时延加进来的版本
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);
S = lmivar(2, [2*n 2*n]); % S是矩阵，阶数为2*n，满块矩阵

lmiterm([1 1 1 R1], tau_l^2, 1)
lmiterm([1 1 1 R2], (tau_u - tau_l)^2, 1)
lmiterm([1 1 1 P], -lambdaNi, 1, 's')
lmiterm([1 1 1 Q1], lambdaNi, lambdaNi)
lmiterm([1 1 1 Q2], lambdaNi, lambdaNi)
lmiterm([1 1 1 R1], -lambdaNi, lambdaNi)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 Q1], -lambdaNi, 1)
lmiterm([1 1 2 Q2], -lambdaNi, 1)
lmiterm([1 1 2 R1], lambdaNi, 1)

lmiterm([1 1 3 P], 1, lambdaNi)
lmiterm([1 1 3 Q1], -lambdaNi, lambdaNi)
lmiterm([1 1 3 Q2], -lambdaNi, lambdaNi)
lmiterm([1 1 3 R1], lambdaNi, lambdaNi)

lmiterm([1 1 4 R1], -lambdaNi, 1)

lmiterm([1 2 2 Q1], 1, 1)
lmiterm([1 2 2 Q2], 1, 1)
lmiterm([1 2 2 R1], -1, 1)

lmiterm([1 2 3 Q1], 1, lambdaNi)
lmiterm([1 2 3 Q2], 1, lambdaNi)
lmiterm([1 2 3 R1], -1, lambdaNi)

lmiterm([1 2 4 R1], 1, 1)

lmiterm([1 3 3 Q1], lambdaNi, lambdaNi)
lmiterm([1 3 3 Q2], lambdaNi, lambdaNi)
lmiterm([1 3 3 R1], -lambdaNi, lambdaNi)

lmiterm([1 3 4 R1], lambdaNi, 1)

lmiterm([1 4 4 Q1], -1, 1)
lmiterm([1 4 4 R1], -1, 1)
lmiterm([1 4 4 R2], -1, 1)

lmiterm([1 4 5 S], 1, 1)

for i = 1:d
    lmiterm([1 4 5+i R2], 1/d, 1)
    lmiterm([1 4 5+i S], -1/d, 1)
end

lmiterm([1 5 5 Q2], -1, 1)
lmiterm([1 5 5 R2], -1, 1)

for i = 1:d
    lmiterm([1 5 5+i R2], 1/d, 1)
    lmiterm([1 5 5+i -S], -1/d, 1)
end

for i = 1:d
    lmiterm([1 5+i 5+i R2], -1/d, 1, 's')
    lmiterm([1 5+i 5+i S], 1/d, 1, 's')
end

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q1],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,Q2],1,1)
lmiterm([4,1,1,0],0)

lmiterm([-5,1,1,R1],1,1)
lmiterm([5,1,1,0],0)

lmiterm([-6,1,1,R2],1,1)
lmiterm([6,1,1,0],0)

lmiterm([-7,1,1,R2],1,1)
lmiterm([-7,2,2,R2],1,1)
lmiterm([-7,2,1,S],1,1)
lmiterm([7,1,1,0],0)

lmis = getlmis;
[tmin, xfeas] = feasp(lmis,[0,0,0,0,0],-10^(-10));
%% 把拉普拉斯矩阵加进来的版本
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);
S = lmivar(2, [2*n 2*n]); % S是矩阵，阶数为2*n，满块矩阵

lmiterm([1 1 1 R1], tau_l^2, 1)
lmiterm([1 1 1 R2], (tau_u - tau_l)^2, 1)
lmiterm([1 1 1 P], -lambdaNi, 1, 's')
lmiterm([1 1 1 Q1], lambdaNi, lambdaNi)
lmiterm([1 1 1 Q2], lambdaNi, lambdaNi)
lmiterm([1 1 1 R1], -lambdaNi, lambdaNi)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 Q1], -lambdaNi, 1)
lmiterm([1 1 2 Q2], -lambdaNi, 1)
lmiterm([1 1 2 R1], lambdaNi, 1)

lmiterm([1 1 3 P], 1, lambdaNi*L1Ni)
lmiterm([1 1 3 Q1], -lambdaNi, lambdaNi*L1Ni)
lmiterm([1 1 3 Q2], -lambdaNi, lambdaNi*L1Ni)
lmiterm([1 1 3 R1], lambdaNi, lambdaNi*L1Ni)

lmiterm([1 1 4 R1], -lambdaNi, 1)

lmiterm([1 2 2 Q1], 1, 1)
lmiterm([1 2 2 Q2], 1, 1)
lmiterm([1 2 2 R1], -1, 1)

lmiterm([1 2 3 Q1], 1, lambdaNi*L1Ni)
lmiterm([1 2 3 Q2], 1, lambdaNi*L1Ni)
lmiterm([1 2 3 R1], -1, lambdaNi*L1Ni)

lmiterm([1 2 4 R1], 1, 1)

lmiterm([1 3 3 Q1], L1Ni.'*lambdaNi, lambdaNi*L1Ni)
lmiterm([1 3 3 Q2], L1Ni.'*lambdaNi, lambdaNi*L1Ni)
lmiterm([1 3 3 R1], -L1Ni.'*lambdaNi, lambdaNi*L1Ni)

lmiterm([1 3 4 R1], L1Ni.'*lambdaNi, 1)

lmiterm([1 4 4 Q1], -1, 1)
lmiterm([1 4 4 R1], -1, 1)
lmiterm([1 4 4 R2], -1, 1)

lmiterm([1 4 5 S], 1, 1)

for i = 1:d
    lmiterm([1 4 5+i R2], 1/d, 1)
    lmiterm([1 4 5+i S], -1/d, 1)
end

lmiterm([1 5 5 Q2], -1, 1)
lmiterm([1 5 5 R2], -1, 1)

for i = 1:d
    lmiterm([1 5 5+i R2], 1/d, 1)
    lmiterm([1 5 5+i -S], -1/d, 1)
end

for i = 1:d
    lmiterm([1 5+i 5+i R2], -1/d, 1, 's')
    lmiterm([1 5+i 5+i S], 1/d, 1, 's')
end

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q1],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,Q2],1,1)
lmiterm([4,1,1,0],0)

lmiterm([-5,1,1,R1],1,1)
lmiterm([5,1,1,0],0)

lmiterm([-6,1,1,R2],1,1)
lmiterm([6,1,1,0],0)

lmiterm([-7,1,1,R2],1,1)
lmiterm([-7,2,2,R2],1,1)
lmiterm([-7,2,1,S],1,1)
lmiterm([7,1,1,0],0)

lmis = getlmis;
[tmin, xfeas] = feasp(lmis,[0,0,0,0,0],-10^(-10));
%% 结果验证
PP = dec2mat(lmis, xfeas, P)
QQ1 = dec2mat(lmis, xfeas, Q1)
QQ2 = dec2mat(lmis, xfeas, Q2)
RR1 = dec2mat(lmis, xfeas, R1)
RR2 = dec2mat(lmis, xfeas, R2)