% LMI example
clear;clc
L11 = diag([2,2,1,2,3,2]);
n = 6;
tau_l = 0.002;
tau_u = 0.03;
K = diag([3,3,3,3,3,3]);
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
d = sum(sum(A));
%%
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);

lmiterm([1 1 1 Q1], lambdaNi, lambdaNi)
lmiterm([1 1 1 Q2], lambdaNi, lambdaNi)
lmiterm([1 1 1 R1], -lambdaNi, lambdaNi)

lmiterm([1 1 2 P], lambdaNi, 1)
lmiterm([1 1 2 Q1], -lambdaNi, lambdaNi)
lmiterm([1 1 2 Q2], -lambdaNi, lambdaNi)
lmiterm([1 1 2 R1], lambdaNi, lambdaNi)

lmiterm([1 1 3 R1], lambdaNi, 1)

lmiterm([1 2 2 R1], tau_l^2, 1)
lmiterm([1 2 2 R2], (tau_u - tau_l)^2, 1)
lmiterm([1 2 2 P], -lambdaNi, 1, 's')
lmiterm([1 2 2 Q1], lambdaNi, lambdaNi)
lmiterm([1 2 2 Q2], lambdaNi, lambdaNi)
lmiterm([1 2 2 R1], -lambdaNi, lambdaNi)

lmiterm([1 2 3 R1], -lambdaNi, 1)

lmiterm([1 3 3 Q1], -1, 1)
lmiterm([1 3 3 R1], -1, 1)
lmiterm([1 3 3 R2], -1, 1)

lmiterm([1 3 4 R2], 1, 1)

lmiterm([1 4 4 Q2], -1, 1)
lmiterm([1 4 4 R2], -1, 1)

lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-1);
% %% 结果验证
% PP = dec2mat(lmis, xfeas, P)
% QQ1 = dec2mat(lmis, xfeas, Q1)
% QQ2 = dec2mat(lmis, xfeas, Q2)
% RR1 = dec2mat(lmis, xfeas, R1)
% RR2 = dec2mat(lmis, xfeas, R2)