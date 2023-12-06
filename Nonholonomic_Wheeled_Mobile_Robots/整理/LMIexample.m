% LMI example
clear;clc
L11 = diag([2,2,1,2,3,2]);
L11_1 = inv(L11);
K = 0.3 * eye(6);
Lambda = kron(L11 * K, eye(2));
n = 6;
tau_l = 0.001;
tau_u = 0.3;

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
S = lmivar(2, [2*n 2*n]); % S是对称矩阵，阶数为2*n，满块矩阵

lmiterm([1 1 1 Q1], 1, 1)
lmiterm([1 1 1 Q2], 1, 1)
lmiterm([1 1 1 R1], -1, 1)

% lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 P], -1, 1)

lmiterm([1 1 3 P], -1, Lambda)

lmiterm([1 1 4 R1], 1, 1)

lmiterm([1 2 2 R1], tau_l^2, 1)
lmiterm([1 2 2 R2], (tau_u - tau_l)^2, 1)

% lmiterm([1 2 3 R1], -tau_l^2, Lambda)
% lmiterm([1 2 3 R2], -(tau_u - tau_l)^2, Lambda)
lmiterm([1 2 3 R1], tau_l^2, Lambda)
lmiterm([1 2 3 R2], (tau_u - tau_l)^2, Lambda)

lmiterm([1 3 3 R1], tau_l^2 * Lambda, Lambda)
lmiterm([1 3 3 R2], (tau_u - tau_l)^2 * Lambda, Lambda)

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
lmiterm([-7,1,2,S'],1,1)
lmiterm([-7,2,1,S],1,1)
lmiterm([7,0,0,0],0)

lmis = getlmis;
[tmin, xfeas] = feasp(lmis,[0,0,0,0,0],-1);
%% 结果验证
PP = dec2mat(lmis, xfeas, P)
QQ1 = dec2mat(lmis, xfeas, Q1)
QQ2 = dec2mat(lmis, xfeas, Q2)
RR1 = dec2mat(lmis, xfeas, R1)
RR2 = dec2mat(lmis, xfeas, R2)
SS = dec2mat(lmis, xfeas, S)
W = zeros(10*n+2*d*n,10*n+2*d*n);

W(1:2*n,1:2*n)=QQ1+QQ2-RR1;
W(1:2*n,2*n+1:4*n)=-PP;    W(2*n+1:4*n,1:2*n)=W(1:2*n,2*n+1:4*n).';
W(1:2*n,4*n+1:6*n)=-PP*Lambda;    W(4*n+1:6*n,1:2*n)=W(1:2*n,4*n+1:6*n).';
W(1:2*n,6*n+1:8*n)=RR1;    W(6*n+1:8*n,1:2*n)=W(1:2*n,6*n+1:8*n).';

W(2*n+1:4*n,2*n+1:4*n)=tau_l^2*RR1+(tau_u-tau_l)^2*RR2;
W(2*n+1:4*n,4*n+1:6*n)=tau_l^2*RR1*Lambda+(tau_u-tau_l)^2*RR2*Lambda;    W(4*n+1:6*n,2*n+1:4*n)=W(2*n+1:4*n,4*n+1:6*n).';

W(4*n+1:6*n,4*n+1:6*n)=tau_l^2*Lambda*RR1*Lambda+(tau_u-tau_l)^2*Lambda*RR2*Lambda;

W(6*n+1:8*n,6*n+1:8*n)=-QQ1-RR1-RR2;
W(6*n+1:8*n,8*n+1:10*n)=SS;
for i = 1:d
    W(6*n+1:8*n,(4+i)*2*n+1:(5+i)*2*n) = 1/d*(RR2-SS);
    W((4+i)*2*n+1:(5+i)*2*n,6*n+1:8*n) = W(6*n+1:8*n,(4+i)*2*n+1:(5+i)*2*n).' ;
end

W(8*n+1:10*n,8*n+1:10*n)=-QQ2-RR2;
for i = 1:d
    W(8*n+1:10*n,(4+i)*2*n+1:(5+i)*2*n) = 1/d*(RR2-SS.');
    W((4+i)*2*n+1:(5+i)*2*n,8*n+1:10*n) = W(8*n+1:10*n,(4+i)*2*n+1:(5+i)*2*n).' ;
    W((4+i)*2*n+1:(5+i)*2*n,(4+i)*2*n+1:(5+i)*2*n)=-1/d*(2*RR2-S.'-S);
end

e = eig(W) 