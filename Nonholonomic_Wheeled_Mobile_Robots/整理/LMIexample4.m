% LMI example
clear;clc

n = 6;
tau_l = 0.002;
tau_u = 0.3;dtau_u=2;
K = 30*eye(6);%diag([-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]);

% follower不会和leader碰撞的网络拓扑
L1 = [2,  0,  0,  0,  0,  0;
     -1,  2,  0,  0,  0,  0;
      0, -1,  3, -1,  0,  0;
      0,  0,  0,  2,  0,  0;
      0, -1,  0,  0,  2,  0;
      0,  0, -1,  0, -1,  2 ]; % 跟随者邻接矩阵
B = diag(diag(L1));

L2 =[ -1,  0,  0, -1;
       0, -1,  0,  0;
       0,  0, -1,  0;
       0, -1,  0, -1;
       0,  0, -1,  0;
       0,  0,  0,  0];

Ma = kron(inv(L1)*inv(K),eye(2));
Mb = kron(inv(L1)*inv(K)*inv(B)*(L1-B),eye(2));

%% 不太行，算了
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q = lmivar(1, [2*n 1]);
R = lmivar(1, [2*n 1]);

lmiterm([1 1 1 Q], 1, 1)
lmiterm([1 1 1 R], -1, 1)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 R], -1, Ma)

lmiterm([1 1 3 R], 1, Mb)

lmiterm([1 2 2 R], tau_u, 1)
lmiterm([1 2 2 Q], -(1-dtau_u)/tau_u*Ma.', Ma)
lmiterm([1 2 2 R], -Ma.', Ma)

lmiterm([1 2 3 Q], (1-dtau_u)*Ma.', Mb)
lmiterm([1 2 3 R], Ma.', Mb)

lmiterm([1 3 3 Q], -(1-dtau_u)*Mb.', Mb)
lmiterm([1 3 3 R], -Mb.', Mb)

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,R],1,1)
lmiterm([4,1,1,0],0)
%% 再试一次
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q = lmivar(1, [2*n 1]);

lmiterm([1 1 1 Q], -1, 1)

lmiterm([1 1 2 P], 1, 1)
lmiterm([1 1 2 Q], -1, Ma)

lmiterm([1 1 3 Q], -1, Mb)

lmiterm([1 2 2 Q], tau_u^2, 1)
lmiterm([1 2 2 Q], -Ma.', Ma)

lmiterm([1 2 3 Q], -Ma.', Mb)

lmiterm([1 3 3 Q], -Mb.', Mb)

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q],1,1)
lmiterm([3,1,1,0],0)

%%
lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-10^(-10)
%% 结果验证
PP = dec2mat(lmis, xfeas, P)
QQ = dec2mat(lmis, xfeas, Q)
RR = dec2mat(lmis, xfeas, R)
eig(PP)
eig(QQ)
eig(RR)