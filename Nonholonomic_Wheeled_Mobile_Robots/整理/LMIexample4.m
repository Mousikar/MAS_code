% LMI example
clear;clc

n = 6;
tau_l = 0.002;
tau_u = 0.3;
K = 0.1*eye(6);%diag([-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]);

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

laf = kron(inv(L1)*inv(K),eye(2));
lad = kron(inv(L1)*inv(K)*inv(B),eye(2));
eig(eye(2*n)-lad)
%% 不太行，算了
% setlmis([])
% [P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
% Q = lmivar(1, [2*n 1]);
% R = lmivar(1, [2*n 1]);
% 
% lmiterm([1 1 1 Q], laf.', laf)
% lmiterm([1 1 1 R], -laf.', laf)
% 
% lmiterm([1 1 2 P], -laf.', 1)
% lmiterm([1 1 2 Q], laf.', lad)
% lmiterm([1 1 2 R], -laf.', lad)
% 
% lmiterm([1 1 3 R], -laf.', 1)
% 
% lmiterm([1 2 2 P], -lad.', 1,'s')
% lmiterm([1 2 2 Q], lad.', lad)
% lmiterm([1 2 2 R], tau_u^2, 1)
% lmiterm([1 2 2 R], -lad.', lad)
% 
% lmiterm([1 2 3 R], -lad.', 1)
% 
% lmiterm([1 3 3 Q], -1, 1)
% lmiterm([1 3 3 R], -1, 1)
% 
% lmiterm([-2,1,1,P],1,1)
% lmiterm([2,1,1,0],0)
% 
% lmiterm([-3,1,1,Q],1,1)
% lmiterm([3,1,1,0],0)
% 
% lmiterm([-4,1,1,R],1,1)
% lmiterm([4,1,1,0],0)
% %%
% lmis = getlmis;
% [tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-10^(-10)
% %% 结果验证
% PP = dec2mat(lmis, xfeas, P)
% QQ = dec2mat(lmis, xfeas, Q)
% RR = dec2mat(lmis, xfeas, R)
% eig(PP)
% eig(QQ)
% eig(RR)