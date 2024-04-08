% LMI example
clear;
% clc
tic
n = 6;
tau_u = 0.5 %0.03
K = 0.3*eye(6) % 5*eye(6)

A_F = [      0     0     0     0     0     0
             1     0     0     0     0     0
             0     1     0     1     0     0
             0     0     0     0     0     0
             0     1     0     0     0     0
             1     0     1     0     0     0]; % 跟随者邻接矩阵
A_LF = [     1     0     0     1
             0     1     0     0
             0     0     1     0
             0     0     1     1
             1     0     0     0
             0     0     0     0]; % 领导者和跟随者的耦合邻接矩阵
A = [A_F,A_LF];
B = diag(sum(A')); % 跟随者能接受到的领导者信息总和
d = sum(sum(A));
L = -A_F;
num_follower = 6;

L1 = L + B;
L2 = -A_LF;

count=1;
L_ce = zeros(num_follower,num_follower,d);
B_cell = cell(d,1);
L2_ce = zeros(num_follower,4,d);
A_cell = kron([zeros(num_follower),eye(num_follower);zeros(num_follower),zeros(num_follower)],eye(2));

for i = 1:num_follower
    for j = 1:num_follower+4
        if A(i,j)==1
            L_ce(i,i,count)=1;
            if j<=num_follower
                L_ce(i,j,count)=-1;
            end
            if j>num_follower
                L2_ce(i,j-num_follower,count)=-1;
            end
            B_cell{count}=kron(...
                [zeros(num_follower),zeros(num_follower);L_ce(:,:,count),K*L_ce(:,:,count)]...
                ,eye(2));
            count = count+1;
        end
    end
end
%% 初始化
setlmis([])
[P, ~, sP] = lmivar(1, [4*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q = lmivar(1, [4*n 1]);
R = lmivar(1, [4*n 1]);
S = lmivar(2, [4*n 4*n]); % S是矩阵，阶数为2*n，满块矩阵

lmiterm([1 1 1 P], 1, A_cell, 's')
lmiterm([1 1 1 Q], 1, 1)
lmiterm([1 1 1 R], tau_u^2*A_cell.', A_cell)
lmiterm([1 1 1 R], -1, 1)

lmiterm([1 1 2 S], 1, 1)

for i =1:d
    lmiterm([1 1 2+i P], -1, B_cell{i})
    lmiterm([1 1 2+i R], -tau_u^2*A_cell.', B_cell{i})
    lmiterm([1 1 2+i R], 1/d, 1)
    lmiterm([1 1 2+i S], -1/d, 1)
end

lmiterm([1 2 2 Q], -1, 1)
lmiterm([1 2 2 R], -1, 1)

for i = 1:d
    lmiterm([1 2 2+i R], 1/d, 1)
    lmiterm([1 2 2+i -S], -1/d, 1)
end

for i = 1:d
    lmiterm([1 2+i 2+i R], -1/d, 1, 's')
    lmiterm([1 2+i 2+i S], 1/d, 1, 's')
end

for i = 1:d
    for j = i:d
        lmiterm([1 2+i 2+j R], tau_u^2*B_cell{i}.', B_cell{j})
    end
end

lmiterm([-2,1,1,P],1,1)
lmiterm([2,1,1,0],0)

lmiterm([-3,1,1,Q],1,1)
lmiterm([3,1,1,0],0)

lmiterm([-4,1,1,R],1,1)
lmiterm([5,1,1,0],0)
% 
% lmiterm([-5,1,1,R],1,1)
% lmiterm([-5,2,2,R],1,1)
% lmiterm([-5,2,1,S],1,1)
% lmiterm([5,1,1,0],0)
%% 求解
lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-10^(-10)
%% 结果验证
PP = dec2mat(lmis, xfeas, P);PP*1000
QQ = dec2mat(lmis, xfeas, Q);QQ*1000
RR = dec2mat(lmis, xfeas, R);RR*1000
SS = dec2mat(lmis, xfeas, S);SS*1000
eig(PP)
eig(QQ)
eig(RR)
zuhe=[RR,SS.';SS,RR];
eig(zuhe)
toc