% LMI example
clear;clc

n = 6;
tau_l = 0.002;
tau_u = 0.3;%0.5 0.03
K = 30*eye(6);%diag([-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]);

% follower不会和leader碰撞的网络拓扑
L1 = [2,  0,  0,  0,  0,  0;
     -1,  2,  0,  0,  0,  0;
      0, -1,  3, -1,  0,  0;
      0,  0,  0,  2,  0,  0;
      0, -1,  0,  0,  2,  0;
      0,  0, -1,  0, -1,  2 ]; % 跟随者邻接矩阵
B = diag(diag(L1));
A_F = B-L1;
L2 =[ -1,  0,  0, -1;
       0, -1,  0,  0;
       0,  0, -1,  0;
       0, -1,  0, -1;
       0,  0, -1,  0;
       0,  0,  0,  0];
A_LF = -L2;

% A_F = [0,1,0,0,0,0;
%        0,0,0,0,0,0;
%        0,0,0,0,0,0;
%        0,1,0,0,0,0;
%        1,0,0,0,0,1;
%        0,0,1,0,1,0]; % 跟随者邻接矩阵
% A_LF = [1,0,0,0;
%         0,1,1,0;
%         0,0,0,1;
%         1,0,0,0;
%         0,1,0,0;
%         0,0,0,0]; % 领导者和跟随者的耦合邻接矩阵
A = [A_F,A_LF];
% B = [1,0,0,0,0,0;
%      0,2,0,0,0,0;
%      0,0,1,0,0,0;
%      0,0,0,1,0,0;
%      0,0,0,0,1,0;
%      0,0,0,0,0,0]; % 跟随者能接受到的领导者信息总和
% 
d = sum(sum(A));
% L = -A_F;
num_follower = 6;
% for i = 1:num_follower
%     L(i,i) = sum(A_F(i,:));
% end
% 
% L1 = L + B;
% L2 = -A_LF;

L1Ni =  kron(inv(L1), eye(2));

count=1;
L_ce = zeros(num_follower,num_follower,d);
L_cell = cell(d,1);
L2_ce = zeros(num_follower,4,d);

for i = 1:num_follower
    % sum1 = 0;
    for j = 1:num_follower+4
        if A(i,j)==1
            L_ce(i,i,count)=1;
            if j<=num_follower
                L_ce(i,j,count)=-1;
                % sum1=sum1+L_ce(:,:,count)*inv(L1)*L2;
                % i
                % j
                % L_ce(:,:,count)
            end
            if j>num_follower
                L2_ce(i,j-num_follower,count)=-1;
                % sum1=sum1-L2_ce(:,:,count);
                % i
                % j
                % L2_ce(:,:,count)
            end
            L_cell{count}=kron(K*L_ce(:,:,count),eye(2));
            count = count+1;
        end
    end
    % sum1;
end
% sum=0;
% for i = 1:d
%     (L_ce(:,:,i)*inv(L1)*L2-L2_ce(:,:,i))%*ones(4,1)
%     if L_ce(:,:,i)*inv(L1)*L2==L2_ce(:,:,i)
%         disp(i)
%     end
    % sum=sum+L_ce(:,:,i)*inv(L1)*L2-L2_ce(:,:,i)
% end
%% 把拉普拉斯矩阵加进来的版本
setlmis([])
[P, ~, sP] = lmivar(1, [2*n 1]); % P是对称矩阵，阶数为2*n，满块矩阵
Q1 = lmivar(1, [2*n 1]);
Q2 = lmivar(1, [2*n 1]); % Q是对称矩阵，阶数为2*n，满块矩阵
R1 = lmivar(1, [2*n 1]);
R2 = lmivar(1, [2*n 1]);
S = lmivar(2, [2*n 2*n]); % S是矩阵，阶数为2*n，满块矩阵

% lmiterm([1 1 1 R1], tau_l^2, 1)
% lmiterm([1 1 1 R2], (tau_u - tau_l)^2, 1)
% lmiterm([1 1 1 P], -lambdaNi, 1, 's')
lmiterm([1 1 1 Q1], 1, 1)
lmiterm([1 1 1 Q2], 1, 1)
lmiterm([1 1 1 R1], -1, 1)

lmiterm([1 1 2 R1], 1, 1)

for i =1:d
    lmiterm([1 1 3+i P], -1, L_cell{i})
end

lmiterm([1 2 2 Q1], -1, 1)
lmiterm([1 2 2 R1], -1, 1)
lmiterm([1 2 2 R2], -1, 1)

lmiterm([1 2 3 S], 1, 1)

for i = 1:d
    lmiterm([1 2 3+i R2], 1/d, 1)
    lmiterm([1 2 3+i S], -1/d, 1)
end

lmiterm([1 3 3 Q2], -1, 1)
lmiterm([1 3 3 R2], -1, 1)

for i = 1:d
    lmiterm([1 3 3+i R2], 1/d, 1)
    lmiterm([1 3 3+i -S], -1/d, 1)
end

for i = 1:d
    lmiterm([1 3+i 3+i R2], -1/d, 1, 's')
    lmiterm([1 3+i 3+i S], 1/d, 1, 's')
end

for i = 1:d
    for j = i:d
        lmiterm([1 3+i 3+j R1], tau_l^2*L_cell{i}.', L_cell{j})
        lmiterm([1 3+i 3+j R2], (tau_u - tau_l)^2*L_cell{i}.', L_cell{j})
    end
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
%%
lmis = getlmis;
[tmin, xfeas] = feasp(lmis);%,[0,0,0,0,0],-10^(-10)
%% 结果验证
PP = dec2mat(lmis, xfeas, P);PP*1000
QQ1 = dec2mat(lmis, xfeas, Q1);QQ1*1000
QQ2 = dec2mat(lmis, xfeas, Q2);QQ2*1000
RR1 = dec2mat(lmis, xfeas, R1);RR1*1000
RR2 = dec2mat(lmis, xfeas, R2);RR2*1000
SS = dec2mat(lmis, xfeas, S);SS*1000
eig(PP)
eig(QQ1)
eig(QQ2)
eig(RR1)
eig(RR2)
zuhe=[RR2,SS.';SS,RR2];
eig(zuhe)