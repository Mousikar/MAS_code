% zhihuLMI
clear
% 控制不了
% K=1.0e+03 *  [ 
%    -2.3338   -2.7518    2.5209    2.9827   -1.5301   -1.8200
%     2.3249    2.7616    2.5214    2.9829   -1.5394   -1.8101
%    -2.3330   -2.7513   -2.5209   -2.9827   -1.5307   -1.8199
%     2.3242    2.7611   -2.5214   -2.9829   -1.5388   -1.8102];

% 可以控制的
% K=1e3*[
%     0.6617    0.9256   -0.5191   -0.8045    0.3458    0.5108;
%    -0.8533   -1.1158   -0.6961   -0.9815    0.3458    0.5311;
%     0.8533    1.1171    0.6961    0.9815    0.3458    0.5108;
%    -0.6617   -0.9242    0.5191    0.8045    0.3458    0.5311];
%
I = [609,659,401];
wt = 0.0667;
A = [0                      1                        0                       0 0                     0;
    4*(I(3)-I(2))*wt^2/I(1) 0                        0                       0 0                     (I(1)+I(3)-I(2))*wt/I(1);
    0                       0                        0                       1 0                     0;
    0                       0                        3*(I(3)-I(1))*wt^2/I(2) 0 0                     0;
    0                       0                        0                       0 0                     1;
    0                       (I(2)-I(1)-I(3))*wt/I(3) 0                       0 (I(1)-I(2))*wt^2/I(3) 0];
B0 = [0 0 0; 1/I(1) 0 0; 0 0 0; 0 1/I(2) 0; 0 0 0; 0 0 1/I(3)];
S = [sqrt(1/3) -sqrt(1/3) sqrt(1/3) -sqrt(1/3);
     -sqrt(1/3) -sqrt(1/3) sqrt(1/3) sqrt(1/3);
     sqrt(1/3) sqrt(1/3) sqrt(1/3) sqrt(1/3)];
B = B0*S;
C=eye(6);%%%%
Bw = eye(size(A,1));


A_dis=zeros(6,6);
B_dis=zeros(6,4);
Bw_dis=zeros(6,6);
[A_dis(:,:),B_dis(:,:)] =c2d(A,B,0.1);%采样时间为0.1时，将矩阵A、B转换为离散时间的矩阵A_dis、B_dis
[A_dis(:,:),Bw_dis(:,:)]=c2d(A,Bw,0.1);
% %% 修改2
% setlmis([])
% Pg=lmivar(1,[6,1]);%创建6阶对称块矩阵变量%%
% Qg=lmivar(1,[6,1]);%创建6阶对称块矩阵变量%%
% Kg=lmivar(2,[4,6]);%创建4x6的矩阵变量%%%
% gamaf=lmivar(1,[1,0]);%创建一个1阶对角矩阵
% 
% LMI_i1=newlmi;
% lmiterm([LMI_i1,1,1,Pg],-1,1);%左项，第一行第一列，变量Pg1，左乘-1，右乘1
% 
% lmiterm([LMI_i1,2,2,gamaf],-eye(6),1);%左项，第二行第二列，变量gamaf，左乘-1，右乘1
% 
% lmiterm([LMI_i1,4,1,Qg],C,1);%左项，第三行第一列，变量Qg，左乘矩阵C
% 
% lmiterm([LMI_i1,4,4,0],-eye(6));%左项，第三行第三列，无变量，负的三阶单位矩阵
% 
% lmiterm([LMI_i1,3,1,Qg],A_dis,1);%左项，第四行第一列，变量Qg，左乘矩阵A_dis(:,:)，右乘1
% lmiterm([LMI_i1,3,1,Kg],-B_dis,1);%左项，第四行第一列，变量Kg1,左乘lamda(1)*B_(:,:,1)，右乘1
% 
% lmiterm([LMI_i1,3,2,0],Bw_dis);%左项，第四行第二列,无变量，左乘D_dis(:,:,1)
% 
% lmiterm([LMI_i1,3,3,Qg],-1,1,'s');%左项，第四行第四列，变量Qg，左乘-1，右乘1
% lmiterm([LMI_i1,3,3,Pg],1,1);%左项，第四行第四列，变量Pg1，左乘1，右乘1
% 
% 
% lmisys=getlmis;
% [tmin,xfeas]=feasp(lmisys);%,[0,0,1000,0,0],-1);%Call feasp to a find a feasible decision vector
% 
% gamafM=dec2mat(lmisys,xfeas,gamaf);
% PM=dec2mat(lmisys,xfeas,Pg)
% QM=dec2mat(lmisys,xfeas,Qg)
% Kg1M=dec2mat(lmisys,xfeas,Kg);
% 
% K1=Kg1M*inv(QM)
% gama=sqrt(gamafM)
% %% 修改1
% setlmis([])
% P=lmivar(1,[6,1]);%创建6阶对称块矩阵变量%%
% % K=lmivar(2,[4,6]);%创建4x6的矩阵变量%%%
% gamaf=lmivar(1,[1,0]);%创建一个1阶对角矩阵
% 
% xishu=A+B*K;
% LMI_i1=newlmi;
% lmiterm([LMI_i1,1,1,P],xishu.',xishu);
% lmiterm([LMI_i1,1,1,P],-1,1);
% lmiterm([LMI_i1,1,1,0],C.'*C);
% lmiterm([LMI_i1,2,2,gamaf],-eye(6),1);
% lmiterm([LMI_i1,1,2,P],xishu.',Bw);
% 
% lmisys=getlmis;
% [tmin,xfeas]=feasp(lmisys,[0,0,1000,0,0],-1);%Call feasp to a find a feasible decision vector
% 
% gamafM=dec2mat(lmisys,xfeas,gamaf);
% PM=dec2mat(lmisys,xfeas,P)
% % Kg1M=dec2mat(lmisys,xfeas,K);
% 
% gama=sqrt(gamafM)
%% 原始
%开始求解LMI,转移概率矩阵为[0.5 0.2 0.3；? [0.5 0.6] ?；0.4 0.1 0.5]
setlmis([])
Qg=lmivar(1,[6,1]);%创建6阶对称块矩阵变量%%
Pg=lmivar(1,[6,1]);
gamaf=lmivar(1,[1,0]);%创建一个1阶对角矩阵
Kg=lmivar(2,[4,6]);%创建4x6的矩阵变量%%%


%
%开始逐个描述LMI,在i=1，2，3下分别有s=1，2，3，其中s=2时又有r=1：t=1和t=3，r=2：t=1和t=3
%when i=1;
LMI_i1=newlmi;
lmiterm([LMI_i1,1,1,Pg],-1,1);%左项，第一行第一列，变量Pg1，左乘-1，右乘1
lmiterm([LMI_i1,2,2,gamaf],-eye(6),1);%左项，第二行第二列，变量gamaf，左乘-1，右乘1
lmiterm([LMI_i1,3,1,Qg],C,1);%左项，第三行第一列，变量Qg，左乘矩阵C
lmiterm([LMI_i1,3,3,0],-eye(6));%左项，第三行第三列，无变量，负的三阶单位矩阵
lmiterm([LMI_i1,4,1,Qg],A_dis,1);%左项，第四行第一列，变量Qg，左乘矩阵A_dis(:,:)，右乘1
lmiterm([LMI_i1,4,1,Kg],-B_dis,1);%左项，第四行第一列，变量Kg1,左乘lamda(1)*B_(:,:,1)，右乘1
lmiterm([LMI_i1,4,2,0],Bw_dis);%左项，第四行第二列,无变量，左乘D_dis(:,:,1)
lmiterm([LMI_i1,4,4,Qg],-1,1,'s');%左项，第四行第四列，变量Qg，左乘-1，右乘1
lmiterm([LMI_i1,4,4,Pg],1,1);%左项，第四行第四列，变量Pg1，左乘1，右乘1
%i=1



%
%描述结束，开始获得LMI并求解矩阵变量
lmisys=getlmis;
[tmin,xfeas]=feasp(lmisys);%Call feasp to a find a feasible decision vector

gamafM=dec2mat(lmisys,xfeas,gamaf);
QM=dec2mat(lmisys,xfeas,Qg);
Kg1M=dec2mat(lmisys,xfeas,Kg);

K1=Kg1M/QM
gama=sqrt(gamafM)