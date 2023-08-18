clc;        clear;      close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  event-triggered control
%%% Event-Triggered Real-Time Scheduling of Stabilizing Control Tasks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% system define
A = [0 1; -2 3;];
B = [0; 1];
K = [1 -4];
P = [1 0.25; 0.25 1 ];
Q = [0.5 0.25; 0.25 1.5];
x0 = [0.5 0.5]';


%% event triggered controller
sigma = 0.05;

%% simulation
x = x0;         xBuf = [];
u = K*x0;    uBuf = [];
dt = 0.001;
Ad = expm(A*dt);
syms s
Bd = int(expm(A*s),0,dt)*B;
Bd = eval(Bd);
V = 0;          vBuf = []

for step = 1:10000
    e = x-x0;
    if norm(e) >= sigma*norm(x)
        x0 = x;
        u = K*x0;
    end
    x = Ad*x + Bd*u;
    
    V = norm(e);
    
    vBuf = [vBuf V];
    xBuf = [xBuf x];
    uBuf = [uBuf u];
end

%% plot
figure(1)
hold on
plot(xBuf');
plot(uBuf)
figure(2)
plot(vBuf)