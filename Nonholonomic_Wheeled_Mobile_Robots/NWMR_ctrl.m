function [sys,x0,str,ts,simStateCompliance] = ctrl(t,x,u,flag,pa)
% 控制输入
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u,pa);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;  
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys= mdlUpdate (t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u,pa)
xd=u(1);
yd=u(2);
thetad=u(3);
x=u(4);
y=u(5);
theta=u(6);

xe=xd-x;
ye=yd-y;
thetae=thetad-theta;

e1=xe*cos(theta)+ye*sin(theta);
e2=-xe*sin(theta)+ye*cos(theta);
e3=thetae;

k=0.1;
% 1 
w=k*e2/e1;
% v=e2*w+k*e1;
v=k*e3*e2+k*e1;

% % 2
% xdot=k*xe;
% ydot=k*ye;
% v=sqrt(xdot^2+ydot^2);
% w=atan2(ydot,xdot)-theta;


sys = [v;w];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];