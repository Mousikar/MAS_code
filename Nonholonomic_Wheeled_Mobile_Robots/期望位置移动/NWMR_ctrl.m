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
sizes.NumInputs      = 9;
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
dotxd=u(4);
dotyd=u(5);
dotthetad=u(6);
x=u(7);
y=u(8);
theta=u(9);

% 3 还没找到解释xdot=k*xe;ydot=k*ye;v=xdot^2+ydot^2;v=sqrt(xdot^2+ydot^2);
k=pa.k;
xe=dotxd+k*(xd-x);
ye=dotyd+k*(yd-y);
thetae=dotthetad+k*(thetad-theta);

v=sqrt(xe^2+ye^2);
w=atan2(ye,xe)-theta;

% % PID
% % 模拟控制过程
% global PIDPara1
% global PIDPara2
% [PIDPara1,v] = PID_demo(PIDPara1,0,-sqrt((xd-x)^2+(yd-y)^2));
% [PIDPara2,w] = PID_demo(PIDPara2,0,-(atan2(yd-y,xd-x)-theta));

sys = [v;w];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];
