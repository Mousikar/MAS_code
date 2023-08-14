function [sys,x0,str,ts,simStateCompliance] = SMC_input(t,x,u,flag,pa)
% 期望位置和角度，期望位置导数和角度导数
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
% 输出是：xd   yd thetad
sizes.NumOutputs     = 6;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
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
A=pa.A;
T=pa.T;

xd = t;
yd= A*sin(2*pi/T*t);
dotyd=2*pi/T*A*cos(2*pi/T*t);
dotxd=1;
thetad = atan2(dotyd,dotxd);
dotthetad=-1/(dotyd^2 + 1)*2*pi/T*2*pi/T*A*sin(2*pi/T*t);

% xd = 6;
% yd= 10;
% dotyd=0;
% dotxd=0;
% thetad = 2*pi/3;
% dotthetad=0;

xd = A*sin(2*pi/T*t);
yd= t;
dotxd=2*pi/T*A*cos(2*pi/T*t);
dotyd=1;
thetad = atan2(dotyd,dotxd);
dotthetad=0;

sys = [xd;yd;thetad;dotxd;dotyd;dotthetad];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;  
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];