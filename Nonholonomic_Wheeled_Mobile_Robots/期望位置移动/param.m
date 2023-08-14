pa=struct('A',1, ...
    'T',20, ...
    'k',0.9);

global PIDPara1
global PIDPara2
PIDPara1=struct;%create PID parameter structure
PIDPara1.Kp=0.1;%proportionality coefficient 
PIDPara1.Ki=0.0;%integration coefficient
PIDPara1.Kd=0.0;%differential coefficient
PIDPara1.ErrorPre=0;% deviation of previous time step
PIDPara1.ErrorCurrent=0;% deviation of current time step
PIDPara1.ErrorInte=0;% the cumulated deviation
PIDPara1.ErrorDiff=0;% diff divation
PIDPara2=struct;%create PID parameter structure
PIDPara2.Kp=0.5;%proportionality coefficient 
PIDPara2.Ki=0.0;%integration coefficient
PIDPara2.Kd=0.0;%differential coefficient
PIDPara2.ErrorPre=0;% deviation of previous time step
PIDPara2.ErrorCurrent=0;% deviation of current time step
PIDPara2.ErrorInte=0;% the cumulated deviation
PIDPara2.ErrorDiff=0;% diff divation

% PIDPara1=zeros(7,1);
% PIDPara1(1)=0.1;
% PIDPara1(2)=0.0;
% PIDPara1(3)=0.0;
% 
% PIDPara2=zeros(7,1);
% PIDPara2(1)=0.1;
% PIDPara2(2)=0.0;
% PIDPara2(3)=0.0;