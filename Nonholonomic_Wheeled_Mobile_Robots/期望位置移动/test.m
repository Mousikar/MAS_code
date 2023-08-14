%PID parameter define
clc;clear;
target=10100; %define target value
CurrentValue(1)=3;%define current value
ErrorAllowable=1;% define allowable error 
PC_Coefficient=2.1;%define the related coefficient between PID output and current value

PIDPara=struct;%create PID parameter structure
PIDPara.Kp=0.5;%proportionality coefficient 
PIDPara.Ki=0.1;%integration coefficient
PIDPara.Kd=0.1;%differential coefficient
PIDPara.ErrorPre=0;% deviation of previous time step
PIDPara.ErrorCurrent=0;% deviation of current time step
PIDPara.ErrorInte=0;% the cumulated deviation
PIDPara.ErrorDiff=0;% diff divation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%analysis of PID process
[PID_new,PID_output]= PID_demo(PIDPara,target,CurrentValue(1));
for a=1:1:10000 % to aviod too many cycles ,it is limited to 10000 cycles
    a=a+1;
    CurrentValue(a)=PID_output*PC_Coefficient;
    PIDPara=PID_new;
    [PID_new,PID_output]= PID_demo(PIDPara,target,CurrentValue(a));
    if(abs(PIDPara.ErrorCurrent)<ErrorAllowable)
        break
    end
end
disp(PIDPara.ErrorCurrent) %display the final error
plot(CurrentValue) %plot the value curve

