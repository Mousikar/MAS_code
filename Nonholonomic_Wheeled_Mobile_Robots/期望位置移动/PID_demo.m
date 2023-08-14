function  [PID_new,PID_output] = PID_demo(PIDPara,target,CurrentValue)
%Newvalue is the current value,PIDPara,target,CurrentValue
    Error=target-CurrentValue;%get divation
    PIDPara.ErrorDiff=PIDPara.ErrorCurrent-PIDPara.ErrorPre;%diff divation
    PIDPara.ErrorInte=PIDPara.ErrorInte+Error;%cumulated divation
    PIDPara.ErrorPre=PIDPara.ErrorCurrent;%swith current divation to previous divation 
    PIDPara.ErrorCurrent=Error; %save current divation
    PID_new=PIDPara; %save new PID parameters
    PID_output=PIDPara.Kp.*Error+PIDPara.Ki.*PIDPara.ErrorInte+PIDPara.Kd.*PIDPara.ErrorDiff;
end