xdydthetad=out.xdydthetad;
xytheta=out.xytheta;
dtheta=out.dtheta;
plot(xytheta(:,1),xytheta(:,2))
hold on
plot(xdydthetad(:,1),xdydthetad(:,2))
figure
plot(xytheta(:,3))
hold on
plot(dtheta(:,3))