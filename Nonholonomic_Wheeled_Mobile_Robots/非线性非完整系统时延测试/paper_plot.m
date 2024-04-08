% paper plot
%%
currentDateTime = datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss');

x_history = double(x_history);
y_history = double(y_history);
rx_history = double(rx_history);
ry_history = double(ry_history);
theta_history = double(theta_history);
v_history = double(v_history);
omega_history = double(omega_history);
hat_ex_history = double(hat_ex_history);
hat_ey_history = double(hat_ey_history);
errx_actual_history = double(errx_actual_history);
erry_actual_history = double(erry_actual_history);
hat_evx_history = double(hat_evx_history);
hat_evy_history = double(hat_evy_history);
%% 轨迹图
figure;

% set(gcf,'Position',[100,100,900,600]); % 设置图形大小
set(gcf,'Position',[50,50,800,600]); % 设置图形大小
hold on;
leader_scatter = scatter(rx, ry, 50, 'r', 'filled', 'DisplayName', 'Leaders');
plot(x_history,y_history, 'LineWidth', 1.5)
legend("Leaders","Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Location","best",'FontName','Times New Roman','FontSize',12)
follower_scatter = scatter(x, y, 50, 'b', 'filled');
hull = convhull(rx, ry);
hull_line = plot(rx(hull), ry(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
xlabel('x/(m)','FontName','Times New Roman','FontSize',12);
ylabel('y/(m)','FontName','Times New Roman','FontSize',12);
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '.png']);

%% 误差图 位置误差 速度误差
figure;
hold on;

set(gcf,'Position',[100,100,800,600]); % 设置图形大小
set(gcf,'Position',[50,50,800,600]); % 设置图形大小
uijmvz=linspace(0,t_sum,length(erry_actual_history));

% latexStr=['$$m \ddot y = C_D \cdot {1 \over 2} \rho {\dot y}^2 \cdot A$$'] ;
% lgh=legend(latexStr，'FontSize',20);
% set(lgh,'interpreter','latex');

subplot(4, 1, 1);
for i = 1:num_follower
    plot(uijmvz,errx_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\xi_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',12,'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',12);
ylabel('x/(m)','FontName','Times New Roman','FontSize',12);
set(lgh,'interpreter','latex');

subplot(4, 1, 2);
for i = 1:num_follower
    plot(uijmvz,erry_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\xi_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',12,'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',12);
ylabel('y/(m)','FontName','Times New Roman','FontSize',12);
set(lgh,'interpreter','latex');
hold off;

subplot(4, 1, 3);
for i = 1:num_follower
    plot(uijmvz,hat_evx_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\dot{\xi}_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',12, 'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',12);
ylabel('x/(m/s)','FontName','Times New Roman','FontSize',12);
set(lgh,'interpreter','latex');
% set(xla, 'interpreter','latex');

subplot(4, 1, 4);
for i = 1:num_follower
    plot(uijmvz,hat_evy_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\dot{\xi}_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',12,'Location', 'best','Orientation','horizontal');
set(lgh,'interpreter','latex');
xlabel('t/(s)','FontName','Times New Roman','FontSize',12);
ylabel('y/(m/s)','FontName','Times New Roman','FontSize',12);
hold off;

fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '_error.png']);

%% leader静止时三维图
% figure;
% set(gcf,'Position',[100,100,800,600]); % 设置图形大小
% set(gcf,'Position',[50,50,800,600]); % 设置图形大小
% uijmvz=linspace(0,t_sum,length(x_history));
% hold on;
% 
% leader_scatter = scatter3([0,0,0,0], rx, ry, 'r', 'filled', 'DisplayName', 'Leaders');
% plot3(uijmvz, x_history,y_history)
% legend("Leaders","Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Location","best")
% follower_scatter = scatter([0,0,0,0,0,0],x, y, 'b', 'filled');
% hull = convhull(rx, ry);
% hull_line = plot3([0,0,0,0,0], rx(hull), ry(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
% xlabel('t/(s)');
% ylabel('x/(m)');
% zlabel('y/(m)')
% view([1,1,1])
% fig = gcf;
% fig.PaperPositionMode = 'auto';
% fig_pos = fig.PaperPosition;
% fig.PaperSize = [fig_pos(3) fig_pos(4)];
% 
% saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '_3d.png']);
%% leader运动时三维图
figure;
set(gcf,'Position',[50,50,800,600]); % 设置图形大小

% set(gcf,'Position',[100,100,800,600]); % 设置图形大小
uijmvz=linspace(0,t_sum+dmax*T,length(x_history));
hold on;
% rxc=[2 4 4 2];
% ryc=[2 2 4 4];
% 动态leaders
rxc=[2 3 3 2];
ryc=[2 2 3 3];
jpqu=iter+dmax;
scatter3([0,0,0,0], rxc, ryc, 'r', 'filled', 'DisplayName', 'Leaders');
plot3(uijmvz(1:jpqu), x_history(1:jpqu,:),y_history(1:jpqu,:),'-','LineWidth',1)
plot3(uijmvz(1:jpqu), rx_history(1:jpqu,:),ry_history(1:jpqu,:),'r--','LineWidth',1)

legend("Leaders","Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6",'FontName','Times New Roman','FontSize',12,"Location","southeast")
follower_scatter = scatter3(jpqu/1000*ones(1,6),x_history(jpqu,:), y_history(jpqu,:), 'b', 'filled');
follower_scatter = scatter3(jpqu/1000*zeros(1,6),x_history(1,:), y_history(1,:), 'b', 'filled');
hull = convhull(rxc, ryc);
plot3([0,0,0,0,0], rxc(hull), ryc(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

hull = convhull(rx_history(jpqu,:), ry_history(jpqu,:));
rxt=rx_history(jpqu,:);
ryt=ry_history(jpqu,:);
leader_scatter = scatter3(jpqu/1000*ones(1,4), rxt, ryt, 'r', 'filled', 'DisplayName', 'Leaders');
hull_line = plot3(jpqu/1000*ones(1,5), rxt(hull), ryt(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

xlabel('t/(s)','FontName','Times New Roman','FontSize',12);
ylabel('x/(m)','FontName','Times New Roman','FontSize',12);
zlabel('y/(m)','FontName','Times New Roman','FontSize',12)
view([-0.3,-1,0.5])

fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '_move3d.png']);