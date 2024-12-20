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

size = 16;

% h1 = legend([p1, p2], 'Fx\_original', 'Fx\_control', 'Interpreter', 'latex', 'FontSize', size, 'Location', 'northwest', 'box', 'off', 'Orientation', 'horizontal');
% axis([0 time_total low up]);
% set(gca, 'FontName', 'Times New Roman', 'FontSize', size);
% xlabel('Time(s)', 'FontName', 'Times New Roman', 'FontSize', size); hold on;
% ylabel('Fx(N)', 'FontName', 'Times New Roman', 'FontSize', size, 'Rotation', 90); hold on;
%% 轨迹图
figure;

set(gcf,'Position',[50,50,800,600]); % 设置图形大小
set(gca, 'FontName', 'Times New Roman', 'FontSize',size);
hold on;
follower_scatter = scatter(x, y, 50, 'b', 'filled');
hull = convhull(rx, ry);
hull_line = plot(rx(hull), ry(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

for i = 1:num_follower
    p(i)=plot(x_history(:,i),y_history(:,i), 'LineWidth', 1.5, 'DisplayName', ['Follower ' num2str(i)]);
    hold on;
end
p(num_follower+1) = scatter(rx, ry, 50, 'r', 'filled');
lgh = legend(p, "Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Leaders",'FontName','Times New Roman','FontSize',size,'Location', 'best');
set(lgh,'interpreter','latex');

xlabel('x/(m)','FontName','Times New Roman','FontSize',size);
ylabel('y/(m)','FontName','Times New Roman','FontSize',size);
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '.png']);

%% 误差图 位置误差 速度误差
figure;

set(gca, 'FontName', 'Times New Roman', 'FontSize',size);
set(gcf,'Position',[0,0,800,800]); % 设置图形大小

hold on;
uijmvz=linspace(0,t_sum,length(erry_actual_history));

subplot(4, 1, 1);
for i = 1:num_follower
    plot(uijmvz,errx_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\xi_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',size,'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',size);
ylabel('x/(m)','FontName','Times New Roman','FontSize',size);
set(lgh,'interpreter','latex');

ax = gca;  % 抓取当前坐标区或图，这个语句必须要有！！！！
ax.XAxis.FontSize = size;% 设置x轴刻度值的字号
ax.YAxis.FontSize = size;  % 设置y轴刻度值的字号
ax.XAxis.FontName = 'Times New Roman';
ax.YAxis.FontName = 'Times New Roman';
xticks(0:1:10)
yticks(-2:1:2)

subplot(4, 1, 2);
for i = 1:num_follower
    plot(uijmvz,erry_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\xi_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',size,'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',size);
ylabel('y/(m)','FontName','Times New Roman','FontSize',size);
set(lgh,'interpreter','latex');
hold off;
ax = gca;  % 抓取当前坐标区或图，这个语句必须要有！！！！
ax.XAxis.FontSize = size;% 设置x轴刻度值的字号
ax.YAxis.FontSize = size;  % 设置y轴刻度值的字号
ax.XAxis.FontName = 'Times New Roman';
ax.YAxis.FontName = 'Times New Roman';
xticks(0:1:10)
yticks(-2:1:2)

subplot(4, 1, 3);
for i = 1:num_follower
    plot(uijmvz,hat_evx_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\dot{\xi}_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',size, 'Location', 'best','Orientation','horizontal');
xlabel('t/(s)','FontName','Times New Roman','FontSize',size);
ylabel('x/(m/s)','FontName','Times New Roman','FontSize',size);
set(lgh,'interpreter','latex');
ax = gca;  % 抓取当前坐标区或图，这个语句必须要有！！！！
ax.XAxis.FontSize = size;% 设置x轴刻度值的字号
ax.YAxis.FontSize = size;  % 设置y轴刻度值的字号
ax.XAxis.FontName = 'Times New Roman';
ax.YAxis.FontName = 'Times New Roman';
xticks(0:1:10)
yticks(-2:1:2)

subplot(4, 1, 4);
for i = 1:num_follower
    plot(uijmvz,hat_evy_history(:,i), 'LineWidth', 2, 'DisplayName', ['$$\dot{\xi}_' num2str(i) '$$']);
    hold on;
end
lgh = legend('FontName','Times New Roman','FontSize',size,'Location', 'best','Orientation','horizontal');
set(lgh,'interpreter','latex');
xlabel('t/(s)','FontName','Times New Roman','FontSize',size);
ylabel('y/(m/s)','FontName','Times New Roman','FontSize',size);
hold off;
ax = gca;  % 抓取当前坐标区或图，这个语句必须要有！！！！
ax.XAxis.FontSize = size;% 设置x轴刻度值的字号
ax.YAxis.FontSize = size;  % 设置y轴刻度值的字号
ax.XAxis.FontName = 'Times New Roman';
ax.YAxis.FontName = 'Times New Roman';
xticks(0:1:10)
yticks(-2:1:2)

fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '_error.png']);
%% leader运动时三维图
figure;
set(gcf,'Position',[50,50,800,600]); % 设置图形大小
set(gca, 'FontName', 'Times New Roman', 'FontSize',size);

% set(gcf,'Position',[100,100,800,600]); % 设置图形大小
uijmvz=linspace(0,t_sum+dmax*T,length(x_history));
hold on;
rxc=[2 4 4 2];
ryc=[2 2 4 4];
jpqu=iter+dmax;
% 动态leaders
% rxc=[2 3 3 2];
% ryc=[2 2 3 3];
plot3(uijmvz(1:jpqu), rx_history(1:jpqu,:),ry_history(1:jpqu,:),'r--','LineWidth',1)
follower_scatter = scatter3(jpqu/1000*ones(1,6),x_history(jpqu,:), y_history(jpqu,:), 'b', 'filled');
follower_scatter = scatter3(jpqu/1000*zeros(1,6),x_history(1,:), y_history(1,:), 'b', 'filled');
hull = convhull(rxc, ryc);
plot3([0,0,0,0,0], rxc(hull), ryc(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

hull = convhull(rx_history(jpqu,:), ry_history(jpqu,:));
rxt=rx_history(jpqu,:);
ryt=ry_history(jpqu,:);
leader_scatter = scatter3(jpqu/1000*ones(1,4), rxt, ryt, 'r', 'filled', 'DisplayName', 'Leaders');
hull_line = plot3(jpqu/1000*ones(1,5), rxt(hull), ryt(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

for i = 1:num_follower
    q(i) = plot3(uijmvz(1:jpqu), x_history(1:jpqu,i),y_history(1:jpqu,i),'-','LineWidth',1);
end
q(num_follower+1) = scatter3([0,0,0,0], rxc, ryc, 'r', 'filled', 'DisplayName', 'Leaders');

lgh = legend(q, "Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Leaders",'FontName','Times New Roman','FontSize',size,'Location', 'best');
set(lgh,'interpreter','latex');

xlabel('t/(s)','FontName','Times New Roman','FontSize',size);
ylabel('x/(m)','FontName','Times New Roman','FontSize',size);
zlabel('y/(m)','FontName','Times New Roman','FontSize',size)
view([-0.3,-1,0.5])

fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];

saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_k' num2str(k1) num2str(k2) num2str(k3) '_move3d.png']);