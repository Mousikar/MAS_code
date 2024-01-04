% 静止
clear
%% 导入数据
load stationary.txt

x_history = stationary(:,1:5:30);
y_history = stationary(:,2:5:30);
rx_history = stationary(:,31:5:50);
ry_history = stationary(:,32:5:50);
theta_history = stationary(:,3:5:30);
v_history = stationary(:,4:5:30);
omega_history = stationary(:,5:5:30);

%% 轨迹图
figure;

hold on;
rx=rx_history(1,:);
ry=ry_history(1,:);
x=x_history(end,:);
y=y_history(end,:);
leader_scatter = scatter(rx, ry, 50, 'r', 'filled', 'DisplayName', 'Leaders');
plot(x_history,y_history,'LineWidth',1)
legend("Leaders","Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Location","best",'FontName','Times New Roman','FontSize',12)
follower_scatter = scatter(x, y, 50, 'b', 'filled','HandleVisibility','off');
hull = convhull(rx, ry);
hull_line = plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');
xlabel('x/(m)','FontName','Times New Roman','FontSize',12);
ylabel('y/(m)','FontName','Times New Roman','FontSize',12);
xlim([-5,5])
ylim([-3,3])
axis equal
set(gca,'color','none');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];