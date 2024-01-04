% 移动
clear
%% 导入数据
load env.txt

x_history = env(:,1:5:30);
y_history = env(:,2:5:30);
rx_history = env(:,31:5:50);
ry_history = env(:,32:5:50);
theta_history = env(:,3:5:30);
v_history = env(:,4:5:30);
omega_history = env(:,5:5:30);

%% 轨迹图
figure;
set(gcf,'Position',[100,100,1200,600]); % 设置图形大小
hold on;
% 多画几个过程框框
rx=rx_history(abs(rx_history(:,4)+7)<=0.001,:);
ry=ry_history(abs(rx_history(:,4)+7)<=0.001,:);
hull = convhull(rx, ry);
plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');

rx=rx_history(abs(rx_history(:,4)+4)<=0.001,:);
ry=ry_history(abs(rx_history(:,4)+4)<=0.001,:);
hull = convhull(rx, ry);
plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');

rx=rx_history(abs(rx_history(:,4)+1.5)<=0.001,:);rx=rx(1,:);
ry=ry_history(abs(rx_history(:,4)+1.5)<=0.001,:);ry=ry(1,:);
hull = convhull(rx, ry);
plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');

% rx=rx_history(abs(rx_history(:,4)+1)<=0.01,:);rx=rx(1,:);
% ry=ry_history(abs(rx_history(:,4)+1)<=0.01,:);ry=ry(1,:);
% hull = convhull(rx, ry);
% plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');

rx=rx_history(end,:);
ry=ry_history(end,:);
hull = convhull(rx, ry);
plot(rx(hull), ry(hull), 'r', 'LineWidth', 2,'HandleVisibility','off');

%--------------
x=x_history(end,:);
y=y_history(end,:);
% leader_scatter = scatter(rx, ry, 50, 'r', 'filled','HandleVisibility','off');
plot(x_history,y_history,'LineWidth',1)
plot(rx_history,ry_history,'--', 'LineWidth',1)
legend("Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Leader 1","Leader 2","Leader 3","Leader 4","Location","best",'Orientation','horizontal','NumColumns',6,'FontName','Times New Roman','FontSize',12)
follower_scatter = scatter(x, y, 50, 'b', 'filled','HandleVisibility','off');

xlabel('x/(m)','FontName','Times New Roman','FontSize',12);
ylabel('y/(m)','FontName','Times New Roman','FontSize',12);
xlim([-10,7])
ylim([-3,6])
axis equal
set(gca,'color','none');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
ylim([-4,5])

%%
find(abs(rx_history(:,4)+1.5)<=0.001)/15000*23