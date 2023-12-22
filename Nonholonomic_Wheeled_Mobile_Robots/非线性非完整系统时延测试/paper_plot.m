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

hold on;
leader_scatter = scatter(rx, ry, 50, 'r', 'filled', 'DisplayName', 'Leaders');
plot(x_history,y_history)
legend("Leaders","Follower 1","Follower 2","Follower 3","Follower 4","Follower 5","Follower 6","Location","best")
follower_scatter = scatter(x, y, 50, 'b', 'filled');
hull = convhull(rx, ry);
hull_line = plot(rx(hull), ry(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
xlabel('X/(m)');
ylabel('Y/(m)');
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_actual) '_k' num2str(k1) num2str(k2) num2str(k3) '.png']);

%%
figure;
set(gcf,'Position',[100,100,2000,1200]); % 设置图形大小

subplot(2, 5, 1);
grid on;
for k = 1:num_follower
    plot(x_history(:,k), 'LineWidth', 2, 'DisplayName', ['x' num2str(k)]);
    hold on;
    plot(y_history(:,k), 'LineWidth', 2, 'DisplayName', ['y' num2str(k)]);
end
legend('Location', 'southeast');
title('turtlebot position');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 6);
grid on;
for k = 1:num_follower
    plot(theta_history(:,k), 'LineWidth', 2, 'DisplayName', ['theta' num2str(k)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot pose');
xlabel('t/(ms)');
ylabel('theta/(rad)');

subplot(2, 5, 2);
grid on;
for k = 1:num_follower
    plot(v_history(:,k), 'LineWidth', 2, 'DisplayName', ['v' num2str(k)]);
    hold on;
end
legend('Location', 'northeast');
title('turtlebot velocity');
xlabel('t/(ms)');
ylabel('v/(m/s)');

subplot(2, 5, 7);
grid on;
for k = 1:num_follower
    plot(omega_history(:,k), 'LineWidth', 2, 'DisplayName', ['omega' num2str(k)]);
    hold on;
end
legend('Location', 'northeast');
title('turtlebot angular');
xlabel('t/(ms)');
ylabel('omega/(rad/s)');

subplot(2, 5, 3);
grid on;
for i = 1:num_follower
    plot(hat_evx_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot vx error');
xlabel('t/(ms)');
ylabel('X/(m/s)');

subplot(2, 5, 8);
grid on;
for i = 1:num_follower
    plot(hat_evy_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot vy error');
xlabel('t/(ms)');
ylabel('Y/(m/s)');
hold off;

subplot(2, 5, 4);
grid on;
for i = 1:num_follower
    plot(hat_ex_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot consensus x error');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 9);
grid on;
for i = 1:num_follower
    plot(hat_ey_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot consensus y error');
xlabel('t/(ms)');
ylabel('Y/(m)');

subplot(2, 5, 5);
grid on;
for i = 1:num_follower
    plot(errx_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot x error');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 10);
grid on;
for i = 1:num_follower
    plot(erry_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot y error');
xlabel('t/(ms)');
ylabel('Y/(m)');
hold off;

saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_actual) '_k' num2str(k1) num2str(k2) num2str(k3) '_error.png']);