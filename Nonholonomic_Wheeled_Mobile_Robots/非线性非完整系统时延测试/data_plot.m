% plot
% % 指定保存的文件名
% file_name = 'containment_l.txt';
% 
% % 将测试点保存到文本文件中
% fid = fopen(file_name, 'w');
% for i = 1:iter
%     fprintf(fid, '%.50f,%.50f\n', errx_actual_history(i,1), hat_ex_history(i,1));
% end
% fclose(fid);
%%
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
hat_evx_history = double(hat_ex_history);
hat_evy_history = double(hat_ey_history);

figure;
set(gcf,'Position',[100,100,2000,1200]); % 设置图形大小
subplot(2, 5, 1);
grid on;
for i = 1:num_follower
    plot(hat_evx_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot vx error');
xlabel('t/(ms)');
ylabel('X/(m/s)');

subplot(2, 5, 6);
grid on;
for i = 1:num_follower
    plot(hat_evy_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot vy error');
xlabel('t/(ms)');
ylabel('Y/(m/s)');
hold off;

subplot(2, 5, 2);
grid on;
for k = 1:num_follower
    plot(x_history(:,k), 'LineWidth', 2, 'DisplayName', ['x' num2str(k)]);
    hold on;
    plot(y_history(:,k), 'LineWidth', 2, 'DisplayName', ['y' num2str(k)]);
end
legend('show');
title('turtlebot position');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 7);
grid on;
for k = 1:num_follower
    plot(theta_history(:,k), 'LineWidth', 2, 'DisplayName', ['theta' num2str(k)]);
    hold on;
end
legend('show');
title('turtlebot pose');
xlabel('t/(ms)');
ylabel('theta/(rad)');

subplot(2, 5, 3);
grid on;
plot(v_history, 'LineWidth', 2);
title('turtlebot velocity');
xlabel('t/(ms)');
ylabel('v/(m/s)');

subplot(2, 5, 8);
grid on;
plot(omega_history, 'LineWidth', 2);
title('turtlebot angular');
xlabel('t/(ms)');
ylabel('omega/(rad/s)');

subplot(2, 5, 4);
grid on;
for i = 1:num_follower
    plot(hat_ex_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot consensus x error');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 9);
grid on;
for i = 1:num_follower
    plot(hat_ey_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot consensus y error');
xlabel('t/(ms)');
ylabel('Y/(m)');

subplot(2, 5, 5);
grid on;
for i = 1:num_follower
    plot(errx_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot x error');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 5, 10);
grid on;
for i = 1:num_follower
    plot(erry_actual_history(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('show');
title('turtlebot y error');
xlabel('t/(ms)');
ylabel('Y/(m)');
hold off;

saveas(gcf, ['turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_actual) '_k' num2str(k1) num2str(k2) num2str(k3) '_error.png']);
%%
figure;
ax = gca;
set(ax, 'XLim', [-1 11], 'YLim', [-1 11]);
grid on;
hold on;
leader_labels = cell(1, num_leader);
follower_labels = cell(1, num_follower);
for i = 1:num_leader
    leader_labels{i} = text(rx(i)+0.3, ry(i), ['Leader ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
end
for i = 1:num_follower
    follower_labels{i} = text(x_history(1,i), y_history(1,i), ['Follower ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontSize', 8, 'FontWeight', 'bold');
end
leader_scatter = scatter(rx, ry, 50, 'r', 'filled', 'DisplayName', 'Leaders');
follower_scatter = scatter(x, y, 50, 'b', 'filled', 'DisplayName', 'Followers');
hull = convhull(rx, ry);
hull_line = plot(rx(hull), ry(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
title('turtlebot position and pose');
xlabel('X/(m)');
ylabel('Y/(m)');
% legend('show');
% hold off;
slice = iter/20;

for fr = 1:iter
    if mod(fr, slice) == 0
        for i = 1:num_follower
            dx = 0.5 * cos(theta_history(fr, i));
            dy = 0.5 * sin(theta_history(fr, i));
            quiver(x_history(fr,i), y_history(fr,i), dx, dy, 'AutoScale', 'on', 'MaxHeadSize', 0.1, 'LineWidth', 1, 'Color', 'k', 'MaxHeadSize', 0.2);
            hold on;
            scatter(x_history(fr,i), y_history(fr,i), 20, 'b', 'filled', 'DisplayName', ['Follower ' num2str(i)], 'MarkerEdgeColor', 'k');
        end
        % hold off;
    end
end
plot(x_history,y_history)
saveas(gcf, ['turtlebot_trajectories_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_actual) '_k' num2str(k1) num2str(k2) num2str(k3) '.png']);

%% 制作动画
figure;clc
set(gcf,'Position',[100,100,1000,1000]); % 设置图形大小

vidObj = VideoWriter('turtlebot_trajectories.gif');
open(vidObj);

slic = 100;
slice = floor(iter / slic);
follower_positions = zeros(num_follower, 2);
leader_positions = zeros(num_leader, 2);

ani = animatedline;
% figure;
axis tight manual;
ax = gca;
ax.XLim = [-1 11];
ax.YLim = [-1 11];
grid on;
set(gca, 'nextplot', 'replacechildren');
set(gcf, 'Renderer', 'zbuffer');
frame = getframe(gcf);
writeVideo(vidObj, frame);

for fr = 1:iter
    if mod(fr, slice) == 0
        set(ax, 'XLim', [-1 11], 'YLim', [-1 11]);
        for i = 1:num_follower
            dx = 0.5 * cos(theta_history(fr, i));
            dy = 0.5 * sin(theta_history(fr, i));
            quiver(x_history(fr,i), y_history(fr,i), dx, dy, 'AutoScale', 'on', 'MaxHeadSize', 0.1, 'LineWidth', 1, 'Color', 'k', 'MaxHeadSize', 0.2);
            hold on;
            scatter(x_history(fr,i), y_history(fr,i), 20, 'b', 'filled', 'DisplayName', ['Follower ' num2str(i)], 'MarkerEdgeColor', 'k');
        end
        plot(x_history, y_history, 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

        leader_scatter = scatter(rx_history(fr,:), ry_history(fr,:), 50, 'r', 'filled', 'DisplayName', 'Leaders');
        follower_scatter = scatter(x_history(fr,:), y_history(fr,:), 50, 'b', 'filled', 'DisplayName', 'Followers');
        rx_temp = rx_history(fr,:);ry_temp = ry_history(fr,:);
        hull = convhull(rx_temp, ry_temp);
        hull_line = plot(rx_temp(hull), ry_temp(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

        grid on;
        title(['turtlebot trajectories T=' num2str(T) ', iter=' num2str(iter) ', delay=' num2str(tau_actual)]);
        xlabel('X/(m)');
        ylabel('Y/(m)');
        axis tight manual;

        hold off;
        frame = getframe(gcf);
        writeVideo(vidObj, frame);
    end
end
close(vidObj);


function update(frame)
    follower_positions(:,1) = x_history(frame*slice+dmax,:);
    follower_positions(:,2) = y_history(frame*slice+dmax,:);
    set(follower_scatter, 'XData', follower_positions(:,1), 'YData', follower_positions(:,2));
    
    leader_positions(:,1) = rx_history(frame*slice,:);
    leader_positions(:,2) = ry_history(frame*slice,:);
    set(leader_scatter, 'XData', leader_positions(:,1), 'YData', leader_positions(:,2));

    for i = 1:num_leader
        set(leader_labels{i}, 'Position', [leader_positions(i,1) + 0.8, leader_positions(i,2) + 0.1]);
    end
    for i = 1:num_follower
        set(follower_labels{i}, 'Position', [follower_positions(i,1) + 0.8, follower_positions(i,2) + 0.1]);
    end

    hull = convhull(leader_positions(:,1), leader_positions(:,2));
    set(hull_line, 'XData', leader_positions(hull,1), 'YData', leader_positions(hull,2));
end