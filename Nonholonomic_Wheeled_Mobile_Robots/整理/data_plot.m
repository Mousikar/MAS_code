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
currentDateTime = datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss');
clc
x_his = double(x_his);
y_his = double(y_his);
rx_his = double(rx_his);
ry_his = double(ry_his);
theta_his = double(theta_his);
v_his = double(v_his);
omega_his = double(omega_his);
errorx_F_his = double(errorx_F_his);
errory_F_his = double(errory_F_his);

figure;
set(gcf,'Position',[100,100,2000,1200]); % 设置图形大小

subplot(2, 3, 1);
grid on;
for k = 1:num_follower
    plot(x_his(:,k), 'LineWidth', 2, 'DisplayName', ['x' num2str(k)]);
    hold on;
    plot(y_his(:,k), 'LineWidth', 2, 'DisplayName', ['y' num2str(k)]);
end
legend('Location', 'southeast');
title('turtlebot position');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 3, 4);
grid on;
for k = 1:num_follower
    plot(theta_his(:,k), 'LineWidth', 2, 'DisplayName', ['theta' num2str(k)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot pose');
xlabel('t/(ms)');
ylabel('theta/(rad)');

subplot(2, 3, 2);
grid on;
for k = 1:num_follower
    plot(v_his(:,k), 'LineWidth', 2, 'DisplayName', ['v' num2str(k)]);
    hold on;
end
legend('Location', 'northeast');
title('turtlebot velocity');
xlabel('t/(ms)');
ylabel('v/(m/s)');

subplot(2, 3, 5);
grid on;
for k = 1:num_follower
    plot(omega_his(:,k), 'LineWidth', 2, 'DisplayName', ['omega' num2str(k)]);
    hold on;
end
legend('Location', 'northeast');
title('turtlebot angular');
xlabel('t/(ms)');
ylabel('omega/(rad/s)');

subplot(2, 3, 3);
grid on;
for i = 1:num_follower
    plot(errorx_F_his(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot containment x error');
xlabel('t/(ms)');
ylabel('X/(m)');

subplot(2, 3, 6);
grid on;
for i = 1:num_follower
    plot(errory_F_his(:,i), 'LineWidth', 2, 'DisplayName', ['follower' num2str(i)]);
    hold on;
end
legend('Location', 'southeast');
title('turtlebot containment y error');
xlabel('t/(ms)');
ylabel('Y/(m)');

hold off;

saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') '_two_dimensional_plane_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_F) '_k' num2str(k_L) '_ktheta' num2str(k_theta) '_F.png']);
%%
figure;
set(gcf,'Position',[100,100,1000,1000]); % 设置图形大小
ax = gca;
set(ax, 'XLim', [-1 25], 'YLim', [-5 25]);
grid on;
hold on;
leader_labels = cell(1, num_leader);
follower_labels = cell(1, num_follower);
for i = 1:num_leader
    leader_labels{i} = text(rx(i)+0.4, ry(i), ['Leader ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
end
for i = 1:num_follower
    follower_labels{i} = text(x_his(1,i)+0.4, y_his(1,i), ['Follower ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontSize', 8, 'FontWeight', 'bold');
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
            dx = 0.5 * cos(theta_his(fr, i));
            dy = 0.5 * sin(theta_his(fr, i));
            quiver(x_his(fr,i), y_his(fr,i), dx, dy, 'AutoScale', 'on', 'MaxHeadSize', 0.1, 'LineWidth', 1, 'Color', 'k', 'MaxHeadSize', 0.2);
            hold on;
            scatter(x_his(fr,i), y_his(fr,i), 20, 'b', 'filled', 'DisplayName', ['Follower ' num2str(i)], 'MarkerEdgeColor', 'k');
        end
        % hold off;
    end
end
plot(x_his,y_his)
saveas(gcf, [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') '_two_dimensional_plane_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_F) '_k' num2str(k_L) '_ktheta' num2str(k_theta) '.png']);

%% 制作动画
figure;clc
set(gcf,'Position',[100,100,1000,1000]); % 设置图形大小

vidObj = VideoWriter([datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') 'turtlebot_trajectories']);
open(vidObj);

slic = 200;
slice = floor(iter / slic);
follower_positions = zeros(num_follower, 2);
leader_positions = zeros(num_leader, 2);

ani = animatedline;

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
        set(ax, 'XLim', [-1 25], 'YLim', [-5 25]);
        for i = 1:num_follower
            dx = 0.5 * cos(theta_his(fr, i));
            dy = 0.5 * sin(theta_his(fr, i));
            quiver(x_his(fr,i), y_his(fr,i), dx, dy, 'AutoScale', 'on', 'MaxHeadSize', 0.1, 'LineWidth', 1, 'Color', 'k', 'MaxHeadSize', 0.2);
            hold on;
            scatter(x_his(fr,i), y_his(fr,i), 20, 'b', 'filled', 'DisplayName', ['Follower ' num2str(i)], 'MarkerEdgeColor', 'k');
        end
        plot(x_his, y_his, 'LineWidth', 2, 'DisplayName', 'Convex Hull');

        leader_labels = cell(1, num_leader);
        follower_labels = cell(1, num_follower);
        for i = 1:num_leader
            leader_labels{i} = text(rx_his(fr,i)+0.4, ry_his(fr,i), ['L ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
        end
        for i = 1:num_follower
            follower_labels{i} = text(x_his(fr,i)+0.4, y_his(fr,i), ['F ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontSize', 8, 'FontWeight', 'bold');
        end

        leader_scatter = scatter(rx_his(fr,:), ry_his(fr,:), 50, 'r', 'filled', 'DisplayName', 'Leaders');
        follower_scatter = scatter(x_his(fr,:), y_his(fr,:), 50, 'b', 'filled', 'DisplayName', 'Followers');
        rx_temp = rx_his(fr,:);ry_temp = ry_his(fr,:);
        hull = convhull(rx_temp, ry_temp);
        hull_line = plot(rx_temp(hull), ry_temp(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

        grid on;
        title(['two dimensional plane,T=' num2str(T) ',iter=' num2str(iter) ',delay:' num2str(tau_F) ',k=' num2str(k_L) ',k_{\theta}=' num2str(k_theta)]);
        xlabel('X/(m)');
        ylabel('Y/(m)');
        axis tight manual;

        hold off;
        frame = getframe(gcf);
        writeVideo(vidObj, frame);
    end
end
close(vidObj);
%% 存为GIF图
% 设置文件名和路径
clc
file_name = [datestr(currentDateTime, 'yyyy_mm_dd_HH_MM_ss') '_two_dimensional_plane_T' num2str(T) '_iter' num2str(iter) '_delay' num2str(tau_F) '_k' num2str(k_L) '_ktheta' num2str(k_theta) '.gif']; 
slic = 200;
slice = floor(iter / slic);
follower_positions = zeros(num_follower, 2);
leader_positions = zeros(num_leader, 2);
figure;
set(gcf,'Position',[100,100,1000,1000]); % 设置图形大小
grid on;
for fr = 1:iter
    if mod(fr, slice) == 0
        plot(x_his, y_his, 'LineWidth', 2, 'DisplayName', 'Convex Hull');
        hold on;
        for i = 1:num_follower
            dx = 0.5 * cos(theta_his(fr, i));
            dy = 0.5 * sin(theta_his(fr, i));
            quiver(x_his(fr,i), y_his(fr,i), dx, dy, 'AutoScale', 'on', 'MaxHeadSize', 0.1, 'LineWidth', 1, 'Color', 'k', 'MaxHeadSize', 0.2);
            scatter(x_his(fr,i), y_his(fr,i), 20, 'b', 'filled', 'DisplayName', ['Follower ' num2str(i)], 'MarkerEdgeColor', 'k');
        end
        % leader_labels = cell(1, num_leader);
        % follower_labels = cell(1, num_follower);
        % for i = 1:num_leader
        %     leader_labels{i} = text(rx_his(fr,i)+0.4, ry_his(fr,i), ['Leader ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
        % end
        % for i = 1:num_follower
        %     follower_labels{i} = text(x_his(fr,i)+0.4, y_his(fr,i), ['Follower ' num2str(i)], 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontSize', 8, 'FontWeight', 'bold');
        % end

        leader_scatter = scatter(rx_his(fr,:), ry_his(fr,:), 50, 'r', 'filled', 'DisplayName', 'Leaders');
        follower_scatter = scatter(x_his(fr,:), y_his(fr,:), 50, 'b', 'filled', 'DisplayName', 'Followers');
        rx_temp = rx_his(fr,:);ry_temp = ry_his(fr,:);
        hull = convhull(rx_temp, ry_temp);
        hull_line = plot(rx_temp(hull), ry_temp(hull), 'r', 'LineWidth', 2, 'DisplayName', 'Convex Hull');

        title(['two dimensional plane,T=' num2str(T) ',iter=' num2str(iter) ',delay:' num2str(tau_F) ',k=' num2str(k_L) ',k_{\theta}=' num2str(k_theta)]);
        xlabel('X/(m)');
        ylabel('Y/(m)');
        xlim([-5 25]);ylim([-5 25]);
        drawnow; % 更新图形窗口
        hold off;

        % 将当前帧保存为图像
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if fr == slice
            imwrite(imind, cm, file_name, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, file_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    end
end
disp('1')

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