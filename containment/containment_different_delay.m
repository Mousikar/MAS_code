% Define the number of leaders and followers
global num_followers
global L1
global L2
global leader_positions
num_leaders = 6;
num_followers = 6;
num_agents = num_leaders + num_followers;

% Define the communication topology for followers
follower_topology = containers.Map({0, 1, 2, 3, 4, 5}, {[1, 3], [0, 2], [1], [0, 4], [3, 5], [4]});

% Define the communication topology for leaders
leader_topology = containers.Map({0, 1, 2, 3, 4, 5}, {[0], [1], [2], [3], [4], [5]});

% Create the communication topology matrix
communication_topology = zeros(num_followers, num_followers);
for i = 1:num_followers
    neighbors = follower_topology(i - 1);
    for j = 1:length(neighbors)
        neighbor = neighbors(j);
        communication_topology(i, neighbor + 1) = 1;
    end
end

% Compute the Laplacian matrix
degree_matrix = diag(sum(communication_topology, 2));
laplacian_matrix = degree_matrix - communication_topology;

fprintf('Communication Topology Matrix:\n');
disp(communication_topology);

fprintf('\nLaplacian Matrix:\n');
disp(laplacian_matrix);

% Create A0
A0 = zeros(num_followers, num_followers);
for i = 1:num_followers
    leaders = leader_topology(i - 1);
    A0(i, i) = length(leaders);
end

fprintf('\nA0:\n');
disp(A0);

L1 = laplacian_matrix + A0;
fprintf('\nL1:\n');
disp(L1);

% Create L2
L2 = zeros(num_followers, num_leaders);
for i = 1:num_followers
    leaders = leader_topology(i - 1);
    for j = 1:length(leaders)
        leader = leaders(j);
        L2(i, leader + 1) = -1;
    end
end

fprintf('\nL2:\n');
disp(L2);

% Define the maximum movement distance per iteration for followers
max_movement_followers = 0.1;

% Define the number of iterations
num_iterations = 200;

% Initialize leader positions randomly in the range [0, 10)
leader_positions = 10 * rand(num_leaders, 2);
leader_positions = zeros(num_leaders, 2) + 4 * rand(num_leaders, 2);

% Initialize follower positions randomly in the range [0, 10)
follower_positions = 10 * rand(num_followers, 2);
follower_positions = 4 * ones(num_leaders, 2) + 6 * rand(num_leaders, 2);

t_sum = 40; % seconds
% dmax = [0.1, 0.02, 0.3, 0.02, 0.3, 0.1];
% dmax = [0.4, 0.02, 0.4, 0.02, 0.4, 0.1];
dmax = [0.1, 0.1, 0.3, 0.1, 0.3, 0.1];
dmax = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01];
t_span = [0, t_sum]; % Time span
t = linspace(0, t_sum, num_iterations); % Time points for plotting

% Initial condition for the differential equation
X = reshape(follower_positions', [1, num_followers * 2]);
initial_condition = X';

% Solve the differential equation for each time step
sol = dde23(@model,dmax,initial_condition,[0, t_sum]);
tint = linspace(0,5);
yy = deval(sol,t);

% Create arrays to store leader and follower positions over time
leader_positions_over_time = zeros(num_leaders, 2, num_iterations);
follower_positions_over_time = zeros(num_followers, 2, num_iterations);

for i = 1:num_iterations
    positions = reshape(yy(:, i), [num_followers, 2]);
    leader_positions_over_time(:, :, i) = leader_positions;
    follower_positions_over_time(:, :, i) = positions;
end

% Plot the leader and follower positions over time
figure;
for k = 1:num_followers
    x = squeeze(follower_positions_over_time(k, 1, :));
    y = squeeze(follower_positions_over_time(k, 2, :));
    plot(x, y, 'LineWidth', 2);
    hold on;
end
for k = 1:num_followers
    xend = squeeze(follower_positions_over_time(k, 1, end));
    yend = squeeze(follower_positions_over_time(k, 2, end));
    plot(xend, yend,'o', 'LineWidth', 2);
    hold on;
end
% 绘制凸包
points = leader_positions;
% 计算凸包的顶点
k = convhull(points(:,1), points(:,2));
% 添加第一个顶点以闭合多边形
k = [k; k(1)];
% 提取闭合多边形的顶点
polygon = points(k, :);
% 绘制闭合多边形
plot(polygon(:,1), polygon(:,2), 'r-', 'LineWidth', 2);
plot(leader_positions(:,1), leader_positions(:,2), 'ro', 'LineWidth', 2);

axis equal;
xlabel('X');
ylabel('Y');
title('Follower Trajectories');
legend('Follower 1', 'Follower 2', 'Follower 3', 'Follower 4', 'Follower 5', 'Follower 6', 'Leaders');
grid on;

% Define the model function for the differential equation
function dX_dt = model(t,x,Z)
    global num_followers
    global L1
    global L2
    global leader_positions
    XX = reshape(x, [2, num_followers])';
    cols = size(XX, 2);
    dX_dt = zeros(num_followers, cols);
    xlag1 = reshape(Z(:,1), [2, num_followers])';
    xlag2 = reshape(Z(:,2), [2, num_followers])';
    xlag3 = reshape(Z(:,3), [2, num_followers])';
    xlag4 = reshape(Z(:,4), [2, num_followers])';
    xlag5 = reshape(Z(:,5), [2, num_followers])';
    xlag6 = reshape(Z(:,6), [2, num_followers])';
    for i = 1:cols
        xnew=[xlag1(1, i);
              xlag2(2, i);
              xlag3(3, i);
              xlag4(4, i);
              xlag5(5, i);
              xlag6(6, i);];
        dX_dt(:, i) = -L1 * xnew - L2 * leader_positions(:, i);
    end

    dX_dt = reshape(dX_dt', [1, num_followers * 2])';
end
