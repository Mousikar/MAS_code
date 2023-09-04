clear

sol = dde23(@ddex1de,[1, 0.2],@ddex1hist,[0, 5]);
figure;
% plot(sol.x,sol.y)

tint = linspace(0,5);
yint = deval(sol,tint);
plot(tint,yint);

title('An example of Wille'' and Baker.');
xlabel('time t');
ylabel('solution y');

% --------------------------------------------------------------------------

%%
% 生成一些随机点作为示例输入
points = rand(10, 2);

% 计算凸包的顶点
k = convhull(points(:,1), points(:,2));

% 添加第一个顶点以闭合多边形
k = [k; k(1)];

% 提取闭合多边形的顶点
polygon = points(k, :);

% 绘制闭合多边形
figure;
plot(polygon(:,1), polygon(:,2), '-o', 'LineWidth', 2);
axis equal;
title('Convex Hull as a Closed Polygon');

%%


function s = ddex1hist(t)
% Constant history function for DDEX1.
s = ones(3,1);
end
% --------------------------------------------------------------------------

function dydt = ddex1de(t,y,Z)
% Differential equations function for DDEX1.
ylag1 = Z(:,1);
ylag2 = Z(:,2);
dydt = [ ylag1(1)
   ylag1(1) + ylag2(2)
   y(2)               ];
end