% 设置输入AVI文件名和路径
input_video_file = '2023_22_12_21_10_02turtlebot_trajectories.gif.avi';

% 读取视频
vidObj = VideoReader(input_video_file);

% 设置GIF文件名和路径
output_gif_file = '2023_22_12_21_10_02turtlebot_trajectories.gif';

% 读取视频的每一帧并保存为GIF
for frame = 1:vidObj.NumFrames
    % 读取当前帧
    current_frame = read(vidObj, frame);
    
    % 将当前帧转换为灰度图像
    gray_frame = rgb2gray(current_frame);
    
    % 将灰度图像保存为GIF
    if frame == 1
        imwrite(gray_frame, output_gif_file, 'gif', 'Loopcount', inf, 'DelayTime', 1/vidObj.FrameRate);
    else
        imwrite(gray_frame, output_gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 1/vidObj.FrameRate);
    end
end
%%
% 创建一个时间序列
t = linspace(0, 2*pi, 100);

% 初始化一个图形窗口
figure;

% 循环绘制小球在坐标轴上的运动
for i = 1:length(t)
    % 计算小球的位置
    x = cos(t(i));
    y = sin(t(i));
    
    % 绘制小球
    plot(x, y, 'ro', 'MarkerSize', 10);
    axis([-1.5 1.5 -1.5 1.5]);
    title('Moving Ball Animation');
    drawnow; % 更新图形窗口
    
    % 将当前帧保存为图像
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i == 1
        imwrite(imind, cm, 'animation1.gif', 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, 'animation1.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end

disp('GIF 文件已保存成功！');

