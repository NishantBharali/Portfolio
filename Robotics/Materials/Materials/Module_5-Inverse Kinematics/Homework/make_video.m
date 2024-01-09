close all
clear
clc

% create figure
figure
axis([-6, 6, -6, 6])
grid on
hold on

% save as a video file
v = VideoWriter('test.mp4', 'MPEG-4');
v.FrameRate = 10;
open(v);

% your code here


while % your condition here
    
    % plot the robot
    % 1. get the position of each link
    p0 = [0; 0];
    p1 = % (x,y) position of end of first link
    p2 = % (x,y) position of end of second link
    p3 = % (x,y) position of end of third link
    p4 = % (x,y) position of end of fourth link
    p = % (x,y) position of end-effector
    P = [p0, p1, p2, p3, p4, p];
    % 2. draw the robot and save the frame
    cla;
    plot(P(1,:), P(2,:), 'o-', 'color',[1, 0.5, 0],'linewidth',4)
    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    % your code here
    
end

close(v);
close all
