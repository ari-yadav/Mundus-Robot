%forward kinematics of 2R robotic arm with animation
%clearing the workspace, closing all and clearing the command window
% clear all
% close all
% clc

%Inputs 
%Length of the First Link (m)
l1 = 2;
%Length of the Second Link (m) 
l2 = 2;
%Length of the Third Link (m) 
l3 = 0.5;
%Fixed Position of the First link 
x0 = 0;  %{x axis coordinate}
y0 = 0;  %{y axis coordinate}

% Angle swept by the links 
% theta1 =linspace(45,45,19);  %{first link}
% theta2 =linspace(-90,-90,19);  %{Second link}
% theta3 =linspace(-45,-45,19);  %{Third link} 

theta1 =[60,60,60,60,60,60,60,60,55,50,45,40,35,40,45,50,55,60];  %{first link}
theta2 = [-120,-110,-100,-90,-80,-70,-70,-70,-70,-70,-70,-70,-70,-80,-90,-100,-110,-120];  %{Second link}
theta3 = [-30,-30,-30,-30,-30,-30,-40,-55,-55,-55,-55,-55,-55,-50,-45,-40,-35,-30];  %{Third link} 


% r=2;
% PP1x = zeros(17,1);
% PP1y = PP1x;
% PP2x = PP1x;
% PP2y = PP1x;
% PP3x = PP1x;
% PP3y = PP1x;


ct =1;  %{Counter number}
 plot([0 10], [0 0],'-k','linewidth',1);
 hold on
 k=0;
while k<=2
    
for i=1:length(theta3)
    plot([0 10], [0 0],'-k','linewidth',1);
    grid on
    set(gca,'xtick',[-1:0.5:5])
    hold on
%     rectangle('Position',[2-r,0.5-r,2*2,2*2],'Curvature',[1,1],...
%     'LineStyle',':')    %drawing a cricle for centre at (2,0.5)
    plot(PP1x, (PP1y+.5), '--g')
    plot(PP2x, (PP2y+.5), '--m')
    plot(PP3x, (PP3y+.5), '--c')
    THETA1 = theta1(i);        %{indexing each angle of first link} 
        THETA2 = theta2(i);       %{indexing each angle of Second link}
        THETA3 = theta3(i);     %{indexing each angle of Third link}
        x1 = x0+l1*cosd(THETA1);     %{Second link x-coordinate node1} 
        y1 = y0+l1*sind(THETA1);     %{Second link y-coordinate node1}  
        x2 = x0+x1+l2*cosd(THETA2+THETA1); %{Second link x-coordinate node2}  
        y2 = y0+y1+l2*sind(THETA2+THETA1); %{Second link y-coordinate node2}  
        x3 = x2 + l3*cosd(THETA3+THETA2+THETA1); %{Third link x-coordinate node2} 
        y3 = y2 + l3*sind(THETA3+THETA2+THETA1); %{Third link y-coordinate node2} 
        
        PP1x(i) = x1;
        PP1y(i) = y1;
        PP2x(i) = x2;
        PP2y(i) = y2;
        PP3x(i) = x3;
        PP3y(i) = y3;

%Plotting
plot([x0 x1],[0.5, y1+.5],'-r',[x1 x2],[y1+0.5 y2+.5],'-b', [x2 x3], [y2+.5 y3+0.5],'-og','linewidth',3);
hold off

axis([-0.1 5 -1 5]); %{Axis Range}
xlabel('Window Surface, x-coordinate [m]')
ylabel('Normal to Window Plane, y-coordinate [m]')
pause(0.02);            %{to pause the program for n seconds}
M(ct)= getframe(gcf);  %{collect different frames of figures} 
ct =ct+1;              %{Increment Counter} 

end
k=k+1;
end

%{    
for i=1:length(theta3)
    plot([0 10], [0 0],'-k','linewidth',1);
    hold on
    
%     plot(PP1x, (PP1y+.5), '--g')
%     plot(PP2x, (PP2y+.5), '--m')
%     plot(PP3x, (PP3y+.5), '.c')
    THETA1 = theta1(i);        %{indexing each angle of first link} 
        THETA2 = theta2(i);       %{indexing each angle of Second link}
        THETA3 = theta3(i);     %{indexing each angle of Third link}
        x1 = x0+l1*cosd(THETA1);     %{Second link x-coordinate node1} 
        y1 = y0+l1*sind(THETA1);     %{Second link y-coordinate node1}  
        x2 = x0+x1+l2*cosd(THETA2+THETA1); %{Second link x-coordinate node2}  
        y2 = y0+y1+l2*sind(THETA2+THETA1); %{Second link y-coordinate node2}  
        x3 = x2 + l3*cosd(THETA3+THETA2+THETA1); %{Third link x-coordinate node2} 
        y3 = y2 + l3*sind(THETA3+THETA2+THETA1); %{Third link y-coordinate node2} 
        
%         PP1x(i) = x1;
%         PP1y(i) = y1;
%         PP2x(i) = x2;
%         PP2y(i) = y2;
%         PP3x(i) = x3;
%         PP3y(i) = y3;

%Plotting
plot([x0 x1],[0, y1],[x1 x2],[y1 y2], [x2 x3], [y2 y3],'linewidth',3);
hold off

axis([-5 5 -5 5]); %{Axis Range}
xlabel('Window Surface, x-coordinate [m]')
ylabel('Normal to Window Plane, y-coordinate [m]')
pause(0.2);            %{to pause the program for n seconds}
M(ct)= getframe(gcf);  %{collect different frames of figures} 
ct =ct+1;              %{Increment Counter} 
hold on
end
%}

movie(M,1,15) %{Making movie of collected frame} 
%{Assigning the videoname and type}
videofile = VideoWriter('Forward_Kinematics_run_6.avi','Uncompressed AVI');
%{to open the file}
open(videofile)   
%{assigning the different frames to videofile} 
writeVideo(videofile,M) 
close(videofile) %{To close the file}
