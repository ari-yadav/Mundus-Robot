%forward kinematics of 2R robotic arm with animation
%clearing the workspace, closing all and clearing the command window
clear all
close all
clc

%Inputs 
%Length of the First Link (m)
l1 = 1;
%Length of the Second Link (m) 
l2 = 1;
%Length of the Third Link (m) 
l3 = 0.1;
%Fixed Position of the First link 
x0 = 0;  %{x axis coordinate}
y0 = 0.5;  %{y axis coordinate}
plot([-5 5],[0.5 0.5], 'linewidth',2)
hold on
plot([0 0],[-5 5], 'linewidth',2)
%Angle swept by the links 
%{
theta1 =linspace(75,45,10);  %{first link}
theta2 =linspace(-45,0,19);  %{Second link}
theta3 =linspace(-90,-120,5);  %{Third link} The more minus,the more backwards it goes
%}
theta1 =linspace(-30,-30,2);  %{first link}
theta2 =linspace(0,0,2);  %{Second link}
theta3 = linspace(-45,-45,2);  %{Third link} The more minus,the more backwards it goes
ct =1;  %{Counter number}
% For each fixed angle of link1 calculating workspace of link2
for i=1:length(theta1)
    THETA1 = theta1(i);        %{indexing each angle of first link}
    for j=1:length(theta2)     
        THETA2 = theta2(j);       %{indexing each angle of Second link}
        for k=1:length(theta3)
        THETA3 = theta3(k);
        x1 = x0+l1*cosd(THETA1);     %{Second link x-coordinate node1} 
        y1 = y0+l1*sind(THETA1);     %{Second link y-coordinate node1}  
        x2 = x0+x1+l2*cosd(THETA2+THETA1); %{Second link x-coordinate node2}  
        y2 = y0+y1+l2*sind(THETA2+THETA1); %{Second link y-coordinate node2}  
        x3 = x2 + l3*cosd(THETA3+THETA2+THETA1); %{Third link x-coordinate node2} 
        y3 = y2 + l3*sind(THETA3+THETA2+THETA1); %{Third link y-coordinate node2} 
%Plotting
plot([x0 x1],[0.5, y1+.5],'-r',[x1 x2],[y1+0.5 y2+.5],'-b', [x2 x3], [y2+.5 y3+0.5],'-og','linewidth',3);
axis([-4 4 -4 4]); %{Axis Range}
xlabel('x-coordinate [m]')
ylabel('y-coordinate [m]')
pause(0.001);            %{to pause the program for n seconds}
M(ct)= getframe(gcf);  %{collect different frames of figures} 
ct =ct+1;   %{Increment Counter} 


        end
    end
end

%{
movie(M) %{Making movie of collected frame} 
%{Assigning the videoname and type}
videofile = VideoWriter('Forward_Kinematics.avi','Uncompressed AVI');
%{to open the file}
open(videofile)   
%{assigning the different frames to videofile} 
writeVideo(videofile,M) 
close(videofile) %{To close the file}
%}