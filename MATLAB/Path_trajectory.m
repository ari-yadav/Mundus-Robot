%% Position, Velocity of each Joint
% Using LSPB method
clc
close all
clear all

%syms t 

%Assummed for joint 1 (q1)
ti = [0, 2, 4];  %seconds
tf = [2, 4, 6];   %seconds


% qi1 = [60, 60 , 35]; %degrees
% qf1 = [60, 35 , 60]; %degrees
qi1 = [60, 60, 35]; %degrees Adjusted for our diagram
qf1 = [60, 35 , 60]; %degrees

qi1_dot = [0, 0 , 0]; %degrees/sec
qf1_dot = [0, 0 , 0]; %degrees/sec




%% joint 2
qi2 = [-120, -70 , -70]; %degrees
qf2 = [-70, -70 , -120]; %degrees

qi2_dot = qi1_dot; %degrees/sec
qf2_dot = qi1_dot; %degrees/sec

%% joint 3
qi3 = [-30, -55 , -55]; %degrees
qf3 = [-55, -55, -30]; %degrees

qi3_dot = qi1_dot; %degrees/sec
qf3_dot = qi1_dot; %degrees/sec

Vconst = 60; %deg/sec

%initialize
tb1 = zeros(1,3);
tb2 = zeros(1,3);
tb3 = zeros(1,3);
a1 = zeros(1,3);
a2 = zeros(1,3);
a3 = zeros(1,3);

q1=zeros(1,3);
q2=zeros(1,3);
q3=zeros(1,3);
t1=zeros(1,3);
t2=zeros(1,3);
t3=zeros(1,3);



for i=1:length(qi1)
    %{
   tb1(i) = (qi1(i)-qf1(i)+Vconst*tf(i))./Vconst;
   a1(i) = Vconst./tb1(i);
   tb2(i) = (qi2(i)-qf2(i)+Vconst*tf(i))./Vconst;
   a2(i) = Vconst./tb2(i);
   tb3(i) = (qi3(i)-qf3(i)+Vconst*tf(i))./Vconst;
   a3(i) = Vconst./tb3(i);
   
   t1(1) = linspace(ti1(1),tb1(1),20);  
   t2(1) = linspace(ti2(1),tb2(1),20);
   t3(1) = linspace(ti3(1),tb3(1),20);
     
   
   %q1(1) = q1i(i+a1(i)/2*t.^2;
   
    t1(2) = linspace(tb1(2),(tf1(2)-tb1(i)),20);
    t2(2) = linspace(ti2(i),tb2(i),20);
    t3(2) = linspace(ti3(i),tb3(i),20);
     
    t1(3) = linspace(ti1(i),tb1(i),20);
    t2(3) = linspace(ti2(i),tb2(i),20);
    t3(3) = linspace(ti3(i),tb3(i),20);
   %}
end
%% Position, Velocity, ( Acceleration) of each End effector

ct = 1;

gamma = [-90.0000,  -85.0000,  -80.0000, -75.0000,  -70.0000,  -65.0000, -70.0000,  -75.0000, -80.0000,   -85.0000,  -90.0000,   -90.0000,  -90.0000,  -90.0000,  -90.0000,  -90.0000];
Endx = [2,     2.3292,    2.6189   ,    2.8615 ,    3.0504 ,    3.1809  ,    3.2500  ,    3.2944 ,    3.3137 ,    3.3077,    3.2766,    3.0642  ,    2.8284,    2.5712  ,    2.2943  ,    2.0000 ];
Endy = [ -0.5000,-.2981,-0.0459,.2491,.57820,0.9316,0.6508,0.3651,0.0766,-0.2125,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5];
theta1 = zeros(1,length(gamma));
theta2 = zeros(1,length(gamma));
theta3 = zeros(1,length(gamma));

for j=1:length(Endx)
    [theta1(j), theta2(j), theta3(j) ] = ikinematics(2,2,0.5, Endx(j), Endy(j), gamma(j));
    M(ct)= getframe(gcf);  %{collect different frames of figures} 
    ct =ct+1;              %{Increment Counter} 
    
end

pause(0.02);            %{to pause the program for n seconds}
M(ct)= getframe(gcf);  %{collect different frames of figures} 
ct =ct+1;              %{Increment Counter} 
movie(M,1,15) %{Making movie of collected frame} 
%{Assigning the videoname and type}
videofile = VideoWriter('Forward_Kinematics_run_69.avi','Uncompressed AVI');
%{to open the file}
open(videofile)   
%{assigning the different frames to videofile} 
writeVideo(videofile,M) 
close(videofile) %{To close the file}

time=linspace(0,6,16);

figure(2)
plot(time,Endx, 'linewidth',2, 'color',[0 0.4470 0.7410])
grid on
xlabel('Time [s]');
ylabel('Position [m]');
title('Position of End-efector');
hold on
plot(time,Endy, 'linewidth',2, 'color',[0.8500 0.3250 0.0980])
legend({'X-position','Y-position'},'Location','southeast');
hold off

figure(3)
plot(time,gamma, 'linewidth',2, 'color',[0.6350 0.0780 0.1840])
grid on
xlabel('Time [s]');
ylabel('Degrees[m]');
title('Orientation of the End-efector');

figure(4)
plot(time,theta1, 'linewidth',2, 'color',[0.8500 0.3250 0.0980])
grid on
xlabel('Time [s]');
ylabel('Joints'' Angles [deg]');
title('Angle of Every Joint during Robot''s Step');
hold on
plot(time,theta2, 'linewidth',2, 'color',[0.4940 0.1840 0.5560])
plot(time,theta3, 'linewidth',2, 'color',[0.3010 0.7450 0.9330])
legend({'\theta_1','\theta_2','\theta_3'},'Location','southeast');
hold off