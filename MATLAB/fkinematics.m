function fkinematics(links1,links2,links3,theta1,theta2,theta3)
% Forward kinematics for 3-Link Planar Manipulator
% Inputs:
% links1,links2,links3 are length of each links in the robotic arm
% base is the length of base of the robotic arm
% theta1,theta2,theta3 are joint angles in reference to x-axis

format compact
format short

L12 = links1;
L23 = links2;
L34 = links3;
J1 = theta1;
J2 = theta2;
J3 = theta3;

%joint equation
x2 = L12*cosd(J1);
x3 = L23*cosd(J1+J2)+ x2;
xe = L34*cosd(J1+J2+J3) + x3;
y2 = L12*sind(J1);
y3 = L23*sind(J1+J2)+ y2;
ye = L34*sind(J1+J2+J3) + y3;
gamma = J1+J2+J3;
fprintf('The position of the end-effector is (%f, %f) and orientation is(%f)\n',xe,ye,gamma)

%plotting the links
r = L12 + L23 + L34;
daspect([1,1,1])
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',[1,1],...
    'LineStyle',':')
hold on
axis([-r r -r r])
line([0 x2], [0 y2])
line([x2 x3],[y2 y3])
line([x3 xe],[y3 ye])
line([0 0], [-r/10 r/10], 'Color', 'r')
line([-r/10 r/10], [0 0], 'Color', 'r')
hold on
plot([0 x2 x3],[0 y2 y3],'o')
plot([xe],[ye],'o','Color','r')
grid on
xlabel('x-axis')
ylabel('y-axis')
title('Forward Kinematics of the Robot Leg')
end