function [J1b,J2b,J3b] = ikinematics(links1, links2, links3, positionx, positiony, gamma)
%ikinematics(2,2,0.5, 2, -0.5, 0)
% Inverse kinematics for 3-Link Planar Manipulator
% Inputs:
% links1,links2,links3 are length of each links in the robotic arm
% base is the length of base of the robotic arm
% positionx,positiony are joint angles in reference to x-axis
% gamma is the orientation

format compact
format short

L12 = links1;
L23 = links2;
L34 = links3;
xe = positionx;
ye = positiony;
g = gamma;

%position P3
x3 = xe-(L34*cosd(g));
y3 = ye-(L34*sind(g));
C = sqrt(x3^2 + y3^2);

if (L12+L23) > C
    %angle a and B
    a = acosd((L12^2 + L23^2 - C^2 )/(2*L12*L23));
    B = acosd((L12^2 + C^2 - L23^2 )/(2*L12*C));

    %joint angles elbow-down
    J1a = atan2d(y3,x3)-B;
    J2a = 180-a;
    J3a = g - J1a -J2a;

    %joint angles elbow-up
    J1b = atan2d(y3,x3)+B;
    J2b = -(180-a);
    J3b = g - J1b - J2b;

   % fprintf('The joint 1, 2 and 3 angles are (%f,%f, %f) respectively for elbow-down configuration.\n',J1a,J2a,J3a)
    fprintf('The joint 1, 2 and 3 angles are (%f,%f, %f) respectively for elbow-up configuration.\n',J1b,J2b,J3b)
else 
    disp('     Dimension error!')
    disp('     End-effecter is outside the workspace.')
    return
end
x2a = L12*cosd(J1a);
y2a = L12*sind(J1a);
x2b = L12*cosd(J1b);
y2b = L12*sind(J1b);
r = L12 + L23 + L34;
daspect([1,1,1])
rectangle('Position',[-r,-r,2*r,2*r],'Curvature',[1,1],...
    'LineStyle',':')
line([0 x2a], [0 y2a],'Color','g','LineStyle','--')
line([x2a x3], [y2a y3],'Color','g','LineStyle','--')
line([x3 xe], [y3 ye],'Color','g','LineStyle','--')
line([0 x2b], [0 y2b],'Color','b')
line([x2b x3], [y2b y3],'Color','b')
line([x3 xe], [y3 ye],'Color','b')
%line([0 xe], [0 ye],'Color','r')
line([0 0], [-r/10 r/10], 'Color', 'r')
line([-r/10 r/10], [0 0], 'Color', 'r')
hold on
plot([0 x2a x3],[0 y2a y3],'o','Color','b')
plot([x2b],[y2b],'o','Color','g')
plot([xe],[ye],'o', 'Color', 'r')
grid on
xlabel('x-axis')
ylabel('y-axis')
title('Inverse Kinematics of the Robot Leg')
plot([-5 5],[-0.5 -0.5], 'linewidth',0.5, 'Color', 'k')

end