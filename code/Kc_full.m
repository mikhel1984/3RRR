function [K] = Kc_full(k1, k2, k3, q, robot)
% Find Cartesian stiffness for full robot based on stiffness of each leg
%   q - total vector of angles
%   robot - system parameters
%   k1, k2, k3 - stiffness of each leg
% Return:
%   K - Cartesian stiffness matrix
    leg1 = zeros(4,2); leg2 = zeros(4,2); leg3 = zeros(4,2);
   % base
   leg1(1,:) = robot.Joints(1).position(1:2)';
   leg2(1,:) = robot.Joints(4).position(1:2)';
   leg3(1,:) = robot.Joints(7).position(1:2)';   
   % first link
   leg1(2,:) = leg1(1,:) + robot.Links(1).length*[cos(q(1)),sin(q(1))];
   leg2(2,:) = leg2(1,:) + robot.Links(4).length*[cos(q(4)),sin(q(4))];
   leg3(2,:) = leg3(1,:) + robot.Links(7).length*[cos(q(7)),sin(q(7))];
   % second link
   leg1(3,:) = leg1(2,:) + robot.Links(2).length*[cos(q(1)+q(2)),sin(q(1)+q(2))];
   leg2(3,:) = leg2(2,:) + robot.Links(5).length*[cos(q(4)+q(5)),sin(q(4)+q(5))];
   leg3(3,:) = leg3(2,:) + robot.Links(8).length*[cos(q(7)+q(8)),sin(q(7)+q(8))];
   % third "link"
   leg1(4,:) = leg1(3,:) + robot.Links(3).length*[cos(q(1)+q(2)+q(3)), sin(q(1)+q(2)+q(3))];
   leg2(4,:) = leg2(3,:) + robot.Links(6).length*[cos(q(4)+q(5)+q(6)), sin(q(4)+q(5)+q(6))];
   leg3(4,:) = leg3(3,:) + robot.Links(9).length*[cos(q(7)+q(8)+q(9)), sin(q(7)+q(8)+q(9))];
   
v1 = [leg1(4,:)-leg1(3,:) 0]/1000;
v2 = [leg2(4,:)-leg2(3,:) 0]/1000;
v3 = [leg3(4,:)-leg3(3,:) 0]/1000;

v1x = [0, -v1(3), v1(2);...
    v1(3), 0, -v1(1);...
    -v1(2), v1(1), 0];

v2x = [0, -v2(3), v2(2);...
    v2(3), 0, -v2(1);...
    -v2(2), v2(1), 0];

v3x = [0, -v3(3), v3(2);...
    v3(3), 0, -v3(1);...
    -v3(2), v3(1), 0];


J1 = [eye(3), v1x';
        zeros(3),eye(3)];
    
J2 = [eye(3), v2x';
        zeros(3),eye(3)];
    
J3 = [eye(3), v3x';
        zeros(3),eye(3)];

K = J1'*k1*J1+J2'*k2*J2+J3'*k3*J3;
end