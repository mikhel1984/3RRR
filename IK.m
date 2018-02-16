function q = IK(T,robot)
% Inverse kinematics

   % joint coordinates at the platform
   xA = robot.Links(3).length*cos(robot.Links(3).twist+robot.plate_angle);
   yA = robot.Links(3).length*sin(robot.Links(3).twist+robot.plate_angle);
   xB = robot.Links(6).length*cos(robot.Links(6).twist+robot.plate_angle);
   yB = robot.Links(6).length*sin(robot.Links(6).twist+robot.plate_angle);
   xC = robot.Links(9).length*cos(robot.Links(9).twist+robot.plate_angle);
   yC = robot.Links(9).length*sin(robot.Links(9).twist+robot.plate_angle);
   
   % rotation + translation
   XA = T*[xA;yA;0;1];
   XB = T*[xB;yB;0;1];
   XC = T*[xC;yC;0;1];
   % legs
   dA = XA(1:2) - robot.Joints(1).position(1:2);
   dB = XB(1:2) - robot.Joints(4).position(1:2);
   dC = XC(1:2) - robot.Joints(7).position(1:2);
   % angles
   q = zeros(9,1);
   [q(1),q(2)] = solve2link(robot.Links(1).length, robot.Links(2).length, dA(1), dA(2), 1);
   [q(4),q(5)] = solve2link(robot.Links(4).length, robot.Links(5).length, dB(1), dB(2), 1);
   [q(7),q(8)] = solve2link(robot.Links(7).length, robot.Links(8).length, dC(1), dC(2), -1);
   % last joints
   tA = T(:,3) - XA;
   tB = T(:,3) - XB;
   tC = T(:,3) - XC;
   q(3) = atan2(tA(2),tA(1)) - q(1) - q(2);
   q(6) = atan2(tB(2),tB(1)) - q(4) - q(5);
   q(9) = atan2(tC(2),tC(1)) - q(7) - q(8);
   
end