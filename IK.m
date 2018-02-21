function q = IK(T,robot,conf)
% Inverse kinematics

   if nargin() == 2
       conf = [1 1 -1];
   end
   % joint coordinates at the platform
   xA = robot.Links(3).length*cos(robot.Links(3).twist);
   yA = robot.Links(3).length*sin(robot.Links(3).twist);
   xB = robot.Links(6).length*cos(robot.Links(6).twist);
   yB = robot.Links(6).length*sin(robot.Links(6).twist);
   xC = robot.Links(9).length*cos(robot.Links(9).twist);
   yC = robot.Links(9).length*sin(robot.Links(9).twist);
   
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
   [q(1),q(2)] = solve2link(robot.Links(1).length, robot.Links(2).length, dA(1), dA(2), conf(1));
   [q(4),q(5)] = solve2link(robot.Links(4).length, robot.Links(5).length, dB(1), dB(2), conf(2));
   [q(7),q(8)] = solve2link(robot.Links(7).length, robot.Links(8).length, dC(1), dC(2), conf(3));
   % last joints
   tA = T(:,4) - XA;
   tB = T(:,4) - XB;
   tC = T(:,4) - XC;
   q(3) = atan2(tA(2),tA(1)) - q(1) - q(2);
   q(6) = atan2(tB(2),tB(1)) - q(4) - q(5);
   q(9) = atan2(tC(2),tC(1)) - q(7) - q(8);
   
end