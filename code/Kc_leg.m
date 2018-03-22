function K = Kc_leg(q,leg,robot)
% Find Cartesian stiffness for one leg
%   q - total vector of angles
%   leg - number of leg (1-3)
%   robot - system parameters
% Return:
%   K - Cartesian stiffness matrix

   Jq = J_passive(q,leg,robot);
   Jtheta = J_theta(q,leg,robot);
   
   % map leg number to range 1-9
   n = 3*leg-2;
   
   % virtual joint stiffness matrix
   Kth = zeros(13);
   Kth(1,1) = robot.Joints(n).stiffness;
   Kth(2:7,2:7) = K_theta(n, robot);
   Kth(8:13,8:13) = K_theta(n+1, robot);
   
   Kc0 = inv((Jtheta/Kth)*Jtheta');
   Kcq = ((Jq'*Kc0*Jq)\Jq')*Kc0;
   K = Kc0 - Kc0*Jq*Kcq;
   %M = inv([zeros(6),Jtheta,Jq; Jtheta', -Kth,zeros(13,2); Jq',zeros(2,13),zeros(2,2)]);
   %K = M(1:6,1:6);
end