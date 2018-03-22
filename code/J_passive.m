function J = J_passive(q,leg,robot)
% Jacobian for passive joints
% Lengthes are in meters, angles in radians
%   q - total vector of angles
%   leg - leg number (1-3)
%   robot - system parameters
% Return:
%   J - Jacobian for passive joints 6 x 2

   % map leg number to range 1-9
   n = 3*leg-2;
   
   % derivative matrices
   dRz = @(v) [-sin(v),-cos(v),0,0;cos(v),-sin(v),0,0;0,0,0,0;0,0,0,0];
   
   % extract column vector
   Jcol = @(m) [m(1,4);m(2,4);m(3,4);m(3,2);m(1,3);m(2,1)];
   
   % angles
   q1 = q(n); q2 = q(n+1); q3 = q(n+2);
   % length (in meters)
   l1 = robot.Links(n).length * 1E-3;
   l2 = robot.Links(n+1).length * 1E-3;
   l3 = robot.Links(n+2).length * 1E-3;
   
   % prepare common terms
   Tbase = eye(4); Tbase(1:3,4) = robot.Joints(n).position * 1E-3;
   Ttool = Tx(l3);
   Ttot = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*Rz(q3)*Ttool;
   Rinv = eye(4); Rinv(1:3,1:3) = Ttot(1:3,1:3)';
   
   J = zeros(6,2);
   % first passive joint
   T = Tbase*Rz(q1)*Tx(l1)*dRz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,1) = Jcol(T);
   % second passive joint
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dRz(q3)*Ttool*Rinv;
   J(:,2) = Jcol(T);
   
end