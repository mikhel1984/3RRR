function J = J_theta(q,leg, robot)
% Jacobian for virtual joints
% Lengthes are in meters, angles in radians
%   q - total vector of angles
%   leg - leg number (1-3)
%   robot - system parameters
% Return:
%   J - Jacobian for virtual joints 6 x 13

   % map leg number to range 1-9
   n = 3*leg-2;
   
   % derivative matrices (for 0 value)
   dTx = zeros(4); dTx(1,4) = 1;
   dTy = zeros(4); dTy(2,4) = 1;
   dTz = zeros(4); dTz(3,4) = 1;
   dRx = zeros(4); dRx(3,2) = 1; dRx(2,3) = -1;
   dRy = zeros(4); dRy(3,1) = -1; dRy(1,3) = 1;
   dRz = zeros(4); dRz(2,1) = 1; dRz(1,2) = -1;
   
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
   
   J = zeros(6,13);
   % theta1 
   T = Tbase*Rz(q1)*dRz*Tx(l1)*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,1) = Jcol(T);
   % theta2
   T = Tbase*Rz(q1)*Tx(l1)*dTx*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,2) = Jcol(T);
   % theta 3
   T = Tbase*Rz(q1)*Tx(l1)*dTy*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,3) = Jcol(T);
   % theta 4
   T = Tbase*Rz(q1)*Tx(l1)*dTz*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,4) = Jcol(T);
   % theta 5
   T = Tbase*Rz(q1)*Tx(l1)*dRx*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,5) = Jcol(T);
   % theta 6
   T = Tbase*Rz(q1)*Tx(l1)*dRy*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,6) = Jcol(T);
   % theta 7
   T = Tbase*Rz(q1)*Tx(l1)*dRz*Rz(q2)*Tx(l2)*Rz(q3)*Ttool*Rinv;
   J(:,7) = Jcol(T);
   % theta 8
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dTx*Rz(q3)*Ttool*Rinv;
   J(:,8) = Jcol(T);
   % theta 9
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dTy*Rz(q3)*Ttool*Rinv;
   J(:,9) = Jcol(T);
   % theta 10
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dTz*Rz(q3)*Ttool*Rinv;
   J(:,10) = Jcol(T);
   % theta 11
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dRx*Rz(q3)*Ttool*Rinv;
   J(:,11) = Jcol(T);
   % theta 12
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dRy*Rz(q3)*Ttool*Rinv;
   J(:,12) = Jcol(T);
   % theta 13
   T = Tbase*Rz(q1)*Tx(l1)*Rz(q2)*Tx(l2)*dRz*Rz(q3)*Ttool*Rinv;
   J(:,13) = Jcol(T);
   
end