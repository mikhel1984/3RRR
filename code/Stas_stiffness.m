% load
robot;

% initial state
pos0 = (r3_robot.Joints(1).position + r3_robot.Joints(4).position + r3_robot.Joints(7).position)/3;
phi0 = 0;
z_old = [pos0(1);pos0(2);phi0];
T = Tx(z_old(1))*Ty(z_old(2))*Rz(z_old(3));
q = IK(T,r3_robot);

% assume displacement
disp = [1E-3;1E-3;1E-3;0;0;0]; % distance in meters!

% leg 1
K1 = Kc_leg(q, 1, r3_robot);
F1 = K1*disp;
del1 = K1 \ F1 - disp

% leg 2
K2 = Kc_leg(q, 2, r3_robot);
F2 = K2*disp;
del2 = K2 \ F2 - disp

% leg 3
K3 = Kc_leg(q, 3, r3_robot);
F3 = K3*disp;
del3 = K3 \ F3 - disp

K_full =  Kc_full(K1,K2,K3,q,r3_robot)

