%% Stas FK

clear
close all;
robot;

% initial state
pos0 = (r3_robot.Joints(1).position + r3_robot.Joints(4).position + r3_robot.Joints(7).position)/3;
phi0 = 0;

z_old = [pos0(1);pos0(2);phi0];

T = Tx(z_old(1))*Ty(z_old(2))*Rz(z_old(3));
q = IK(T,r3_robot);

qnew = [q(1)+0.1;q(4)+0.1;q(7)-0.1];


% new state
x1 = pos0(1) + 10;
y1 = pos0(2) - 10;
phi1 = pi/6;

for i = 1:10    
    f = getF(z_old(1),z_old(2),z_old(3),qnew,r3_robot);
    jf = getdF(z_old(1),z_old(2),z_old(3),qnew,r3_robot);
    z_new = z_old - jf\f;
    z_old = z_new;
end

z_new

% check
Tnew = Tx(z_new(1))*Ty(z_new(2))*Rz(z_new(3));
qq = IK(Tnew, r3_robot);
[qq(1);qq(4);qq(7)]
qnew