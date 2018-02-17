clear
close all;
robot;

robot=r3_robot;

x0=50;
y0=100;
phi0=pi/3;
T_Start=Tx(x0)*Ty(y0)*Rz(phi0);

q_9=IK(T_Start,r3_robot);
dt=0.1;
q6=[q_9(1:2), q_9(4:5), q_9(7:8)]';
q3=[q_9(1), q_9(4), q_9(7)]';
dz=[0 10 0]';
z=[x0, y0, phi0];
for i=1:10
    
    dq=Jacobian3RRR(q6,robot)*dz;
   
    q3=q3+dq*dt;
    
    T=FK(q3,r3_robot);
    T(1:3,4)
    q_9=IK(T,r3_robot);
    q6=[q_9(1:2), q_9(4:5), q_9(7:8)]';
    
%     hold on
%     visualisation(q_9,robot);
%     pause(1);
end

% theta=[q(1), q(3), q(5)];
% FK(theta,robot)






