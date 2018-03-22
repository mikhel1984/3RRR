function J = Jacobian3RRR(q,robot)
% Jacobian for 3RRR
% q have 6 values: theta and psi

a(1)=robot.Links(1).length;
b(1)=robot.Links(2).length;

a(2)=robot.Links(4).length;
b(2)=robot.Links(5).length;

a(3)=robot.Links(7).length;
b(3)=robot.Links(8).length;

L(1)=robot.Links(3).length; 
alph(1)=robot.Links(3).twist;

L(2)=robot.Links(6).length; 
alph(2)=robot.Links(6).twist;

L(3)=robot.Links(9).length; 
alph(3)=robot.Links(9).twist;

beta(1)=q(1)+q(2);
beta(2)=q(3)+q(4);
beta(3)=q(5)+q(6);

Ax=a(1)*cos(q(1))+b(1)*cos(beta(1));
Ay=a(1)*sin(q(1))+b(1)*sin(beta(1));

Bx=a(2)*cos(q(2))+b(2)*cos(beta(2));
By=a(2)*sin(q(2))+b(2)*sin(beta(2));

phi=atan2(By-Ay,Bx-Ax);


Jz=[cos(beta(1)), sin(beta(1)), -L(1)*sin(beta(1)-(alph(1)+pi+phi));...
    cos(beta(2)), sin(beta(2)), -L(2)*sin(beta(2)-(alph(2)+pi+phi));...
    cos(beta(3)), sin(beta(3)), -L(1)*sin(beta(3)-(alph(3)+pi+phi))];

Jt=[a(1)*sin(q(2)), 0, 0;...
    0, a(2)*sin(q(4)), 0;...
    0, 0, a(3)*sin(q(6))];

J=inv(Jt)*Jz;
% J=inv(Jz)*Jt;
end