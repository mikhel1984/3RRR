function T = FK(q,robot,z)
% Forward kinematics
% q have 3 values

syms x y phi


B1=robot.Joints(1).position(1:2)';
B2=robot.Joints(4).position(1:2)';
B3=robot.Joints(7).position(1:2)';  

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

O1=q(1);
O2=q(2);
O3=q(3);

F1=(x-B1(1)-a(1)*cos(O1)-L(1)*cos(alph(1)+phi-pi))^2+(y-B1(2)-a(1)*sin(O1)-L(1)*sin(alph(1)+phi-pi))^2-b(1)^2;
F2=(x-B2(1)-a(2)*cos(O2)-L(2)*cos(alph(2)+phi-pi))^2+(y-B2(2)-a(2)*sin(O2)-L(2)*sin(alph(2)+phi-pi))^2-b(2)^2;
F3=(x-B3(1)-a(3)*cos(O3)-L(3)*cos(alph(3)+phi-pi))^2+(y-B3(2)-a(3)*sin(O3)-L(3)*sin(alph(3)+phi-pi))^2-b(3)^2;
        
J=[diff(F1,'x'), diff(F1,'y'), diff(F1,'phi');
    diff(F2,'x'), diff(F2,'y'), diff(F2,'phi');
    diff(F3,'x'), diff(F3,'y'), diff(F3,'phi')];

J2(x,y,phi)=inv(J);
F(x,y,phi)=[F1;F2;F3];

%  q=[0.4073, 1.3690, 0.7223];

if nargin < 3
    x0=(B1(1)+B2(1)+B3(1))/3;
    y0=(B1(2)+B2(2)+B3(2))/3;
    phi0=0;
    z=[x0, y0, phi0]';
end

% while true
for i=1:100
    
    VecF=double(F(z(1),z(2),z(3)));
    Jac2=double(J2(z(1),z(2),z(3)));
    z_old=z;
    del_z=Jac2*VecF;
    z=z_old-del_z;
    
    if (sum(del_z.^ 2)<0.1); break; end
        
end

T=Tx(z(1))*Ty(z(2))*Rz(z(3));
   
   
end