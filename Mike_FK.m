clear
close all;
robot
syms x y phi O1 O2 O3 a b l alph  pi  L

robot=r3_robot;
% L is a length of big triangle
% L=200;
% l=50;
% alph=pi/6;

x0=50;
y0=100;
phi0=pi/4;
T_Start=Tx(x0)*Ty(y0)*Rz(phi0);

q_Start=IK(T_Start,r3_robot);

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

F1=(x-B1(1)-a(1)*cos(O1)-L(1)*cos(alph(1)+phi))^2+(y-B1(2)-a(1)*sin(O1)-L(1)*sin(alph(1)+phi))^2-b(1)^2;
F2=(x-B2(1)-a(2)*cos(O2)-L(2)*cos(alph(2)+phi))^2+(y-B2(2)-a(2)*sin(O2)-L(2)*sin(alph(2)+phi))^2-b(2)^2;
F3=(x-B3(1)-a(3)*cos(O3)-L(3)*cos(alph(3)+phi))^2+(y-B3(2)-a(3)*sin(O3)-L(3)*sin(alph(3)+phi))^2-b(3)^2;


% while true
        
    J=[diff(F1,'x'), diff(F1,'y'), diff(F1,'phi');
        diff(F2,'x'), diff(F2,'y'), diff(F2,'phi');
        diff(F3,'x'), diff(F3,'y'), diff(F3,'phi')];
    
    J2=inv(J);
    F(x,y,phi,O1,O2,O3)=[F1;F2;F3];
    
    VecF=F(x0,y0,0, q_Start(1), q_Start(4), q_Start(7))
    double(VecF)
%     z=zold-J2*VecF;
    
%     if (J2*VecF<20); break; end
        
% end









