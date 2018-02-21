clc
clear
close all

robot

robot=r3_robot;
i=1;

M=zeros(310,310);
for x=-100:1:200
    j=1;
    for y=-100:1:200
%         for phi=0:10:360
            phi=pi/6;
            T=Tx(x)*Ty(y)*Rz(phi*pi/180);
            q=IK(T,robot);
            if (isreal(q))
                q2=[q(1:2), q(4:5), q(7:8)];
                J=Jacobian3RRR(q2,robot);
                M(i,j)=sqrt(det(J*J'));
                
%                 if (M(i,j)>0.1)
%                     disp(x); disp(y); disp(phi);
%                     T=Tx(x)*Ty(y)*Rz(phi*180/pi);
%                     q_9=IK(T,r3_robot);
%                     q6=[q_9(1:2)', q_9(4:5)', q_9(7:8)']';
%                     figure()
%                     visualisation(q_9,robot);
%                 end
                
            end
            if (M(i,j)==0); M(i,j)=NaN;end
            j=j+1;
%         end
    end
    i=i+1;
end
surf(M,'EdgeColor','none')
