clc
clear
close all

robot

robot=r3_robot;

phi=0;

i=1;
hold on;
M=zeros(310,310);
for x=-100:1:200
    j=1;
    for y=-100:1:200
%         for phi=0:10:360
            
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
            
%             plot3(x,y,M(i,j));
            if (M(i,j)==0); M(i,j)=NaN; else; M(i,j)= log(M(i,j)); end
            j=j+1;
%         end
    end
    i=i+1;
end
hold on
surf(M,'EdgeColor','none')
 
plot3(robot.Joints(1).position(1)+100,robot.Joints(1).position(2)+100,0,'ok')
plot3(robot.Joints(4).position(1)+100,robot.Joints(4).position(2)+100,0,'ok')
plot3(robot.Joints(7).position(1)+100,robot.Joints(7).position(2)+100,0,'ok')

T=Tx(70)*Ty(70)*Rz(phi*pi/180);
q=IK(T,robot)

   leg1(1,:) = robot.Joints(1).position(1:2)'+100;
   leg2(1,:) = robot.Joints(4).position(1:2)'+100;
   leg3(1,:) = robot.Joints(7).position(1:2)'+100;   
   % first link
   leg1(2,:) = leg1(1,:) + robot.Links(1).length*[cos(q(1)),sin(q(1))];
   leg2(2,:) = leg2(1,:) + robot.Links(4).length*[cos(q(4)),sin(q(4))];
   leg3(2,:) = leg3(1,:) + robot.Links(7).length*[cos(q(7)),sin(q(7))];
   % second link
   leg1(3,:) = leg1(2,:) + robot.Links(2).length*[cos(q(1)+q(2)),sin(q(1)+q(2))];
   leg2(3,:) = leg2(2,:) + robot.Links(5).length*[cos(q(4)+q(5)),sin(q(4)+q(5))];
   leg3(3,:) = leg3(2,:) + robot.Links(8).length*[cos(q(7)+q(8)),sin(q(7)+q(8))];
   % third "link"
   leg1(4,:) = leg1(3,:) + robot.Links(3).length*[cos(q(1)+q(2)+q(3)), sin(q(1)+q(2)+q(3))];
   leg2(4,:) = leg2(3,:) + robot.Links(6).length*[cos(q(4)+q(5)+q(6)), sin(q(4)+q(5)+q(6))];
   leg3(4,:) = leg3(3,:) + robot.Links(9).length*[cos(q(7)+q(8)+q(9)), sin(q(7)+q(8)+q(9))];
   % platform
   platform = zeros(4,2);
   platform(1,:) = leg1(3,:);
   platform(2,:) = leg2(3,:);
   platform(3,:) = leg3(3,:);
   platform(4,:) = leg1(3,:);
   
   % show
   plot(leg1(:,1),leg1(:,2),'r',leg2(:,1),leg2(:,2),'r',leg3(:,1),leg3(:,2),'r');
   line(platform(:,1),platform(:,2),'LineWidth',2,'Color','b');

xlim([0, 300])
ylim([0, 300])
% visualisation(q,robot)
