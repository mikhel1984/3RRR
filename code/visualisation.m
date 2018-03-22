function visualisation(q,robot)
% Robot visualization for given state
%   q - vector of angles
%   robot - parameters

   leg1 = zeros(4,2); leg2 = zeros(4,2); leg3 = zeros(4,2);
   % base
   leg1(1,:) = robot.Joints(1).position(1:2)';
   leg2(1,:) = robot.Joints(4).position(1:2)';
   leg3(1,:) = robot.Joints(7).position(1:2)';   
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
   xlabel('x'); ylabel('y');
   title('3RRR');
   % leg 1
   rectangle('Position',[leg1(1,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg1(2,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg1(3,:)-3,6,6],'Curvature',1);
   % leg 2
   rectangle('Position',[leg2(1,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg2(2,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg2(3,:)-3,6,6],'Curvature',1);
   % leg 3
   rectangle('Position',[leg3(1,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg3(2,:)-3,6,6],'Curvature',1);
   rectangle('Position',[leg3(3,:)-3,6,6],'Curvature',1);
   % base
   line([leg1(1,1)-10,leg1(1,1)+10],[leg1(1,2),leg1(1,2)],'Color','k','LineWidth',2);
   line([leg2(1,1)-10,leg2(1,1)+10],[leg2(1,2),leg2(1,2)],'Color','k','LineWidth',2);
   line([leg3(1,1)-10,leg3(1,1)+10],[leg3(1,2),leg3(1,2)],'Color','k','LineWidth',2);
   % platform
   line(platform(:,1),platform(:,2),'LineWidth',2,'Color','b');
   
   axis equal;
   
   
   
end