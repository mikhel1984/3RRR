%% Workspace and singularity

clear
close all;
robot;

X = linspace(-50,200,250);
Y = linspace(-50,200,250);
PHI = linspace(-pi/3,pi/2,6);

WS = zeros(length(X), length(Y), length(PHI));
%Singular = [];

for i = 1:length(X)
    for j = 1:length(Y)
        Txy = Tx(X(i))*Ty(Y(j));
        for k = 1:length(PHI)
            T = Txy*Rz(PHI(k));
            q = IK(T,r3_robot);
            if ~isreal(q)
                continue;
            end
            q = [q(1:2),q(4:5),q(7:8)]';
            J = Jacobian3RRR(q, r3_robot);
            V = sqrt(det(J*J')); 
            %if V > 0.1
            %    Singular = [Singular; X(i),Y(j),PHI(k)];               
            %end
            WS(i,j,k) = V;
        end
    end
end

WS1 = log(WS);

for k = 1:length(PHI)
    h=figure();
ws = WS1(:,:,k);
hold on
surf(ws,'EdgeColor','none');

robot = r3_robot;
plot3(robot.Joints(1).position(1)+50,robot.Joints(1).position(2)+50,0,'ok')
plot3(robot.Joints(4).position(1)+50,robot.Joints(4).position(2)+50,0,'ok')
plot3(robot.Joints(7).position(1)+50,robot.Joints(7).position(2)+50,0,'ok')

T=Tx(70)*Ty(70)*Rz(PHI(k));
q=IK(T,robot)

   leg1(1,:) = robot.Joints(1).position(1:2)'+50;
   leg2(1,:) = robot.Joints(4).position(1:2)'+50;
   leg3(1,:) = robot.Joints(7).position(1:2)'+50;   
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

xlim([0, 250])
ylim([0, 250])

title(['Phi=' num2str(PHI(k)*180/pi)])
end
