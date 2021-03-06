%% Workspace and singularity

clear
close all;
robot;

X = linspace(-50,200,250);
Y = linspace(-50,200,250);
% PHI = linspace(-pi/3,pi/2,6);
PHI = [-pi/3,0,pi/2];
% PHI = 0;

WS = zeros(length(X), length(Y), length(PHI));
%Singular = [];
XYZ = 3; % 2 - Y or 3 - Z
F = [0, 0, 0, 0, 0, 0]';
F(XYZ) = 100;

for i = 1:length(X)
    for j = 1:length(Y)
        Txy = Tx(X(i))*Ty(Y(j));
        for k = 1:length(PHI)
            T = Txy*Rz(PHI(k));
            q = IK(T,r3_robot);
            if ~isreal(q)
                WS(i,j,k)= NaN;
                continue;
            end
            %             q = [q(1:2),q(4:5),q(7:8)]';
            %             J = Jacobian3RRR(q, r3_robot);
            %             V = sqrt(det(J*J'));
            %if V > 0.1
            %    Singular = [Singular; X(i),Y(j),PHI(k)];
            %end
            %             WS(i,j,k) = V;
            
            %             stiffness analysis
            
            K1 = Kc_leg(q, 1, r3_robot);
            K2 = Kc_leg(q, 2, r3_robot);
            K3 = Kc_leg(q, 3, r3_robot);
            
            K_full =  Kc_full(K1,K2,K3,q,r3_robot);
            del = K_full \ F;
            WS(i,j,k) =  sqrt(sum(del(1:3).^2));
%             WS(i,j,k) =  find_max_del(K_full);
            
%             if (WS(i,j,k)>1)
%                 disp(X(i)); disp(Y(j)); disp(PHI(k));
%                 T=Tx(X(i))*Ty(Y(j))*Rz(PHI(k));
%                 q_9=IK(T,r3_robot);
%                 q6=[q_9(1:2)', q_9(4:5)', q_9(7:8)']';
%                 figure()
%                 visualisation(q_9,r3_robot);
%             end
            
            
            if (WS(i,j,k)>0.01)
                WS(i,j,k)= 0.01;
            end
            
            if (WS(i,j,k)<0.00001)
                WS(i,j,k)= NaN;
            end
            
            
            
        end
    end
end

% WS1 = log(WS);

for k = 1:length(PHI)
    h=figure();
    ws = WS(:,:,k);
    hold on
    surf(ws,'EdgeColor','none');
%     
%     robot = r3_robot;
%     plot3(robot.Joints(1).position(1)+50,robot.Joints(1).position(2)+50,0,'ok')
%     plot3(robot.Joints(4).position(1)+50,robot.Joints(4).position(2)+50,0,'ok')
%     plot3(robot.Joints(7).position(1)+50,robot.Joints(7).position(2)+50,0,'ok')
%     
%     T=Tx(70)*Ty(70)*Rz(PHI(k));
%     q=IK(T,robot);
%     
%     leg1(1,:) = robot.Joints(1).position(1:2)'+50;
%     leg2(1,:) = robot.Joints(4).position(1:2)'+50;
%     leg3(1,:) = robot.Joints(7).position(1:2)'+50;
%     % first link
%     leg1(2,:) = leg1(1,:) + robot.Links(1).length*[cos(q(1)),sin(q(1))];
%     leg2(2,:) = leg2(1,:) + robot.Links(4).length*[cos(q(4)),sin(q(4))];
%     leg3(2,:) = leg3(1,:) + robot.Links(7).length*[cos(q(7)),sin(q(7))];
%     % second link
%     leg1(3,:) = leg1(2,:) + robot.Links(2).length*[cos(q(1)+q(2)),sin(q(1)+q(2))];
%     leg2(3,:) = leg2(2,:) + robot.Links(5).length*[cos(q(4)+q(5)),sin(q(4)+q(5))];
%     leg3(3,:) = leg3(2,:) + robot.Links(8).length*[cos(q(7)+q(8)),sin(q(7)+q(8))];
%     % third "link"
%     leg1(4,:) = leg1(3,:) + robot.Links(3).length*[cos(q(1)+q(2)+q(3)), sin(q(1)+q(2)+q(3))];
%     leg2(4,:) = leg2(3,:) + robot.Links(6).length*[cos(q(4)+q(5)+q(6)), sin(q(4)+q(5)+q(6))];
%     leg3(4,:) = leg3(3,:) + robot.Links(9).length*[cos(q(7)+q(8)+q(9)), sin(q(7)+q(8)+q(9))];
%     % platform
%     platform = zeros(4,2);
%     platform(1,:) = leg1(3,:);
%     platform(2,:) = leg2(3,:);
%     platform(3,:) = leg3(3,:);
%     platform(4,:) = leg1(3,:);
%     
%     % show
%     plot(leg1(:,1),leg1(:,2),'r',leg2(:,1),leg2(:,2),'r',leg3(:,1),leg3(:,2),'r');
%     line(platform(:,1),platform(:,2),'LineWidth',2,'Color','b');
    
    xlim([0, 250])
    ylim([0, 250])
    
    title(['Phi=' num2str(PHI(k)*180/pi)])
end
