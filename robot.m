% Advanced Robotics Manipulation
% 3RRR VSA 
% Stanislav Mikhel, Mikhail Ostanin, Innopolis 2018

%% Robot model description
%%% Link description
% link.type      - specify this part as a link with 'L' value
% link.length    - length of the link in meters
% link.axis      - orientation of the link in initial pose [X Y Z], so [1 0 0]
%                  corresponds to link with 10m length in X direction
% link.mass      - mass of the link (for dynamic calculation in future)
% link.inertia   - inertia matrix
% link.stiffness - stiffness matrix
% link.twist     - additional twist of the link

link1.type = 'L';
link1.length = 70; 
link1.axis = [1 0 0];
link1.twist = 0;

link2.type = 'L';
link2.length = 70;
link2.axis = [1 0 0];
link2.twist = 0;

link3.type = 'P';
link3.length = 30;
link3.axis = [1 0 0];
link3.twist = pi+pi/6;

link4.type = 'L';
link4.length = 70;
link4.axis = [1 0 0];
link4.twist = 0;

link5.type = 'L';
link5.length = 70;
link5.axis = [1 0 0];
link5.twist = 0;

link6.type = 'P';
link6.length = 30;
link6.axis = [1 0 0];
link6.twist = -pi/6;

link7.type = 'L';
link7.length = 70;
link7.axis = [1 0 0];
link7.twist = 0;

link8.type = 'L';
link8.length = 70;
link8.axis = [1 0 0];
link8.twist = 0;

link9.type = 'P';
link9.length = 30;
link9.axis = [1 0 0];
link9.twist = pi/2;

%%% Joint description
% joint.type     - specify this part as a joint with 'P' or 'R' value
%                  (revolute or prismatic joint)
% joint.axis     - axis of rotation in base frame [X Y Z], so [1 0 0]
%                  corresponds to joint with rotation about X axis
% joint.limit    - lower and upper limit for joint angle (distance)
% joint.child    - array with child links
% joint.parent   - array with parent links
% joint.position - coordinates of the joint

joint1.type = 'R';
joint1.axis = [0 0 1];
joint1.limit = [-pi pi];
joint1.parent = ['base'];
joint1.child = ['link1'];
joint1.position = [0;0;0];

joint2.type = 'R';
joint2.axis = [0 0 1];
joint2.limit = [-pi pi];
joint2.parent = ['link1'];
joint2.child = ['link2'];
joint2.position = [0;0;0];     % not used

joint3.type = 'R';
joint3.axis = [0 0 1];
joint3.limit = [-pi pi];
joint3.parent = ['link2'];
joint3.child = ['link3'];
joint3.position = [0;0;0];     % not used

joint4.type = 'R';
joint4.axis = [0 0 1];
joint4.limit = [-pi pi];
joint4.parent = ['base'];
joint4.child = ['link4'];
joint4.position = [130;0;0];

joint5.type = 'R';
joint5.axis = [0 0 1];
joint5.limit = [-pi pi];
joint5.parent = ['link4'];
joint5.child = ['link5'];
joint5.position = [130;0;0];    % not used

joint6.type = 'R';
joint6.axis = [0 0 1];
joint6.limit = [-pi pi];
joint6.parent = ['link5'];
joint6.child = ['link6'];
joint6.position = [130;0;0];    % not used

joint7.type = 'R';
joint7.axis = [0 0 1];
joint7.limit = [-pi pi];
joint7.parent = ['base'];
joint7.child = ['link7'];
joint7.position = [20;150;0];

joint8.type = 'R';
joint8.axis = [0 0 1];
joint8.limit = [-pi pi];
joint8.parent = ['link7'];
joint8.child = ['link8'];
joint8.position = [20;150;0];    % not used

joint9.type = 'R';
joint9.axis = [0 0 1];
joint9.limit = [-pi pi];
joint9.parent = ['link8'];
joint9.child = ['link9'];
joint9.position = [20;150;0];    % not used

%%% Robot assembly
% robot.Name - take a guess
% robot.Links - array with links
% robot.Joints - array with joints

r3_robot.Name = '3RRR VSA';
r3_robot.Links = [link1 link2 link3 link4 link5 link6 link7 link8 link9];
r3_robot.Joints = [joint1 joint2 joint3 joint4 joint5 joint6 joint7 joint8 joint9];