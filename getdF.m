function m = getdF(x,y,phi,q,robot)
   
   N = @(n) 3*n-2;
   dFdx = @(i) 2*(x - robot.Joints(N(i)).position(1) - robot.Links(N(i)).length*cos(q(i)) - robot.Links(3*i).length*cos(robot.Links(3*i).twist+phi-pi));
   dFdy = @(i) 2*(y - robot.Joints(N(i)).position(2) - robot.Links(N(i)).length*sin(q(i)) - robot.Links(3*i).length*sin(robot.Links(3*i).twist+phi-pi));
   dFdphi = @(i) 2*(x - robot.Joints(N(i)).position(1) - robot.Links(N(i)).length*cos(q(i)) - ...
                    robot.Links(3*i).length*cos(robot.Links(3*i).twist+phi-pi))*(robot.Links(3*i).length*sin(robot.Links(3*i).twist+phi-pi)) + ...
                 2*(y - robot.Joints(N(i)).position(2) - robot.Links(N(i)).length*sin(q(i)) - ...
                    robot.Links(3*i).length*sin(robot.Links(3*i).twist+phi-pi))*(-robot.Links(3*i).length*cos(robot.Links(3*i).twist+phi-pi));
                
   m = [dFdx(1) dFdy(1) dFdphi(1);
        dFdx(2) dFdy(2) dFdphi(2);
        dFdx(3) dFdy(3) dFdphi(3)];
end