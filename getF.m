function v = getF(x,y,phi,q,robot)

   N = @(n) 3*n-2;

   Fi = @(i) (x - robot.Joints(N(i)).position(1) - robot.Links(N(i)).length*cos(q(N(i))) - robot.Links(3*i).length*cos(robot.Links(3*i).twist+phi-pi))^2 + ...
             (y - robot.Joints(N(i)).position(2) - robot.Links(N(i)).length*sin(q(N(i))) - robot.Links(3*i).length*sin(robot.Links(3*i).twist+phi-pi))^2 - ...
             robot.Links(N(i)+1).length^2;
         
   v = [Fi(1);Fi(2);Fi(3)];   
end

