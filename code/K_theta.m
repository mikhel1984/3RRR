function K = K_theta(n,robot)
% Find joint stiffness matrix for one link
%   n - number of the link
%   robot - parameters of the system
% Return:
%   K - stiffness matrix 6 x 6

   % alluminium
   E = 7E10; nu = 0.346; 
   G = E / (2*(1+nu));
   
   % link size (convert to meters)
   L = robot.Links(n).length * 1E-3;
   dex = robot.Links(n).diametr * 1E-3;
   din = (robot.Links(n).diametr-robot.Links(n).thickness)*1E-3;
   
   % quill cylinder
   S = pi*(dex^2-din^2)/4;
   Iy = pi*(dex^4-din^4)/64; 
   Iz = Iy;
   J = Iy + Iz;
   
   K = (E/L^2) * [S*L,       0,       0,       0,      0,      0;
                    0, 12*Iz/L,       0,       0,      0,  -6*Iz;
                    0,       0, 12*Iy/L,       0,   6*Iy,      0;
                    0,       0,       0, G*J*L/E,      0,      0;
                    0,       0,    6*Iy,       0, 4*Iy*L,      0;
                    0,   -6*Iz,       0,       0,      0, 4*Iz*L];
   
   
end