%% Workspace and singularity

clear
close all;
robot;

X = linspace(-50,200,200);
Y = linspace(-50,200,200);
PHI = linspace(0,2*pi,50);

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
ws = WS1(:,:,1);
surf(ws);