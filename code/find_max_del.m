function max_del = find_max_del(K)
%  Find the maximun deflaction in Cartezian space
%  K - robot stiffness matrix

K_m = inv(K);

max_del = 0;
n = 5;
for x=-n:n
    for y=-n:n
        for z=0:n
            vec = [x y z];
            l = sqrt(sum(vec.^2));
            k = 100/l;
            F = [ vec*k, 0, 0, 0]';
            del = K_m * F;
            del_sum = sqrt(sum(del(1:3)).^2);
            if abs(del_sum)>abs(max_del)
                max_del = del_sum;
            end
        end
    end
end

end