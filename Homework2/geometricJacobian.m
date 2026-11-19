function J = geometricJacobian(HT_0, jointType)
% HT_0{i} = H^w_{i-1}
% jointType = list w/ strings

n = length(jointType);

if isa(HT_0{end}(1,1), 'sym')
    J = sym(zeros(6,n));
else
    J = zeros(6,n);
end

o_n = HT_0{end}(1:3,4);   % point of interest

for i = 1:n
    z = HT_0{i}(1:3,3);   % z_{i-1} in world frame
    o = HT_0{i}(1:3,4);   % origin of frame i-1

    if jointType(i) == 'R'
        J(1:3,i) = cross(z, o_n - o);
        J(4:6,i) = z;
    else % Prismatic
        J(1:3,i) = z;
        J(4:6,i) = [0;0;0];
    end
end
end