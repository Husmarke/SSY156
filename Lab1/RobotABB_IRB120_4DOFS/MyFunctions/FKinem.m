function T = FKinem(DH)
    %FKinem Computes forward kinematics based on DH parameters
    %   DH - [θ1, α1, r1, d1; θ2, α2, r2, d2;...]
    
    T = eye(4);
    for i=1:size(DH,1)
        t = DH(i,1);
        a = DH(i,2);
        r = DH(i,3);
        d = DH(i,4);

        A = [cos(t),    -sin(t)*cos(a), sin(t)*sin(a),  r*cos(t);
             sin(t),    cos(t)*cos(a),  -cos(t)*sin(a), r*sin(t);
             0,         sin(a),         cos(a),         d;
             0,         0,              0,              1];
        
        T = T*A;
    end
end