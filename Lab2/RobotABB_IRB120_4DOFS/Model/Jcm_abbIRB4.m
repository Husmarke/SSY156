function [Jcm1,Jcm2,Jcm3,Jcm4] = Jcm_abbIRB4(q,L)
%JCM CM geometric Jacobians
%   q: Joint position vector 4x1
%   L: Dynamic parameters, see abbIRB4_dyn_params.m

% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic and dynamic parameters
L2=L(2);
L7=L(3);
al=L(5);
L21=L(7);
L41=L(10);
L51=L(11);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

% Jacobians CM
%TODO: define the geometric Jacobians for each CM
Jcm1=[sym(0), sym(0), sym(0), sym(0); sym(0), sym(0), sym(0), sym(0); sym(0), sym(0), sym(0), sym(0); sym(0), sym(0), sym(0), sym(0); sym(0), sym(0), sym(0), sym(0); sym(1), sym(0), sym(0), sym(0)];
				    
Jcm2=[-L21*s1*s2, L21*c1*c2, sym(0), sym(0); L21*c1*s2, L21*c2*s1, sym(0), sym(0); sym(0), -L21*s2*(c1^2 + s1^2), sym(0), sym(0); sym(0), -s1, sym(0), sym(0); sym(0), c1, sym(0), sym(0); sym(1), sym(0), sym(0), sym(0)];
	    
Jcm3=[-sin(q1)*(L2*sin(q2) + L51*sin(al + q2 + q3)), cos(q1)*(L2*cos(q2) + L51*cos(al + q2 + q3)), L51*cos(al + q2 + q3)*cos(q1), sym(0); cos(q1)*(L2*sin(q2) + L51*sin(al + q2 + q3)), sin(q1)*(L2*cos(q2) + L51*cos(al + q2 + q3)), L51*cos(al + q2 + q3)*sin(q1), sym(0); sym(0), - L2*sin(q2) - L51*sin(al + q2 + q3), -L51*sin(al + q2 + q3), sym(0); sym(0), -sin(q1), -sin(q1), sym(0); sym(0), cos(q1), cos(q1), sym(0); sym(1), sym(0), sym(0), sym(0)];
    
Jcm4=[-sin(q1)*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)), cos(q1)*(L2*cos(q2) + L7*cos(al + q2 + q3) - L41*sin(q2 + q3 + q4)), cos(q1)*(L7*cos(al + q2 + q3) - L41*sin(q2 + q3 + q4)), -L41*sin(q2 + q3 + q4)*cos(q1); cos(q1)*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)), sin(q1)*(L2*cos(q2) + L7*cos(al + q2 + q3) - L41*sin(q2 + q3 + q4)), sin(q1)*(L7*cos(al + q2 + q3) - L41*sin(q2 + q3 + q4)), -L41*sin(q2 + q3 + q4)*sin(q1); sym(0), - L2*sin(q2) - L41*cos(q2 + q3 + q4) - L7*sin(al + q2 + q3), - L41*cos(q2 + q3 + q4) - L7*sin(al + q2 + q3), -L41*cos(q2 + q3 + q4); sym(0), -sin(q1), -sin(q1), -sin(q1); sym(0), cos(q1), cos(q1), cos(q1); sym(1), sym(0), sym(0), sym(0)];


end

