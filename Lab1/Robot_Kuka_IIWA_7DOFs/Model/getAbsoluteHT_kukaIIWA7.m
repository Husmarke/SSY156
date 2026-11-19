function [HT_0, HT_W] = getAbsoluteHT_kukaIIWA7(q,L, T0_W)
%GETABSOLUTEHT_KUKAIIWA7 calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X7
%   L: is the kinematic parameter array (see abbIRB4_params.m)
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: Cell array with all the absolute HT (4x4) wrt robot base
%   HT_W: Cell array with all the absolute HT (4x4) wrt wcf

% Joint positions

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
q7=q(7);

% Kinematic parameters
L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);

% Common Substitutions
s1=sin(q1);
s2=sin(q2);
s3=sin(q3);
s4=sin(q4);
s5=sin(q5);
s6=sin(q6);
s7=sin(q7);

c1=cos(q1);
c2=cos(q2);
c3=cos(q3);
c4=cos(q4);
c5=cos(q5);
c6=cos(q6);
c7=cos(q7);

s12=s1*s2;
s13=s1*s3;

s23=s2*s3;
s24=s2*s4;

c12=c1*c2;
c13=c1*c3;
c14=c1*c4;

c23=c2*c3;
c24=c2*c4;

c34=c3*c4;


c123=c12*c3;
c12s3=c12*s3;
c1s23=c1*s24;
c14s2=c14*s2;
c23s1=c23*s1;
c34s2=c34*s2;
c4s12=c4*s12;
c2s13=c2*s13;
s124=s12*s4;
c3s24=c3*s24;
c5s23=c5*s23;
s135=s23*s5;

% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)
T1_0=double([c1, sym(0), -s1, sym(0); s1, sym(0), c1, sym(0); sym(0), -sym(1), sym(0), L1; sym(0), sym(0), sym(0), sym(1)]);
    
T2_0=double([c1*c2, -s1, c1*s2, sym(0); c2*s1, c1, s1*s2, sym(0); -s2, sym(0), c2, L1; sym(0), sym(0), sym(0), sym(1)]);
   
T3_0=double([c1*c2*c3 - s1*s3, c1*s2, c3*s1 + c1*c2*s3, L2*c1*s2; c1*s3 + c2*c3*s1, s1*s2, c2*s1*s3 - c1*c3, L2*s1*s2; -c3*s2, c2, -s2*s3, L1 + L2*c2; sym(0), sym(0), sym(0), sym(1)]);
    
T4_0=double([c1*s2*s4 - c4*(s1*s3 - c1*c2*c3), - c3*s1 - c1*c2*s3, s4*(s1*s3 - c1*c2*c3) + c1*c4*s2, L2*c1*s2; c4*(c1*s3 + c2*c3*s1) + s1*s2*s4, c1*c3 - c2*s1*s3, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), L2*s1*s2; c2*s4 - c3*c4*s2, s2*s3, c2*c4 + c3*s2*s4, L1 + L2*c2; sym(0), sym(0), sym(0), sym(1)]);
    
T5_0=double([- c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3), - s4*(s1*s3 - c1*c2*c3) - c1*c4*s2, s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L2*c1*s2; c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3), s4*(c1*s3 + c2*c3*s1) - c4*s1*s2, c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), L2*s1*s2 - L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2); c5*(c2*s4 - c3*c4*s2) + s2*s3*s5, - c2*c4 - c3*s2*s4, c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), L1 + L2*c2 + L3*(c2*c4 + c3*s2*s4); sym(0), sym(0), sym(0), sym(1)]);
   
T6_0=double([- s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3), c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L2*c1*s2; s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), L2*s1*s2 - L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2); c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4), L1 + L2*c2 + L3*(c2*c4 + c3*s2*s4); sym(0), sym(0), sym(0), sym(1)]);
    
T7_0=double([s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))), s7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)), c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)), L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + L2*c1*s2; c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)), - c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))), s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2), L2*s1*s2 - L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2); c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) - s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3), - c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)), s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4), L1 + L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4); sym(0), sym(0), sym(0), sym(1)]);
    


% List with all the HT wrt to link 0, starting with H0_0
HT_0={eye(4),T1_0, T2_0, T3_0, T4_0,T5_0, T6_0, T7_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 9 elements Hw_w, H0_w, ..., Hef_w.
HT_W={eye(4),T0_W,T0_W*T1_0,T0_W*T2_0,T0_W*T3_0,T0_W*T4_0,T0_W*T5_0,T0_W*T6_0,T0_W*T7_0};

end