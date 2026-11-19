function [HTcm_0, HTcm_W] = getAbsoluteHTcm_abbIRB4(q,L, T0_W)
%GETABSOLUTEHTCM calculates the absolute Homogeneous transformations of
% the cms wrt the robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HTcm_0: List with all the absolute CM HT (4x4) wrt robot base (0)
%   HTcm_W: List with all the absolute CM HT (4x4) wrt wcf (w)

% Joint postion

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
%    1  2   3  4  5  6   7   8   9  10  11
% p=[L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];

L1=L(1);
L2=L(2);
L7=L(3);
al=L(5);
L11=L(6);
L21=L(7);
L32=L(9);
L41=L(10);
L51=L(11);

% Common substitutions

c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

s234=sin(q2 + q3 + q4);
c234=cos(q2 + q3 + q4);


% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)

Tcm1_0 = double([c1, -s1, sym(0), sym(0); s1, c1, sym(0), sym(0); sym(0), sym(0), sym(1), L11; sym(0), sym(0), sym(0), sym(1)]);
    
    
Tcm2_0 = double([c1*s2, c1*c2, -s1, L21*c1*s2; s1*s2, c2*s1, c1, L21*s1*s2; c2, -s2, sym(0), L21*c2 + sym(29/100); sym(0), sym(0), sym(0), sym(1)]);
    
    
Tcm3_0 = double([c1*c2*sin(al + q3) + c1*s2*cos(al + q3), c1*c2*cos(al + q3) - c1*s2*sin(al + q3), -s1, (c1*(27*s2 + 100*L51*c2*sin(al + q3) + 100*L51*s2*cos(al + q3)))/100; c2*s1*sin(al + q3) + s1*s2*cos(al + q3), c2*s1*cos(al + q3) - s1*s2*sin(al + q3), c1, (s1*(27*s2 + 100*L51*c2*sin(al + q3) + 100*L51*s2*cos(al + q3)))/100; c2*cos(al + q3) - s2*sin(al + q3), - c2*sin(al + q3) - s2*cos(al + q3), sym(0), (27*c2)/100 + L51*c2*cos(al + q3) - L51*s2*sin(al + q3) + sym(29/100); sym(0), sym(0), sym(0), sym(1)]);
    
    
Tcm4_0 = double([c1*ca23*cos(al - q4) + c1*sa23*sin(al - q4), c1*ca23*sin(al - q4) - c1*sa23*cos(al - q4), -s1, (27*c1*s2)/100 + (sym("2792289879327961")*c1*sa23)/sym("9007199254740992") + L41*c1*ca23*cos(al - q4) + L41*c1*sa23*sin(al - q4); ca23*s1*cos(al - q4) + s1*sa23*sin(al - q4), ca23*s1*sin(al - q4) - s1*sa23*cos(al - q4), c1, (27*s1*s2)/100 + (sym("2792289879327961")*s1*sa23)/sym("9007199254740992") + L41*ca23*s1*cos(al - q4) + L41*s1*sa23*sin(al - q4); ca23*sin(al - q4) - sa23*cos(al - q4), - ca23*cos(al - q4) - sa23*sin(al - q4), sym(0), (27*c2)/100 + (sym("2792289879327961")*ca23)/sym("9007199254740992") + L41*ca23*sin(al - q4) - L41*sa23*cos(al - q4) + sym(29/100); sym(0), sym(0), sym(0), sym(1)]);



%List with all the HT wrt to link 0, starting with H0_0
HTcm_0={eye(4),Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
HTcm_W = {eye(4),T0_W,T0_W*Tcm1_0,T0_W*Tcm2_0,T0_W*Tcm3_0,T0_W*Tcm4_0};


end

