function Xp=DifFK_kukaIIWA7(q,qp,L)
%DIFFK_KUKAIIWA7 calculates the end-effector's linear and angular velocity
%vector wrt the robot base (link 0)
% input:
%   q: is the joint position vector 1X7
%   qp: is the joint velocity vector 1X7
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Xp: End-effector linear and angular velocitites wrt robot base (6X1)


% Joint positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
q7=q(7);

% Joint positions
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);
qp5=qp(5);
qp6=qp(6);
qp7=qp(7);

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

c1=cos(q1);
c2=cos(q2);
c3=cos(q3);
c4=cos(q4);
c5=cos(q5);
c6=cos(q6);

s12=s1*s2;
s13=s1*s3;
s14=s1*s4;
s15=s1*s5;
s16=s1*s6;

s23=s2*s3;
s24=s2*s4;

s34=s3*s4;
s35=s3*s5;
s36=s3*s6;

s45=s4*s5;

s56=s5*s6;

c12=c1*c2;
c13=c1*c3;
c14=c1*c4;

c23=c2*c3;
c24=c2*c4;
c25=c2*c5;
c26=c2*c6;

c34=c3*c4;
c35=c3*c5;
c36=c3*c6;

c45=c4*c5;
c46=c4*c6;

c56=c5*c6;

c123=c12*c3;
c12s3=c12*s3;
c1s23=c1*s24;
c14s2=c14*s2;
c23s1=c23*s1;
c34s2=c34*s2;
c4s12=c4*s12;
c2s13=c2*s13;
s124=s12*s4;
c46s2=c46*s2;
c236s4=c23*c6*s4;
c3s24=c3*s24;
c5s246=c5*s24*s6;
c2345s6=c23*c45*s6;
c6s34=c6*s34;
c26s4=c26*s4;
c245s6=c24*c5*s6;
c5s23=c5*s23;
c2s45=c2*s45;
c24s6=c24*s6;
c256s4=c25*c6*s4;
c6s235=c6*s23*s5;
c3456s2=c34*c56*s2;
c136s4=c13*c6*s4;
c13s4=c13*s4;
c1s356=c1*s35*s6;
c26s134=c26*s13*s4;
c1345s6=c13*c45*s6;
c36s14=c36*s14;
s1356=s13*s56;
c245s136=c24*c5*s13*s6;
c345s16=c34*c5*s16;
c35s1=c35*s1;
c125s3=c12*c5*s3;
c4s135=c4*s13*s5;
c25s13=c25*s13;
c135=c13*c5;
c14s35=c14*s35;
c234s15=c23*c4*s15;
s135=s23*s5;

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn
Jef_0=[L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L2*s1*s2, c1*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)), c2*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L2*s1*s2) + s1*s2*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)), - (L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(c1*c3 - c2*s1*s3) - s2*s3*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))), (c2*c4 + c3*s2*s4)*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))) - (s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)), - L4*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L4*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)), sym(0); 
       L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + L2*c1*s2, s1*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)), c2*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + L2*c1*s2) - c1*s2*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)), - (L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(c3*s1 + c1*c2*s3) - s2*s3*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))), (c2*c4 + c3*s2*s4)*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))) - (L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), - L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - L4*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)), sym(0); 
       sym(0), -(c1^2 + s1^2)*(L2*s2 + L3*c4*s2 - L3*c2*c3*s4 + L4*c4*c6*s2 - L4*c2*c3*c6*s4 - L4*c2*s3*s5*s6 + L4*c5*s2*s4*s6 + L4*c2*c3*c4*c5*s6), -s2*(c1^2 + s1^2)*(L3*s3*s4 + L4*c6*s3*s4 - L4*c3*s5*s6 - L4*c4*c5*s3*s6), (c1^2 + s1^2)*(L3*c3*c4*s2 - L3*c2*s3^2*s4 - L3*c2*c3^2*s4 + L4*c3*c4*c6*s2 - L4*c2*c3^2*c6*s4 - L4*c2*c6*s3^2*s4 + L4*c2*c3^2*c4*c5*s6 + L4*c2*c4*c5*s3^2*s6 + L4*c3*c5*s2*s4*s6), L4*s6*(c1^2 + s1^2)*(- c2*s5*c3^2*s4 + s2*s5*c3*c4 + c5*s2*c4^2*s3 - c2*s5*s3^2*s4 + c5*s2*s3*s4^2), L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - L4*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))), sym(0); 
       sym(0), -s1, c1*s2, (c3*s1 + c1*c2*s3), (s4*(s1*s3 - c1*c2*c3) + c1*c4*s2), (s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)), (c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))); 
       sym(0), c1, s1*s2, c2*s1*s3 - c1*c3, c4*s1*s2 - s4*(c1*s3 + c2*c3*s1), c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4), s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2); 
       sym(1), sym(0), c2, -s2*s3, (c2*c4 + c3*s2*s4), c5*s2*s3 - s5*(c2*s4 - c3*c4*s2), (s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))];
    

%TODO: Calcualte the linear and angular velocity vector of the ef wrt 0
Xp=[qp5*((c2*c4 + c3*s2*s4)*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))) - (s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))) - qp6*(L4*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + L4*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))) + qp1*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L2*s1*s2) + qp3*(c2*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - L2*s1*s2) + s1*s2*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))) - qp4*((L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(c1*c3 - c2*s1*s3) + s2*s3*(L3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + L4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))))) + c1*qp2*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)); 
    qp1*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + L2*c1*s2) + qp3*(c2*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + L2*c1*s2) - c1*s2*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))) - qp4*((L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(c3*s1 + c1*c2*s3) + s2*s3*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))))) + qp5*((c2*c4 + c3*s2*s4)*(L3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))) - (L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4))*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - qp6*(L4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + L4*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))) + qp2*s1*(L2*c2 + L4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + L3*(c2*c4 + c3*s2*s4)); 
    qp4*(c1^2 + s1^2)*(L3*c3*c4*s2 - L3*c2*s3^2*s4 - L3*c2*c3^2*s4 + L4*c3*c4*c6*s2 - L4*c2*c3^2*c6*s4 - L4*c2*c6*s3^2*s4 + L4*c2*c3^2*c4*c5*s6 + L4*c2*c4*c5*s3^2*s6 + L4*c3*c5*s2*s4*s6) - qp2*(c1^2 + s1^2)*(L2*s2 + L3*c4*s2 - L3*c2*c3*s4 + L4*c4*c6*s2 - L4*c2*c3*c6*s4 - L4*c2*s3*s5*s6 + L4*c5*s2*s4*s6 + L4*c2*c3*c4*c5*s6) - qp3*s2*(c1^2 + s1^2)*(L3*s3*s4 + L4*c6*s3*s4 - L4*c3*s5*s6 - L4*c4*c5*s3*s6) - L4*qp6*(c1^2 + s1^2)*(c2*s6*c3^2*c4*c5^2 + c2*s6*c3^2*c4*s5^2 - c2*c6*c3^2*c5*s4 + c6*s2*c3*c4*c5 + s2*s6*c3*c5^2*s4 + s2*s6*c3*s4*s5^2 - c6*s2*c4^2*s3*s5 + c2*s6*c4*c5^2*s3^2 + c2*s6*c4*s3^2*s5^2 - c2*c6*c5*s3^2*s4 - c6*s2*s3*s4^2*s5) + L4*qp5*s6*(c1^2 + s1^2)*(- c2*s5*c3^2*s4 + s2*s5*c3*c4 + c5*s2*c4^2*s3 - c2*s5*s3^2*s4 + c5*s2*s3*s4^2); 
    qp5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - qp2*s1 + qp7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + qp6*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + qp4*(c3*s1 + c1*c2*s3) + c1*qp3*s2; 
    c1*qp2 - qp5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - qp7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - qp6*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - qp4*(c1*c3 - c2*s1*s3) + qp3*s1*s2; 
    qp1 + c2*qp3 - qp6*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) + qp7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + qp5*(c2*c4 + c3*s2*s4) - qp4*s2*s3];
