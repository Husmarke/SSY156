function Qpp=Dynamic_abbIRB4v2(u)
% Dynamic_abbIRB4 Calculates the direct dynamics of robot ABB IRB 120 (4DOF)
%
% This model uses external parameters


%Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);

%Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);

%Gravity Vector
gz=u(9);

%Viscous Friction Matrix
Beta(1,1)=u(10);
Beta(2,2)=u(11);
Beta(3,3)=u(12);
Beta(4,4)=u(13);

%Time
t=u(14);


%Joint Position Vector
Q=[q1; q2; q3; q4];

%Joint Velocity Vector
Qp=[qp1; qp2; qp3; qp4];

% Load Robot Parameters
abb_irb120_params

% Inertia Tensors
I133=I1(6);

I211=I2(1);
I212=I2(2);
I213=I2(3);
I222=I2(4);
I223=I2(5);
I233=I2(6);

I311=I3(1);
I312=I3(2);
I313=I3(3);
I322=I3(4);
I323=I3(5);
I333=I3(6);

I411=I4(1);
I412=I4(2);
I413=I4(3);
I422=I4(4);
I423=I4(5);
I433=I4(6);

%Common substitutions
c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca3=cos(al + q3);
sa3=sin(al + q3);

c2_234=cos(2*q2 + 2*q3 + 2*q4);
s2_234=sin(2*q2 + 2*q3 + 2*q4);

c34=cos(q3 + q4);
s34=sin(q3 + q4);

c2_a23=cos(2*al + 2*q2 + 2*q3);
s2_a23=sin(2*al + 2*q2 + 2*q3);

cam4=cos(al - q4);
sam4=sin(al - q4);

c22=cos(2*q2);
s22=sin(2*q2);

ca_223_4=cos(al + 2*q2 + 2*q3 + q4);

c22_34=cos(2*q2 + q3 + q4);


%% Inertia Matrix
%TODO: define the inertia Matrix 
M(1,1) = I133 + I322*sin(al + q2 + q3)^2 + cos(al + q2 + q3)*(I311*cos(al + q2 + q3) - I312*sin(al + q2 + q3)) + cos(q2 + q3 + q4)*(I422*cos(q2 + q3 + q4) + I412*sin(q2 + q3 + q4)) + sin(q2 + q3 + q4)*(I412*cos(q2 + q3 + q4) + I411*sin(q2 + q3 + q4)) + cos(q2)*(I211*cos(q2) - I212*sin(q2)) + m4*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3))^2 + m3*(L2*sin(q2) + L51*sin(al + q2 + q3))^2 - (I312*sin(2*al + 2*q2 + 2*q3))/2 - (I212*sin(2*q2))/2 + I222*sin(q2)^2 + L21^2*m2*sin(q2)^2;
M(1,2) = I213*cos(q2) - I223*sin(q2) + I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(1,3) = I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(1,4) = - I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4);
M(2,1) = I213*cos(q2) - I223*sin(q2) + I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(2,2) = I233 + I333 + I433 + L2^2*m3 + L2^2*m4 + L7^2*m4 + L21^2*m2 + L41^2*m4 + L51^2*m3 - 2*L2*L41*m4*sin(q3 + q4) + 2*L7*L41*m4*sin(al - q4) + 2*L2*L7*m4*cos(al + q3) + 2*L2*L51*m3*cos(al + q3);
M(2,3) = I333 + I433 + L7^2*m4 + L41^2*m4 + L51^2*m3 - L2*L41*m4*sin(q3 + q4) + 2*L7*L41*m4*sin(al - q4) + L2*L7*m4*cos(al + q3) + L2*L51*m3*cos(al + q3);
M(2,4) = I433 + L41^2*m4 - L2*L41*m4*sin(q3 + q4) + L7*L41*m4*sin(al - q4);
M(3,1) = I313*cos(al + q2 + q3) - I423*cos(q2 + q3 + q4) - I323*sin(al + q2 + q3) - I413*sin(q2 + q3 + q4);
M(3,2) = I333 + I433 + L7^2*m4 + L41^2*m4 + L51^2*m3 - L2*L41*m4*sin(q3 + q4) + 2*L7*L41*m4*sin(al - q4) + L2*L7*m4*cos(al + q3) + L2*L51*m3*cos(al + q3);
M(3,3) = I333 + I433 + L7^2*m4 + L41^2*m4 + L51^2*m3 + 2*L7*L41*m4*sin(al - q4);
M(3,4) = I433 + L41^2*m4 + L7*L41*m4*sin(al - q4);
M(4,1) = - I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4);
M(4,2) = I433 + L41^2*m4 - L2*L41*m4*sin(q3 + q4) + L7*L41*m4*sin(al - q4);
M(4,3) = I433 + L41^2*m4 + L7*L41*m4*sin(al - q4);
M(4,4) = I433 + L41^2*m4;
   
% Centripetal and Coriolis Matrix
% TODO: define the C matrix
C(1,1) = I412*qp2*cos(2*q2 + 2*q3 + 2*q4) - I312*qp3*cos(2*al + 2*q2 + 2*q3) - I212*qp2*cos(2*q2) - I312*qp2*cos(2*al + 2*q2 + 2*q3) + I412*qp3*cos(2*q2 + 2*q3 + 2*q4) + I412*qp4*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp2*sin(2*al + 2*q2 + 2*q3))/2 - (I311*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp3*sin(2*al + 2*q2 + 2*q3))/2 - (I211*qp2*sin(2*q2))/2 + (I222*qp2*sin(2*q2))/2 + (I411*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 + (I411*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 + (I411*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (L7^2*m4*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp2*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp3*sin(2*al + 2*q2 + 2*q3))/2 + (L2^2*m3*qp2*sin(2*q2))/2 + (L2^2*m4*qp2*sin(2*q2))/2 + (L21^2*m2*qp2*sin(2*q2))/2 - (L41^2*m4*qp2*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp3*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp4*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp2*cos(al + 2*q2 + 2*q3 + q4) + L7*L41*m4*qp3*cos(al + 2*q2 + 2*q3 + q4) + (L7*L41*m4*qp4*cos(al + 2*q2 + 2*q3 + q4))/2 + L2*L41*m4*qp2*cos(2*q2 + q3 + q4) + (L2*L41*m4*qp3*cos(2*q2 + q3 + q4))/2 + (L2*L41*m4*qp4*cos(2*q2 + q3 + q4))/2 + L2*L7*m4*qp2*sin(al + 2*q2 + q3) + (L2*L7*m4*qp3*sin(al + 2*q2 + q3))/2 + L2*L51*m3*qp2*sin(al + 2*q2 + q3) + (L2*L51*m3*qp3*sin(al + 2*q2 + q3))/2 - (L2*L41*m4*qp3*cos(q3 + q4))/2 - (L2*L41*m4*qp4*cos(q3 + q4))/2 - (L2*L7*m4*qp3*sin(al + q3))/2 - (L2*L51*m3*qp3*sin(al + q3))/2 - (L7*L41*m4*qp4*cos(al - q4))/2;
C(1,2) = I423*qp2*sin(q2 + q3 + q4) - I323*qp3*cos(al + q2 + q3) - I413*qp2*cos(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I313*qp2*sin(al + q2 + q3) - I313*qp3*sin(al + q2 + q3) - I312*qp1*cos(2*al + 2*q2 + 2*q3) - I323*qp2*cos(al + q2 + q3) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) - I212*qp1*cos(2*q2) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp1*sin(2*al + 2*q2 + 2*q3))/2 - (I211*qp1*sin(2*q2))/2 + (I222*qp1*sin(2*q2))/2 + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - I223*qp2*cos(q2) - I213*qp2*sin(q2) + (L7^2*m4*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L2^2*m3*qp1*sin(2*q2))/2 + (L2^2*m4*qp1*sin(2*q2))/2 + (L21^2*m2*qp1*sin(2*q2))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4) + L2*L41*m4*qp1*cos(2*q2 + q3 + q4) + L2*L7*m4*qp1*sin(al + 2*q2 + q3) + L2*L51*m3*qp1*sin(al + 2*q2 + q3);
C(1,3) = I423*qp2*sin(q2 + q3 + q4) - I323*qp3*cos(al + q2 + q3) - I413*qp2*cos(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I313*qp2*sin(al + q2 + q3) - I313*qp3*sin(al + q2 + q3) - I312*qp1*cos(2*al + 2*q2 + 2*q3) - I323*qp2*cos(al + q2 + q3) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) - (I311*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I322*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*qp1*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*qp1*sin(2*al + 2*q2 + 2*q3))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4) + (L2*L41*m4*qp1*cos(2*q2 + q3 + q4))/2 + (L2*L7*m4*qp1*sin(al + 2*q2 + q3))/2 + (L2*L51*m3*qp1*sin(al + 2*q2 + q3))/2 - (L2*L41*m4*qp1*cos(q3 + q4))/2 - (L2*L7*m4*qp1*sin(al + q3))/2 - (L2*L51*m3*qp1*sin(al + q3))/2;
C(1,4) = I423*qp2*sin(q2 + q3 + q4) - I413*qp3*cos(q2 + q3 + q4) - I413*qp4*cos(q2 + q3 + q4) - I413*qp2*cos(q2 + q3 + q4) + I423*qp3*sin(q2 + q3 + q4) + I423*qp4*sin(q2 + q3 + q4) + I412*qp1*cos(2*q2 + 2*q3 + 2*q4) + (I411*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*qp1*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7*L41*m4*qp1*cos(al + 2*q2 + 2*q3 + q4))/2 + (L2*L41*m4*qp1*cos(2*q2 + q3 + q4))/2 - (L2*L41*m4*qp1*cos(q3 + q4))/2 - (L7*L41*m4*qp1*cos(al - q4))/2;
C(2,1) = -qp1*(I412*cos(2*q2 + 2*q3 + 2*q4) - I212*cos(2*q2) - I312*cos(2*al + 2*q2 + 2*q3) - (I311*sin(2*al + 2*q2 + 2*q3))/2 + (I322*sin(2*al + 2*q2 + 2*q3))/2 - (I211*sin(2*q2))/2 + (I222*sin(2*q2))/2 + (I411*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*sin(2*al + 2*q2 + 2*q3))/2 + (L2^2*m3*sin(2*q2))/2 + (L2^2*m4*sin(2*q2))/2 + (L21^2*m2*sin(2*q2))/2 - (L41^2*m4*sin(2*q2 + 2*q3 + 2*q4))/2 + L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4) + L2*L41*m4*cos(2*q2 + q3 + q4) + L2*L7*m4*sin(al + 2*q2 + q3) + L2*L51*m3*sin(al + 2*q2 + q3));
C(2,2) = - qp3*(L2*L41*m4*cos(q3 + q4) + L2*L7*m4*sin(al + q3) + L2*L51*m3*sin(al + q3)) - qp4*(L2*L41*m4*cos(q3 + q4) + L7*L41*m4*cos(al - q4));
C(2,3) = - L2*qp2*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L2*qp3*(L41*m4*cos(q3 + q4) + L7*m4*sin(al + q3) + L51*m3*sin(al + q3)) - L41*m4*qp4*(L2*cos(q3 + q4) + L7*cos(al - q4));
C(2,4) = -L41*m4*(L2*cos(q3 + q4) + L7*cos(al - q4))*(qp2 + qp3 + qp4);
C(3,1) = -qp1*(I412*cos(2*q2 + 2*q3 + 2*q4) - I312*cos(2*al + 2*q2 + 2*q3) - (I311*sin(2*al + 2*q2 + 2*q3))/2 + (I322*sin(2*al + 2*q2 + 2*q3))/2 + (I411*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*sin(2*q2 + 2*q3 + 2*q4))/2 + (L7^2*m4*sin(2*al + 2*q2 + 2*q3))/2 + (L51^2*m3*sin(2*al + 2*q2 + 2*q3))/2 - (L41^2*m4*sin(2*q2 + 2*q3 + 2*q4))/2 - (L2*L41*m4*cos(q3 + q4))/2 - (L2*L7*m4*sin(al + q3))/2 - (L2*L51*m3*sin(al + q3))/2 + L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4) + (L2*L41*m4*cos(2*q2 + q3 + q4))/2 + (L2*L7*m4*sin(al + 2*q2 + q3))/2 + (L2*L51*m3*sin(al + 2*q2 + q3))/2);
C(3,2) = qp2*(L2*L41*m4*cos(q3 + q4) + L2*L7*m4*sin(al + q3) + L2*L51*m3*sin(al + q3)) - L7*L41*m4*qp4*cos(al - q4);
C(3,3) = -L7*L41*m4*qp4*cos(al - q4);
C(3,4) = -L7*L41*m4*cos(al - q4)*(qp2 + qp3 + qp4);
C(4,1) = -qp1*(I412*cos(2*q2 + 2*q3 + 2*q4) + (I411*sin(2*q2 + 2*q3 + 2*q4))/2 - (I422*sin(2*q2 + 2*q3 + 2*q4))/2 - (L41^2*m4*sin(2*q2 + 2*q3 + 2*q4))/2 - (L2*L41*m4*cos(q3 + q4))/2 - (L7*L41*m4*cos(al - q4))/2 + (L7*L41*m4*cos(al + 2*q2 + 2*q3 + q4))/2 + (L2*L41*m4*cos(2*q2 + q3 + q4))/2);
C(4,2) = qp2*(L2*L41*m4*cos(q3 + q4) + L7*L41*m4*cos(al - q4)) + L7*L41*m4*qp3*cos(al - q4);
C(4,3) = L7*L41*m4*cos(al - q4)*(qp2 + qp3);
C(4,4) = 0;

% Gravitational Torques Vector
% TODO: define the G vector
G(1,1) = 0;
G(2,1) = - gz*m4*(L2*sin(q2) + L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)) - gz*m3*(L2*sin(q2) + L51*sin(al + q2 + q3)) - L21*gz*m2*sin(q2);
G(3,1) = - gz*m4*(L41*cos(q2 + q3 + q4) + L7*sin(al + q2 + q3)) - L51*gz*m3*sin(al + q2 + q3);
G(4,1) = -L41*gz*m4*cos(q2 + q3 + q4);
  
%TODO: define the external torque to simulate the natural dynamics of the
%robot
Tao=-Beta*Qp;
  
%TODO: compute the direct dynamics 
Qpp=M \ ( Tao - C*Qp - G );
% disp("M=")
% disp(M)
% disp("C=")
% disp(C)
% disp("G=")
% disp(G)
% disp(size(Qpp));
