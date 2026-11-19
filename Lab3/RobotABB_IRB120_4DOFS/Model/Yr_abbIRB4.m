function Yr_out = Yr_abbIRB4(u)
%YR_ABBIRB4 Computes the Robot regressor

% Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);
q = [q1 q2 q3 q4];


% Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);
qp = [qp1 qp2 qp3 qp4];


% Joint Velocity reference
qp1r=u(9);
qp2r=u(10);
qp3r=u(11);
qp4r=u(12);
qpr = [qp1r qp2r qp3r qp4r];


% Joint Acceleration reference
qpp1r=u(13);
qpp2r=u(14);
qpp3r=u(15);
qpp4r=u(16);
qppr = [qpp1r qpp2r qpp3r qpp4r];

%Gravity
gz=u(17);

% Kinematic Parameters
% p=[L1,L2,L3,L4,L6,L7,L8,al];
L=abbIRB4_params;
L2 = L(2); 
L3 = L(3); 
L4 = L(4); 
% L6 = L(5);
L7 = L(6);
L8 = L(7);

L21=L2/2;
L31=L3/2; 
L32=L4/2; 
L41=L8/2; 
L51=sqrt(L32^2+L31^2); 
% Angle offset
al=L(5);

% Common substitutions
c22=cos(2*q2);
s22=sin(2*q2);

c2a_22_23=cos(2*al + 2*q2 + 2*q3);
s2a_22_23=sin(2*al + 2*q2 + 2*q3);

sa_22_3=sin(al + 2*q2 + q3);
ca_22_3=cos(al + 2*q2 + q3);

ca3=cos(al + q3);
sa3=sin(al + q3);

s22_23_24=sin(2*q2 + 2*q3 + 2*q4);
c22_23_24=cos(2*q2 + 2*q3 + 2*q4);

c22_3_4=cos(2*q2 + q3 + q4);
s22_3_4=sin(2*q2 + q3 + q4);

s34=sin(q3 + q4);
c34=cos(q3 + q4);

c2=cos(q2);
s2=sin(q2);

sa_22_23_4=sin(al + 2*q2 + 2*q3 + q4);
ca_22_23_4=cos(al + 2*q2 + 2*q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

sam4=sin(al - q4);
cam4=cos(al - q4);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

% TODO: Define the robot regressor Yr(q,qp,qpr,qppr)
Yr = Yr_abbIRB4_Num(q, qp, qpr, qppr, L2, L7, L21, L41, L51, al, gz);

% TODO: Define the robot regressor extension with the viscous friction
% coefficients
Yr_Beta=diag([qp1 qp2 qp3 qp4]);

% TODO: Extended regressor with robot dynamic model and viscous friction
% Yr_ex=[Yr Yr_Beta];
Yr_ex = Yr;

% Regressor serialization
% TODO: define the variables n=DOFs, p=number of parameters
n=4;
p=39;
% disp(n*p);
Yr_out=reshape(Yr_ex,n*p,1);

%To recover the Yr matrix
%Yr_in=reshape(Yr_out,n,p);
end

