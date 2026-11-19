function [HT_0, HT_W] = getAbsoluteHT_abbIRB4(q,L, T0_W)
%GETABSOLUTEHT_abbIRB4 calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: Cell array with all the absolute HT (4x4) wrt robot base
%   HT_W: Cell array with all the absolute HT (4x4) wrt wcf

% Joint postions

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions

c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

s234=sin(q2 + q3 + q4);
c234=cos(q2 + q3 + q4);


% TODO: Define the absolute HT wrt robot base (link 0)
    T1_0 = [c1,  0, -s1,      0;
            s1,  0,  c1,      0;
             0, -1,   0, 29/100;
             0,  0,   0,      1];
    
    
    T2_0 = [c1*s2, c1*c2, -s1,       (27*c1*s2)/100;
            s1*s2, c2*s1,  c1,       (27*s1*s2)/100;
               c2,   -s2,   0, (27*c2)/100 + 29/100;
                0,     0,   0,                    1];
    
    
    T3_0 = [c1*sa23, c1*ca23, -s1,    (27*c1*s2)/100 + (2792289879327961*c1*sa23)/9007199254740992;
            s1*sa23, ca23*s1,  c1,    (27*s1*s2)/100 + (2792289879327961*s1*sa23)/9007199254740992;
               ca23,   -sa23,   0, (27*c2)/100 + (2792289879327961*ca23)/9007199254740992 + 29/100;
                  0,       0,   0,                                                               1];
    
    
    T4_0 = [c1*c234, -c1*s234, -s1, (c1*(193654783976931328*c234 + 303992974847508480*s2 + 349036234915995125*sa23))/1125899906842624000;
            c234*s1, -s1*s234,  c1, (s1*(193654783976931328*c234 + 303992974847508480*s2 + 349036234915995125*sa23))/1125899906842624000;
              -s234,    -c234,   0,                      (27*c2)/100 + (2792289879327961*ca23)/9007199254740992 - (43*s234)/250 + 29/100;
                  0,        0,   0,                                                                                                    1];


% List with all the HT wrt to link 0, starting with H0_0
HT_0={eye(4),T1_0, T2_0, T3_0, T4_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
HT_W={eye(4),T0_W,T0_W*T1_0,T0_W*T2_0,T0_W*T3_0,T0_W*T4_0};

end

