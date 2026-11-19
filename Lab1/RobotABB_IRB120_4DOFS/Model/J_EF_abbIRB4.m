function Jef_0=J_EF_abbIRB4(q,L)
%J_EF_ABBIRB4 calculates the geometric Jacobian of the end-effector wrt the
%robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Jef_0: End-effector Jacobian wrt robot base (6Xn)


% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic paramters
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

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

Jef_0 = [-(s1*(193654783976931328*c234 + 303992974847508480*s2 + 349036234915995125*sa23))/1125899906842624000,                                      c1*((27*c2)/100 + (2792289879327961*ca23)/9007199254740992 - (43*s234)/250),                            c1*((2792289879327961*ca23)/9007199254740992 - (43*s234)/250),            -(43*c1*s234)/250;
          (c1*(193654783976931328*c234 + 303992974847508480*s2 + 349036234915995125*sa23))/1125899906842624000,                                      s1*((27*c2)/100 + (2792289879327961*ca23)/9007199254740992 - (43*s234)/250),                            s1*((2792289879327961*ca23)/9007199254740992 - (43*s234)/250),            -(43*s1*s234)/250;
                                                                                                             0, -((c1^2 + s1^2)*(193654783976931328*c234 + 303992974847508480*s2 + 349036234915995125*sa23))/1125899906842624000, -((193654783976931328*c234 + 349036234915995125*sa23)*(c1^2 + s1^2))/1125899906842624000, -(43*c234*(c1^2 + s1^2))/250;
                                                                                                             0,                                                                                                              -s1,                                                                                      -s1,                          -s1;
                                                                                                             0,                                                                                                               c1,                                                                                       c1,                           c1;
                                                                                                             1,                                                                                                                0,                                                                                        0,                            0];

end