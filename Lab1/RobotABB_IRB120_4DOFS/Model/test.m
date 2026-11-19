clear, clc
format longG;
q0 = zeros(1, 6);

% Get ABBIRB4 parameters
L = abbIRB4_params;
% disp(L)

% Set robot base frame as World frame
T0_W = eye(4);

% Get Absolute Homogeneous Transformations
[HT_0, HT_W] = getAbsoluteHT_abbIRB4(q0, L, T0_W);
disp(HT_W{5})


%% Syms 
clc
syms q1 q2 q3 q4 al c1 s1 c2 s2 ca23 sa23 s234 c234 real

L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
% al=L(5);

DH_table = [q1      -pi/2  0    L1;
            q2-pi/2  0     L2   0;
            q3+al    0     L7   0;
            q4-al+pi/2    0     L8   0];

FKDH_sym = @(t, a, r, d) ...
[ cos(t), -sin(t)*cos(a),  sin(t)*sin(a), r*cos(t);
  sin(t),  cos(t)*cos(a), -cos(t)*sin(a), r*sin(t);
  0,           sin(a),             cos(a),          d;
  0,           0,                      0,           1 ];

% TODO: Define the absolute HT wrt robot base (link 0)
T1_0 = simplify(FKDH_sym(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4)))
T2_0 = simplify(T1_0*FKDH_sym(DH_table(2,1), DH_table(2,2), DH_table(2,3), DH_table(2,4)))
T3_0 = simplify(T2_0*FKDH_sym(DH_table(3,1), DH_table(3,2), DH_table(3,3), DH_table(3,4)))
T4_0 = simplify(T3_0*FKDH_sym(DH_table(4,1), DH_table(4,2), DH_table(4,3), DH_table(4,4)))


% Substitution list (expressions to replace)
old = [ cos(q1),  sin(q1), ...
        cos(q2),  sin(q2), ...
        cos(al+q2+q3),  sin(al+q2+q3), ...
        sin(q2+q3+q4),  cos(q2+q3+q4) ];

% New symbolic variables
new = [ c1, s1, ...
        c2, s2, ...
        ca23, sa23, ...
        s234, c234 ];

% Perform substitution
T1_0 = simplify(subs(T1_0, old, new))
T2_0 = simplify(subs(T2_0, old, new))
T3_0 = simplify(subs(T3_0, old, new))
T4_0 = simplify(subs(T4_0, old, new))

%% jacobian sumbolic
clc
syms q1 q2 q3 q4 al c1 s1 c2 s2 ca23 sa23 s234 c234 qp1 qp2 qp3 qp4 real

n = 4;

HT_0={eye(4),T1_0, T2_0, T3_0, T4_0};

% End-effector origin (frame 4 wrt 0)
o_n = HT_0{5}(1:3,4)

% Preallocate
Jv = sym(zeros(3,n));
Jw = sym(zeros(3,n));

for i = 1:n
    
    % z_{i-1} and o_{i-1}
    T_prev = HT_0{i};
    
    z = T_prev(1:3,3);   % z-axis of frame i-1 wrt base
    o = T_prev(1:3,4);   % origin of frame i-1 wrt base
    
    % All joints are revolute
    Jv(:,i) = cross(z, o_n - o);
    Jw(:,i) = z;
    
end

% TODO: Calculate the Jacobian of the end-effector wrt link 0 (6xn)
Jef_0 = simplify([Jv;
                  Jw])
Xp=simplify(Jef_0 * [qp1; qp2; qp3; qp4]);

%% Jacobian test
clc
L = abbIRB4_params;
u_0 = [0 0 0.30 0 0 122.79 9.26 1 0 0 0 0]; 

% Joint positions
Q_0 = u_0(2:5);

% Joint velocities
Qp_0 = u_0(9:12);      

% Get end-effector Jacobian
jef_0 = J_EF_abbIRB4(Q_0, L)
% syms q1 q2 q3 q4
% jef_q = simplify(jef_0*[q1;q2;q3;q4])
