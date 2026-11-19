function [P,Pp,Ppp] = Pol5th( Pini, Pend, tini,tend,t )
    %POL5TH Generates a n-DOF trajectory based on a 5th degree polynomial
    %function
    %   The function calculates a trajectory between the initial position to
    %   the final position with a duration of tend-tini
    %   Pini:   initial position (nx1)
    %   Pend:   final position (nx1)
    %   tini:   initial time
    %   tend:   final time
    %   t:      current time (simulation time)
    % Return:
    %   P:  position vector (nx1)
    %   Pp:  velocity vector (nx1)
    %   Ppp:  velocity vector (nx1)
    n = numel(Pini);
    P = zeros(n,1); % TODO
    Pp  = zeros(n,1);
    Ppp = zeros(n,1);
    if t < tini
        P = Pini;
        Pp  = zeros(n,1);
        Ppp = zeros(n,1);
    elseif t > tend
        P = Pend;
    else
        Dt  = t - tini;
        Fac = tend - tini;
        r   = Dt / Fac;
        r2  = r*r;
        r3  = r2*r;
        r4  = r3*r;
        r5  = r4*r;

        for i = 1:3
            dP     = Pend(i) - Pini(i);
            P(i,1) = Pini(i) + dP * (10*r3 - 15*r4 + 6*r5);
            Pp(i,1)  = dP * (30*r2 - 60*r3 + 30*r4) / Fac;
            Ppp(i,1) = dP * (60*r - 180*r2 + 120*r3) / (Fac^2);
        end
    end
end


