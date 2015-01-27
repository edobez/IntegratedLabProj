function [ solP1,solP2,pole ] = compute_pot( Kdc,zero,pid)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

syms P1 P2

% Equazioni con R9
% eq1 = Kdc - ( (P1+pid.R6)*(P2+pid.R8)+pid.R5*pid.R9 ) / (pid.R5*(P1+pid.R6));
% eq2 = 1/zero - ( (P2+pid.R8)*(pid.C3*pid.R5*pid.R9 + P1 + pid.R6) ) / ...
%     ( (P1 + pid.R6)*(P2+pid.R8)+pid.R5*pid.R9 );

% Equazioni senza R9
tau1 = pid.C3*(pid.R8 + P2);
Kp = (pid.R8 + P2)/pid.R5;
Ki = 1/(pid.C4*(pid.R6 + P1));
eq1 = Kdc == Ki;
eq2 = 1/zero == Kp/Ki + tau1;

[solP1,solP2] = solve(eq1,eq2);

solP1 = double(solP1);
solP2 = double(solP2);

solP1 = solP1(solP1 > 0 & solP1 < 1e7);
solP2 = solP2(solP2 > 0 & solP2 < 1e7);
pole = -1/(pid.C3*(solP2+pid.R8));

end

