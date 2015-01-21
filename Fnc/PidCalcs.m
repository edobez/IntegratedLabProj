function [ P1,zero_f ] = PidCalcs( P2,Kdc,pid )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

P1 = ( pid.R6*P2 + pid.R6*pid.R8 + pid.R5*pid.R9 - pid.R5*pid.R6*Kdc ) / ( pid.R5*Kdc - P2 - pid.R8 )
zero_T = ( (P2+pid.R8)*(pid.C3*pid.R5*pid.R9 + P1 + pid.R6) ) / ...
    ( (P1 + pid.R6)*(P2+pid.R8)+pid.R5*pid.R9 );

zero_f = 1/zero_T

end

