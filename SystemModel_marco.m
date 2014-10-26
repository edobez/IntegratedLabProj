clear all, close all, clc;

s = tf('s');

%% Parametri sistema
Vdc = 30;
% Kshunt = 1; % Guadagno current-sense
Ktach = 1; % Guadagno speed-sense
% Kc1_P = 3; % Guadagno controllore C1
% Kc1_I = 4;
C = 10e-9; % Capacità condensatore
R1 = 1.8e3;
R4 = 47e3;
R6 = 100e3;
R7 = 27e3;
Rs = 0.1; % Resistenza di shunt
Rv2 = 0; % Trimmer
R8 = 3e5;



%% Parametri motore DC
nMAX = 4000; % Max giri motore [rpm]
TmMAX = 0.06; % Coppia max [Nm]

J = 5.18e-6; % Inerzia del motore []
beta = 5e-6; % Attrito viscoso motore [Nm*s/rad]
KT = 0.046; % Costante di coppia [Nm/A]

La = 2.8; % Induttanza di armatura [H]
Ra = 5.5; % Resistenza di armatura [Ohm]

%% Dati ricavati dai parametri
omegaMAX = nMAX*2*pi/60; % Velocità angolare max [rad/s]
% KE = Vdc/omegaMAX; % Costante elettrica [V*s/rad]
IaMAX = TmMAX / KT;

%% Schemi a blocchi
G1 = 1/(s*La+Ra);
G2 = 1/(s*J+beta);

%% Calcolo fdt anello di corrente closed loop
C1 = R8/(1+s*C*R8);
G = C1*Vdc*G1;
H = (Rs*R4)/(R1*(Rv2+R7));
W = feedback(G, H);
W2 = (1/R6)*W
step(W2)
figure();
bode(W2)