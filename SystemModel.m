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
R8 = 350e3;

Rv2 = realp('Rv2',1000);
Rv2.Minimum = 0;
Rv2.Maximum = 10e3;
Rv2_vals = [Rv2.Minimum:1e3:Rv2.Maximum];


%% Parametri motore DC
motor.rpm_max = 4000; % Max giri motore [rpm]
motor.Tm_max = 0.06; % Coppia max [Nm]

motor.J = 5.18e-6; % Inerzia del motore []
motor.beta = 5e-6; % Attrito viscoso motore [Nm*s/rad]
motor.KT = 0.046; % Costante di coppia [Nm/A]

motor.La = 2.8; % Induttanza di armatura [H]
motor.Ra = 5.5; % Resistenza di armatura [Ohm]

%% Dati ricavati dai parametri
motor.omega_max = motor.rpm_max*2*pi/60; % Velocità angolare max [rad/s]
% KE = Vdc/omegaMAX; % Costante elettrica [V*s/rad]
motor.Ia_max = motor.Tm_max / motor.KT;

%% Schemi a blocchi
G1 = 1/(s*motor.La+motor.Ra);
G1.InputName = '
G2 = 1/(s*motor.J+motor.beta);
G1s = feedback(G1,motor.KT*motor.KT*G2);

C2.Kp = 1;
C2.Ki = 0;
C2.Kd = 0;
C2.gamma = 0.1;
C2.Tf = C2.gamma * C2.Kd / C2.Kp;
C2 = pid(C2.Kp,C2.Ki,C2.Kd,C2.Tf);

%% Calcolo fdt anello di corrente closed loop
C1 = -R8/(s*R8*C+1);
G = C1*-Vdc*G1; % Fdt in catena aperta con rotore bloccato
Gs = C1*-Vdc*G1s; % Fdt in catena aperta con rotore libero

H1 = -(R4*Rs/R1);
H2 = 1/(R7+Rv2);
H = H1*H2;

W = 1/R6 * feedback(G,H,+1);
Ws = 1/R6 * feedback(Gs,H,+1);

Wsample = zpk(replaceBlock(Ws,'Rv2',Rv2_vals));
t = linspace(0,0.04,100);
for i = 1:11
    y(:,i) = step(Wsample(:,:,1,i),t);
end 

mesh(t,Rv2_vals,y')

