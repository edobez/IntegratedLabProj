clear all, close all, clc;

s = tf('s');

%% Parametri sistema
Vdc = 30;
Ktach = 6/(4000*2*pi/60); % Guadagno tach. [V/rad/s]

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
motor.KE = motor.KT;

motor.La = 2.8; % Induttanza di armatura [H]
motor.Ra = 5.5; % Resistenza di armatura [Ohm]

motor.omega_max = motor.rpm_max*2*pi/60; % Velocità angolare max [rad/s]
motor.Ia_max = motor.Tm_max / motor.KT;

%% Schemi a blocchi
G1 = 1/(s*motor.La+motor.Ra);
G1.InputName = 'voltage';
G1.OutputName = 'current';

G2 = 1/(s*motor.J+motor.beta);
G2.InputName = 'torque';
G2.OutputName = 'rad/s';

G3 = feedback(G1,motor.KE*motor.KT*G2);

%% Calcolo fdt anello di corrente closed loop
C1 = -R8/(s*R8*C+1);
G4blocked = C1*-Vdc*G1; % Fdt in catena aperta con rotore bloccato
G4 = C1*-Vdc*G3; % Fdt in catena aperta con rotore libero

H1 = -(R4*Rs/R1) * 1/(R7+Rv2);

W1blocked = 1/R6 * feedback(G4blocked,H1,+1);
W1 = 1/R6 * feedback(G4,H1,+1);

%% Calcolo fdt anello di velocità
C2.Kp = 1;
C2.Ki = 0;
C2.Kd = 0;
C2.gamma = 0.1;
C2.Tf = C2.gamma * C2.Kd / C2.Kp;
C2 = pid(C2.Kp,C2.Ki,C2.Kd,C2.Tf);

H2 = Ktach;

G5 = W1 * motor.KT * G2; % Catena aperta 
G5.InputName = 'voltage';
G5.OutputName = 'omega';

W2 = feedback(C2*G5,H2)
return;

%% Step su W con Rv2 variabile
Wsample = zpk(replaceBlock(W1,'Rv2',Rv2_vals));
t = linspace(0,0.04,100);
for i = 1:11
    y(:,i) = step(Wsample(:,:,1,i),t);
end 

mesh(t,Rv2_vals,y')

