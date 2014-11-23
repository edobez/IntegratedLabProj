close all;
clc;

s = tf('s');
rad2rpm = 60/(2*pi);
rpm2rad = (2*pi)/60;

%% Parametri sistema
Vdc = 30;
Ktach = 8/(4000*rpm2rad); % Guadagno tach. [V/rad/s]

C = 10e-9;      % Capacità condensatore
R1 = 1.8e3;
R4 = 47e3;
R6 = 100e3;
R7 = 27e3;
Rs = 0.1;       % Resistenza di shunt

R8 = realp('R8',100e6);

Rv2 = realp('Rv2', 0);
Rv2.Minimum = 0;
Rv2.Maximum = 10e3;

%% Parametri motore DC
motor.rpm_max = 4000;       % Max giri motore [rpm]
motor.Tm_max = 0.06;        % Coppia max [Nm]

motor.J = 5.18e-6;          % Inerzia del motore []
motor.beta = 1.275e-5;      % Attrito viscoso motore [Nm*s/rad] - DA MISURARE
motor.KT = 0.046;           % Costante di coppia [Nm/A]
motor.KE = motor.KT;

motor.La = 2.8e-3;             % Induttanza di armatura [H]
motor.Ra = 5.5;             % Resistenza di armatura [Ohm]

motor.omega_max = motor.rpm_max*2*pi/60; % Velocità angolare max [rad/s]
motor.Ia_max = motor.Tm_max / motor.KT;

%% Schemi a blocchi
G1 = 1/(s*motor.La+motor.Ra);               % parte elettrica motore
G1.InputName = 'voltage';
G1.OutputName = 'current';

G2 = 1/(s*motor.J+motor.beta);              % parte meccanica motore
G2.InputName = 'torque';
G2.OutputName = 'rad/s';

G3 = feedback(G1,motor.KE*motor.KT*G2);     % parte elettrica motore + retroazione fem

%% Calcolo fdt anello di corrente closed loop
C1 = -R8/(s*R8*C+1);        % "controllore" anello di corrente
G4blocked = C1*-Vdc*G1;     % Fdt in catena aperta con rotore bloccato
G4 = C1*-Vdc*G3;            % Fdt in catena aperta con rotore libero

H1 = (R4*Rs/R1) * 1/(R7+Rv2);

W1blocked = 1/R6 * feedback(G4blocked,H1);       % fdt rotore bloccato
W1 = 1/R6 * feedback(G4,H1);                     % fdt closed-loop anello corrente

%% Calcolo fdt anello di velocità
% Proporzionale
pid.R5 = 10e3;
pid.R8 = 10e3;
pid.P2 = realp('P2',450e3); 
pid.P2.Minimum = 0;
pid.P2.Maximum = 1e6;
pid.C3 = 1e-9;
pid.p = -1/pid.R5 * (pid.R8 + pid.P2) / (s*(pid.R8+pid.P2)*pid.C3 + 1);
pid.Kp = (pid.R8 + pid.P2)/pid.R5;

% Integrativo
pid.R6 = 4.7e3;
pid.R9 = 1e6;
pid.P1 = realp('P1',1e3); % Integrativo
pid.P1.Minimum = 0;
pid.P1.Maximum = 100e3;
pid.C4 = 1e-6;
pid.i = -1/(pid.P1+pid.R6) * (pid.R9)/(s*pid.R9*pid.C4+1);
pid.Ki = 1/(pid.C4*(pid.P1+pid.R6));

% Derivativo
pid.R10 = 1e3;
pid.P3 = realp('P3',0); 
pid.P3.Minimum = 0;
pid.P3.Maximum = 1e6;
pid.C1 = 4.7e-9;
pid.C2 = 1e-6;
pid.d = -(s*(pid.R10+pid.P3)*pid.C2)/(1+s*(pid.R10+pid.P3)*pid.C1);
pid.Kd = pid.C2*(pid.P3+pid.R10);

% double([pid.Kp pid.Ki pid.Kd])

C2 = -(pid.p + pid.i + pid.d);

tach.C1 = 1e-6;
tach.R1 = 10e3;
tach.Rv1 = 47e3;
tach.Rv1_2 = 40.8e3; % Parte del potenziometro fra il pin centrale e massa
tach.Rv1_1 = tach.Rv1 - tach.Rv1_2;

H2 = Ktach;

G5 = W1 * motor.KT * G2;    % Catena aperta plant
G5.InputName = 'voltage';
G5.OutputName = 'omega';

G5_uf = minreal(zpk(G5))*H2;

W2 = feedback(C2*G5,H2);    % Catena chiusa totale

