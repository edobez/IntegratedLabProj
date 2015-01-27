
s = tf('s');
rad2rpm = 60/(2*pi);
rpm2rad = (2*pi)/60;
par_res = @(r1,r2) (r1*r2)/(r1+r2);

%% Parametri sistema
Vdc = 30;
Kpwm = 1/8;
% Ktach = 8/(4000*rpm2rad); % Guadagno tach. [V/rad/s]
Vsat = 11;

C = 10e-9;      % Capacità condensatore
R1 = 1.8e3;
R4 = 47e3;
R6 = 100e3;
R7 = 27e3;
Rs = 0.1;       % Resistenza di shunt

R8 = realp('R8',100e6);

Rv2 = 0;

%% Parametri motore DC
motor.rpm_max = 4000;       % Max giri motore [rpm]
motor.Tm_max = 0.06;        % Coppia max [Nm]

motor.J = 5.18e-6;              % Inerzia del motore []
motor.Jadd = 5.8e-6;
% busta = [0 0.78];
busta = [1 1];
motor.beta = 1.5327e-05*busta(1);        % Attrito viscoso motore [Nm*s/rad]
motor.coulomb = 0.0073*busta(2);         % Offset dovuto alla forza di Coulomb [Nm]
motor.stick = 0.009689;         % Stiction [Nm]
motor.KT = 0.046;               % Costante di coppia [Nm/A]
motor.KE = motor.KT;

motor.La = 2.8e-3;          % Induttanza di armatura [H]
motor.Ra = 5.5;             % Resistenza di armatura [Ohm]

motor.omega_max = motor.rpm_max*2*pi/60; % Velocità angolare max [rad/s]
motor.Ia_max = motor.Tm_max / motor.KT;

%% Schemi a blocchi
G1 = 1/(s*motor.La+motor.Ra+Rs);               % parte elettrica motore
% G1.InputName = 'voltage';
% G1.OutputName = 'current';

G2 = 1/(s*(motor.J+motor.Jadd)+motor.beta); % parte meccanica motore
% G2.InputName = 'torque';
% G2.OutputName = 'rad/s';

% G2nl = 1/(s*(motor.J+Jdisco));
% G2nl.InputName = 'torque';
% G2nl.OutputName = 'rad/s';

G3 = feedback(G1,motor.KE*motor.KT*G2);     % parte elettrica motore + retroazione fem

%% Calcolo fdt anello di corrente closed loop
C1 = -R8/(s*R8*C+1);        % "controllore" anello di corrente

C1_dig = 2.6/(s*C*R7);      % controllore digitale
C1_dig = c2d(tf(C1_dig),0.0001,'tustin');

G4blocked = C1*-Vdc*Kpwm*G1;     % Fdt in catena aperta con rotore bloccato
G4 = C1*-Vdc*Kpwm*G3;            % Fdt in catena aperta con rotore libero
% G4_dig = -C1_dig*-Vdc*Kpwm*G3;

H1 = Rs * (R4/R1) * 1/(R7+Rv2);
% H1_dig = Rs * (R4/R1) * 1/2.6;

W1blocked = 1/R6 * feedback(G4blocked,H1);       % fdt rotore bloccato
W1 = 1/R6 * feedback(G4,H1);                     % fdt closed-loop anello corrente
% W1_dig = feedback(G4_dig,H1_dig);

%% Calcolo fdt anello di velocità
% Proporzionale
pid.R5 = 10e3;
pid.R8 = 10e3;
pid.P2 = realp('P2',4.7138e+04);
pid.P2.Minimum = 0;
pid.P2.Maximum = 1e6;
pid.C3 = 1e-9;
pid.p = -1/pid.R5 * (pid.R8 + pid.P2) / (s*(pid.R8+pid.P2)*pid.C3 + 1);
pid.Kp = (pid.R8 + pid.P2)/pid.R5;

% Integrativo
pid.R6 = 10e3;
pid.R9 = 1e9;
pid.P1 = realp('P1', 115000); % Integrativo
pid.P1.Minimum = 0;
pid.P1.Maximum = 1e6;
pid.C4 = 1e-6;
% pid.i = -(pid.R9)/(pid.P1+pid.R6) * 1/(s*pid.R9*pid.C4+1);
pid.i = -1/(s*pid.C4*(pid.P1+pid.R6));
% pid.Ki = 1/(pid.C4*(pid.P1+pid.R6));
pid.bc = pid.R9/(pid.P1 + pid.R6); % back-calculation gain

% Derivativo
pid.R10 = 1e3;
pid.P3 = realp('P3', 0); 
pid.P3.Minimum = 0;
pid.P3.Maximum = 1e6;
pid.C1 = 4.7e-9;
pid.C2 = 1e-6;
pid.d = -(s*(pid.R10+pid.P3)*pid.C2)/(1+s*(pid.R10+pid.P3)*pid.C1);
pid.Kd = pid.C2*(pid.P3+pid.R10);

%double([pid.Kp pid.Ki pid.Kd])

C2 = -(pid.p + pid.i);
C2_dig = C2*R7/(R6*2.6);
C2_dig = c2d(tf(C2_dig),0.001,'tustin');

%% Tachimetro

tach.K = 0.003/rpm2rad; % Guadagno tachimetro (dal datasheet)
tach.C1 = 1e-6;
tach.R1 = 10e3;
tach.Rv1 = 47e3;
tach.Rv1_b = 38e3; % Parte del potenziometro fra il pin centrale e massa
tach.Rv1_a = tach.Rv1 - tach.Rv1_b;

tach.tau = tach.C1 * par_res(tach.R1,tach.Rv1);
tach.gain = (tach.K * tach.Rv1_b) / (tach.Rv1 + tach.R1);

H2 = tach.gain * 1/(s*tach.tau + 1); 

G5 = W1 * motor.KT * G2;    % Catena aperta plant
G5.InputName = 'voltage';
G5.OutputName = 'omega';

G5_uf = minreal(zpk(G5)*H2);

% G5_dig = W1_dig * motor.KT * G2;

W2 = feedback(C2*G5,H2);    % Catena chiusa totale
% W2_dig = feedback(C2_dig*G5_dig,H2);
L = G5*H2;
S = 1/(1+L);
T = 1-S;

%% Vettori velocità angolare e coppia resistente per la creazione della Look-Up table

% w = [0.00001; 600; 1000; 1435; 1870; 2300; 2760; 3210; 3660];
% Tr = [0.013; 0.010658298; 0.011627234; 0.01259617; 0.013124681; 0.013776511; 0.013917447; 0.014357872; 0.014534043];