%% Calcolo misure indirette

Ia = misureB.rawCurrent * -R1/(Rs*R4); % Armature current
misureB.frictionTorque = Ia * motor.KT;
sp = misureB.tach / tach.gain * rad2rpm;    % [RPM]
tau = misureB.frictionTorque;
p = polyfit(sp(1:end-2),tau(1:end-2),1);
betafit = p(1)
co = p(2)
betafitRads = betafit / rpm2rad
beta = (tau - co) ./ sp;

%% Plots data
close all;

f1 = figure;
hold on;

yfit = polyval(p,[sp(1) sp(end)]);
plot(sp,tau);
plot([sp(1) sp(end)],yfit,'--');
title('Friction torque');
xlabel('RPM');
ylabel('Nm');
text(500,0.0145,sprintf('Linearized Beta = %.4g Nm/RPM',betafit));
text(500,0.014,sprintf('Linearized Beta = %.4g Nm/rad/s',betafit/rpm2rad));
text(500,0.0135,sprintf('Linearized Coulomb friction = %.4g Nm',co));
hold off;

figure;
hold on;

plot(sp,beta);
title('Beta (RPM)');
xlabel('RPM');
ylabel('Nm/RPM');
plot([0 4000],[betafit betafit],'--')
text(1000,0.5e-6,sprintf('Linearized Beta = %.4g Nm/RPM',betafit));
text(1000,0,sprintf('Linearized Beta = %.4g Nm/rad/s',betafit/rpm2rad));

% figure;
% hold on;
% 
% plot(sp*rpm2rad,beta/rpm2rad);
% title('Beta (rad/s)');
% xlabel('rad/s');
% ylabel('Nm*s/rad');
% plot([0 450],[betafit/rpm2rad betafit/rpm2rad],'--');
% text(100,0.75e-5,sprintf('Linearized Beta = %.3g Nm/rad/s',betafit/rpm2rad));