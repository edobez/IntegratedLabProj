%% Calcolo misure indirette
Ia = misureB.rawCurrent * -R1/(Rs*R4); % Armature current
misureB.frictionTorque = Ia * motor.KT;
sp = misureB.shaftSpeed;
tau = misureB.frictionTorque;
p = polyfit(sp(2:end),tau(2:end),1);
betafit = p(1)
co = p(2)
betafitRads = betafit / rpm2rad
beta = (tau - co) ./ sp;

%% Plots data
close all;

figure;
hold on;

yfit = polyval(p,[sp(1) sp(end)]);
plot(sp,tau);
plot([sp(1) sp(end)],yfit,'--');
title('Shaft speed vs Friction torque');
xlabel('RPM');
ylabel('Nm');
text(500,0.015,sprintf('Fitted Beta = %.2g Nm/RPM',betafit));
text(500,0.0145,sprintf('Coulomb friction = %.2g Nm',co));
hold off;

figure;
hold on;

plot(sp,beta);
title('Shaft speed vs Damping factor');
xlabel('RPM');
ylabel('Nm*s/rad');
plot([0 4000],[betafit betafit])