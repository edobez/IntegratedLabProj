clear test*;

%% Import from files 
path = 'C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Misurazioni\';

% Test 1: 2000 +- 500 RPM
filename = 'test1.txt';
test{1} = ImportTestFnc(strcat(path,filename));
test{1}.Properties.Description = 'Test 1: 2000 +- 500 RPM';

% Test 2: 2000 +- 1500 RPM
filename = 'test2.txt';
test{2} = ImportTestFnc(strcat(path,filename));
test{2}.Properties.Description = 'Test 2: 2000 +- 1500 RPM';

% Test 3: 2000 +- 2000 RPM
filename = 'test3.txt';
test{3} = ImportTestFnc(strcat(path,filename));
test{3}.Properties.Description = 'Test 3: 2000 +- 2000 RPM';

% Test 4: 0 +- 500 RPM
filename = 'test4.txt';
test{4} = ImportTestFnc(strcat(path,filename));
test{4}.Properties.Description = 'Test 4: 0 +- 500 RPM';

% Test 5: 2000 +- 2000 RPM (con rotore frenato)
% filename = 'test5.txt';
% test{5} = ImportTestFnc(strcat(path,filename));
% test{5}.Properties.Description = 'Test 5: 2000 +- 2000 RPM (con rotore frenato)';

clear path filename;

%% Condition data
k = -0.3830; % conversione tensione-corrente

for i=1:4 
    test{i}.speedf = smooth(test{i}.speed);
    test{i}.cur = test{i}.cur * k;
    test{i}.curf = smooth(test{i}.cur,41,'sgolay');    
end

%% Plot data
close all;
offset = [2.5665 2.1050 4.306 3.1845]; % Istante in cui parte lo step per ciascun test
twidth = 5; % Finestra temporale del grafico
stepstart = 1; % Instante in cui far partire lo step
dec = 5; % Decimazione

for i=1:4
    rows = test{i}.time < (offset(i) - stepstart + twidth) & test{i}.time >= (offset(i) - stepstart);
    tab = test{i}(rows,:);
    t = tab.time(1:dec:end) - offset(i) + stepstart;
    
    figure('Name',tab.Properties.Description,'NumberTitle','off');
    ha(1) = subplot(2,1,1);
    plot(t*ones(1,2),[tab.ref(1:dec:end) tab.speedf(1:dec:end)]);
    title('Motor Speed');
    ylabel('Volt');
    legend('Set-point','Real system');

    ha(2) = subplot(2,1,2);
    plot(t,tab.curf(1:dec:end));
    title('Armature current');
    xlabel('Time');
    ylabel('Ampere');

    linkaxes(ha,'x');
end
return;

%% Plot test results by combining the plots
i = 1; % Scegliere test da plottare

rows = test{i}.time < (offset(i) - stepstart + twidth) & test{i}.time >= (offset(i) - stepstart);
tab = test{i}(rows,:);
t = tab.time(1:dec:end) - offset(i) + stepstart;

Ts = 0.001;
period = 5;
[ugen,tgen] = gensig('square',period,10,Ts);
ugen = 2*ugen + 3;
ugen = ugen( (period/2-stepstart)/Ts+1:(period/2-stepstart+period)/Ts+1 );
tgen = linspace(0,twidth,twidth/Ts+1);
[ylin,tlin,~] = lsim(tf(W2*H2/tach.gain),ugen ,tgen);

% sim('ModelNonLinearMechFram')

figure('Name',tab.Properties.Description,'NumberTitle','off');
hold on;
plot(t*ones(1,2),[tab.ref(1:dec:end) tab.speedf(1:dec:end)]./tach.gain*rad2rpm);
plot(tlin,ylin*rad2rpm);
plot(sim_omega);

title('Motor Speed');
ylabel('Volt');
legend('Set-point','Real system','Linear','Non-linear');
grid on;
hold off;
