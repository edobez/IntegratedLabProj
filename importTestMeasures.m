clear test*;

%% Import from files 
path = 'C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Misure\analog\';
% path = 'C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Misure\digital\';

% Test 1: 2000 +- 500 RPM
filename = 'test1.txt';
test{1} = ImportTestFnc(strcat(path,filename));
test{1}.Properties.Description = 'mesTest1';

% Test 2: 2000 +- 1500 RPM
filename = 'test2.txt';
test{2} = ImportTestFnc(strcat(path,filename));
test{2}.Properties.Description = 'mesTest2';

% Test 3: 2000 +- 2000 RPM
filename = 'test3.txt';
test{3} = ImportTestFnc(strcat(path,filename));
test{3}.Properties.Description = 'mesTest3';

% Test 4: 0 +- 500 RPM
filename = 'test4.txt';
test{4} = ImportTestFnc(strcat(path,filename));
test{4}.Properties.Description = 'mesTest4';

clear path filename;

%% Condition data
k = -0.3830; % conversione tensione-corrente

for i=1:4
    test{i}.reff = smooth(test{i}.ref,11);
    test{i}.speedf = smooth(test{i}.w,51);
    test{i}.curf = smooth(test{i}.cur,51); 
end

%% Plot data
close all;
offset = [1.8935 0.7635 1.8105 1.187]; % Istante in cui parte lo step per ciascun test (ANALOG)
% offset = [1.6785 1.9252 2.0835 1.9345]; % (DIGITAL)
twidth = 15; % Finestra temporale del grafico
stepstart = 1; % Instante in cui far partire lo step
dec = 5; % Decimazione

for i=1:4
    rows = test{i}.time < (offset(i) - stepstart + twidth) & test{i}.time >= (offset(i) - stepstart);
    tab = test{i}(rows,:);
    t = tab.time(1:dec:end) - offset(i) + stepstart;
    
    figure('Name',tab.Properties.Description,'NumberTitle','off');
    ha(1) = subplot(2,1,1);
    plot(t*ones(1,2),[tab.reff(1:dec:end) tab.speedf(1:dec:end)]);
%     plot(t,tab.speedf(1:dec:end));  
    title('Motor Speed');
    ylabel('Volt');
    legend('Reference','Real system');

    ha(2) = subplot(2,1,2);
    plot(t,tab.curf(1:dec:end));
    title('Armature current');
    xlabel('Time');
    ylabel('Ampere');

    linkaxes(ha,'x');
end
return;

%% Plot test results by combining the plots
i = 4; % Scegliere test da plottare

rows = test{i}.time < (offset(i) - stepstart + twidth) & test{i}.time >= (offset(i) - stepstart);
tab = test{i}(rows,:);
t = tab.time(1:dec:end) - offset(i) + stepstart;

% REFERENCE generation
Ts = 0.0001;
period = 5;
[ugen,tgen] = gensig('square',period,20,Ts);
ugen_pars = [2 3;6 1;8 0;2 -1];
ugen = ugen_pars(i,1)*ugen + ugen_pars(i,2);
ugen = ugen( (period/2-stepstart)/Ts+1:(period/2-stepstart+twidth)/Ts+1 );
tgen = linspace(0,twidth,twidth/Ts+1);
[ylin,tlin,~] = lsim(tf(W2*H2/tach.gain),ugen ,tgen);

% sim('ModelNonLinearMechFram')

fig = figure('Name',tab.Properties.Description,'NumberTitle','off');
hold on;

% REFERENCE
plot(t,tab.reff(1:dec:end)/tach.gain*rad2rpm); %Plot Reference and speed (ANALOG)
% plot(tgen,ugen/tach.gain*rad2rpm);  % Plot Reference (DIGITAL)

% SPEED
plot(t,tab.speedf(1:dec:end)/tach.gain*rad2rpm); % (ANALOG)
% plot(t,(tab.speedf(1:dec:end)/tach.gain*rad2rpm)*1.03 - 120); % (DIGITAL)

% LINEAR
plot(tlin,ylin*rad2rpm); % (ANALOG)
% plot(sim_omegaDL4); % (DIGITAL)

% NON-LINEAR
plot(sim_omegaA4);

% DESCRIPTIVE DATA
realpeak = max(tab.speedf)/tach.gain*rad2rpm;
plot([5.5 10.5],[realpeak realpeak],'b -.');
text(5.45,realpeak,num2str(realpeak,4),'HorizontalAlignment','right','Color','blue');

title('Motor Speed - Analog controller');
ylabel('RPM');
xlabel('Time');
legend('Reference','Real system','Linear','Non-linear');
grid on;
xlim([5.5 10.5]);
hold off;

%% Save image
fignam = 'Img\analog';
filenam = strcat(fignam,num2str(i),'.png');
filenam2 = strcat(fignam,num2str(i),'.fig');

savefig(fig,filenam2,'compact');
sdf(fig,'Latex Report');
print(fig,'-dpng',filenam);
close(fig);

