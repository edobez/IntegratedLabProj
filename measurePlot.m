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
filename = 'test5.txt';
test{5} = ImportTestFnc(strcat(path,filename));
test{5}.Properties.Description = 'Test 5: 2000 +- 2000 RPM (con rotore frenato)';

clear path filename;

%% Condition data
k = -0.3830; % conversione tensione-corrente

for i=1:5 
    test{i}.speedf = smooth(test{i}.speed);
    test{i}.cur = test{i}.cur * k;
    test{i}.curf = smooth(test{i}.cur,41,'sgolay');    
end

%% Plot data
close all;
tmax = 3;
tstart = 1.5;
offset = [5.402 1.5115 4.365 1.604 2.896];

for i=1:5
    rows = test{i}.time < (offset(i) - tstart + tmax) & test{i}.time >= (offset(i) - tstart);
    tab = test{i}(rows,:);
    t = tab.time - offset(i) + tstart;
    
    figure('Name',tab.Properties.Description,'NumberTitle','off');
    ha(1) = subplot(2,1,1);
    plot(t*ones(1,2),[tab.ref tab.speedf]);
    title('Set point - Tachometer output');
    ylabel('Volt');

    ha(2) = subplot(2,1,2);
    plot(t,tab.curf);
    title('Armature current');
    xlabel('Time');
    ylabel('Ampere');

    linkaxes(ha,'x');
end
return;
%% Plot comparison
plot(t*ones(1,2),[tab.ref tab.speedf]);
hold on;
plot(sim_speed);








