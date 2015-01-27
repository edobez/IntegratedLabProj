clear test*;

%% Import from files 
pathA = 'C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Misure\analog\';
pathD = 'C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Misure\digital\';

% Test 1: 2000 +- 500 RPM
filename = 'test1.txt';
testA{1} = ImportTestFnc(strcat(pathA,filename));
testA{1}.Properties.Description = 'mesTest1A';
testD{1} = ImportTestFncD(strcat(pathD,filename));
testD{1}.Properties.Description = 'mesTest1D';

% Test 2: 2000 +- 1500 RPM
filename = 'test2.txt';
testA{2} = ImportTestFnc(strcat(pathA,filename));
testA{2}.Properties.Description = 'mesTest2A';
testD{2} = ImportTestFncD(strcat(pathD,filename));
testD{2}.Properties.Description = 'mesTest2D';

% Test 3: 2000 +- 2000 RPM
filename = 'test3.txt';
testA{3} = ImportTestFnc(strcat(pathA,filename));
testA{3}.Properties.Description = 'mesTest3A';
testD{3} = ImportTestFncD(strcat(pathD,filename));
testD{3}.Properties.Description = 'mesTest3D';

% Test 4: 0 +- 500 RPM
filename = 'test4.txt';
testA{4} = ImportTestFnc(strcat(pathA,filename));
testA{4}.Properties.Description = 'mesTest4A';
testD{4} = ImportTestFncD(strcat(pathD,filename));
testD{4}.Properties.Description = 'mesTest4D';

clear path filename;

%% Condition data
k = -0.3830; % conversione tensione-corrente

for i=1:4
    testA{i}.reff = smooth(testA{i}.ref,11);
    testA{i}.speedf = smooth(testA{i}.w,51);
    testD{i}.speedf = smooth(testD{i}.w,51);
end

%% Plot data
close all;
offsetA = [1.8935 0.7635 1.8105 1.187]; % Istante in cui parte lo step per ciascun test (ANALOG)
offsetD = [1.6785 1.9252 2.0835 1.9345]; % (DIGITAL)
twidth = 15; % Finestra temporale del grafico
stepstart = 1; % Instante in cui far partire lo step
dec = 5; % Decimazione

for i=1
    rowsA = testA{i}.time < (offsetA(i) - stepstart + twidth) & testA{i}.time >= (offsetA(i) - stepstart);
    tabA = testA{i}(rowsA,:);
    tA = tabA.time(1:dec:end) - offsetA(i) + stepstart;
    
    rowsD = testD{i}.time < (offsetD(i) - stepstart + twidth) & testD{i}.time >= (offsetD(i) - stepstart);
    tabD = testD{i}(rowsD,:);
    tD = tabD.time(1:dec:end) - offsetD(i) + stepstart;
    
    figure('Name',tabA.Properties.Description,'NumberTitle','off');
    hold on;
    plot(tA,tabA.reff(1:dec:end)/tach.gain*rad2rpm);
    plot(tA,tabA.speedf(1:dec:end)/tach.gain*rad2rpm);
    plot(tD,tabD.speedf(1:dec:end)/tach.gain*rad2rpm*1.02-110);  
    hold off;
    title('Motor Speed');
    ylabel('RPM');
    legend('Reference','Analog system','Digital system');

end
return;