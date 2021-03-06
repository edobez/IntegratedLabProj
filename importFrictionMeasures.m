%% Import data from spreadsheet
% Script for importing data from the following spreadsheet:
%
%    Workbook: D:\Users\Edoardo\Dropbox\Integrated Design
%    Laboratory\Misurazioni corrente motore DC.xlsx Worksheet: MATLAB
%    export
%
% To extend the code for use with different selected data or a different
% spreadsheet, generate a function instead of a script.

% Auto-generated by MATLAB on 2014/11/10 19:07:33

%% Import the data
[~, ~, raw] = xlsread('C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Attrito.xlsx','Misura Tr - Mot 3','C8:F24');
% [~, ~, raw] = xlsread('C:\Users\Edoardo\Dropbox\Integrated Design Laboratory\Attrito.xlsx','Misura Tr - Mot 3','C27:F39');

%% Create output variable
data = reshape([raw{:}],size(raw));

%% Create table
misureB = table;

%% Allocate imported array to column variable names
misureB.setpoint = data(:,3);
misureB.tach = data(:,2);
misureB.rawCurrent = data(:,4);
disp(misureB)

%% Clear temporary variables
clearvars data raw;