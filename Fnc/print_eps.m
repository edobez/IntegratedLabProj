function [ output_args ] = print_eps( handle, name )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% options = { 'Units','centimeters',...
%             'Position',[0 0 5 5],...
%             'PaperPositionMode','auto'};
%         
set(handle,'Units','centimeters',...
            'Position',[0 0 15 15],...
            'PaperPositionMode','auto',...
            'FontSize',9,...
            'FontName','Times');

print(handle,'-depsc2','myplot.eps')

end
