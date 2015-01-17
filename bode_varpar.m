%BODE_VARPAR Mesh Bode plot for varying parameter of TF
%
%   BODE_VARPAR(F,param,values) graphs a mesh plot of the Bode plot
%   over the values provided as input.
%
%   BODE_VARPAR(F,param,values,plottype) same as before but plottype can be
%   specified between "mesh" and "plot" (default).
%
%   Notes:
%   - TF must be a tunable transfer function

function bode_varpar( F,param,values,varargin )
    minargs = 3;
    maxargs = 4;
    narginchk(minargs,maxargs);
    nVarargs = length(varargin);

    assert(~isempty(F),'F must not be empty');
    assert(ischar(param),'param must be a string cointaining the name of tunable parameter');
    assert(length(values) > 0,'values must be an array of values for the tunlable parameter');
    
    Fsample = replaceBlock(F,param,values);
    Fsample.SamplingGrid = struct(param,values);
    
    opt = 'plot';
    if nVarargs == 1
        opt = validatestring(varargin{1},{'mesh','plot'});
    end
    
    switch opt
        case 'mesh'
           [mag(:,1),~,wout] = bode(Fsample(:,:,1,1));
            for i = 1:length(values)
                mag(:,i) = bode(Fsample(:,:,1,i),wout);
            end
            
            magdb = 20*log10(mag);
            woutlog = log10(wout);
            
            figure;
            mesh(woutlog,values,magdb');
                
        case 'plot'
            bodeplot(Fsample);
    end

end

