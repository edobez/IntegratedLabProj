%STEP_VARPAR Mesh step response for varying parameter of TF
%
%   STEP_VARPAR(F,param,values) graphs a mesh plot of the step response
%   over the values provided as input.
%
%   STEP_VARPAR(F,param,values,plottype) same as before but plottype can be
%   specified between "mesh" and "plot" (default).
%
%   Notes:
%   - TF must be a tunable transfer function

function step_varpar( F,param,values,varargin )
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
           [y(:,1),t] = step(Fsample(:,:,1,1));
            for i = 1:length(values)
                y(:,i) = step(Fsample(:,:,1,i),t);
            end
            figure;
            mesh(t,values,y');
                
        case 'plot'
            stepplot(Fsample);
    end

end

