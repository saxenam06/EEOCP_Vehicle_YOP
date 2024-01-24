function [Z_fit, fitresult, gof] = FitEffMap(Fmesh, Vmesh, Lmesh)
    
    % Reshape matrices into column vectors for fitting
    xData = Vmesh(:);
    yData = Fmesh(:);
    zData = Lmesh(:);
    % Fit Power loss
    fitType = 'poly52';
    [fitresult, gof] = fit([xData, yData], zData, fitType);
    
    % Evaluate the fit on the same meshgrid
    Z_fit = feval(fitresult, Vmesh, Fmesh);