clear 
clc
%% Track Configuration
slen = 36000;                         % Maximum Track length[m]

% Varying Mass
mass_fnc = @(s, m) m/2 * (s > slen/2) + m * (s <= slen/2);                      % Varying mass assumed half from B --> A

% Road slope
relslope = 15/300;                   % Relative slope [-]
theta0 = atan(relslope);                                                    % Slope angle [rad]
L = @(s, s0) theta0/(1 + exp(-0.1*(s-s0)));                                 
theta  = @(s) -L(s, 2000) + L(s, 16000) + L(s, 20000) - L(s, 34000);             % Track topography ( A--> B downhill and B --> A uphill )

% Speed Thresholds
vmax = 120/3.6;                       % Maximum allowed speed limit [m/s]
vmin = 3.6/3.6;                     % Minimum allowed speed limit [m/s], Zero speed not allowed due to singularity issues. 
V = @(s, vm) vm + (vmin -vm)/(exp(0.1*abs(s-slen/2)));                      % Upper Speed limit: 0.05 kph at A and B
v_ub  = @(s) V(s, vmax);
vmean = 60/3.6;                      % Desired Average Speed [m/s]  
g   = 9.81;                             % Gravitational acc [m/s^2]
mass = 400000;                          % Mass of train [kg]
s = 0:slen;
m   = 400000;                % Mass of train [kg]
Af  = 10;                               % Frontal area [m^2]
rho = 1.292;                            % Density of air [kg/m^3]
cd  = 0.5;                              % Drag coefficient [-]
b = (60/3.6)^2;
cr  = 0.006;                            % Rolling res. param [-]

v = sqrt(b);                            % Function of state b
Ploss = 2.e5;
%% Battery Parameters
Emax = 2500*1000*3600;                 % Energy Capacity [60 kwh]
sig = 1.0;
OCV = @(soe) 625 + 0.1746*soe + 0.01*soe^2;                           % Nominal Voltage [V]
Req = @(soe) 0.015 - 0.05*soe;                                         % Equivalent Resistance [Ohms]
Fmax = 1.2e5;                        % Maximum Driving Force [N]
Fmin = -1.2e5;                       % Maximum Regenerative force [N]
Fp = [zeros*(1:2100) -2.e5*ones(1,16000-2101)];
bi = b;
sigi = 0.2;
cr  = 0.006;
for i = 1:15999
    if i == 2100
        temp =1;
    end
    Fa  = 0.5 * rho * Af * cd * b;          % Aerodynamic resistance [N]
    Fr  = m * g * cr * cos(theta(s(i)));
    Fg  = m * g * sin(theta(s(i)));         % Force due to gravity [N] 
    db = 2*(Fp(i) - Fr - Fa - Fg)/m;        % Acceleration [N]
    dsig = -(OCV(sig)^2 - OCV(sig)*sqrt(OCV(sig)^2 - 4*Req(sig)*(Fp(i)*v + Ploss)))/(2*Emax*v*Req(sig));
    sig = sig + dsig;
    b = b + db;
    bi(i+1) = bi(i) + db;
    sigi(i+1) = sigi(i) + dsig;
end
    

%%
figure;
plot(s(1:16000), sqrt(bi)*3.6)
yyaxis right
plot(s(1:16000), sigi)


