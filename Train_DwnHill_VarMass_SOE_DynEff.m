clear;
clc;
%% Vehicle Configuration Parameters
Fmax = 3.0e4;                        % Maximum Driving Force [N]
Fmin = -3.0e4;                       % Maximum Regenerative force [N]

%% Track Configuration
slen = 2200;                         % Maximum Track length[m]

% Varying Mass
mass_fnc = @(s, m) m/2 * (s > slen/2) + m * (s <= slen/2);                      % Varying mass assumed half from B --> A

% Road slope
relslope = 15/300;                   % Relative slope [-]
theta0 = atan(relslope);                                                    % Slope angle [rad]
L = @(s, s0) theta0/(1 + exp(-0.1*(s-s0)));                                 
theta  = @(s) -L(s, 400) + L(s, 700) + L(s, 1500) - L(s, 1800);             % Track topography ( A--> B downhill and B --> A uphill )

% Speed Thresholds
vmax = 60/3.6;                       % Maximum allowed speed limit [m/s]
vmin = 3.6/3.6;                     % Minimum allowed speed limit [m/s], Zero speed not allowed due to singularity issues. 
V = @(s, vm) vm + (vmin -vm)/(exp(0.1*abs(s-slen/2)));                      % Upper Speed limit: 0.05 kph at A and B
v_ub  = @(s) V(s, vmax);
vmean = 40/3.6;                      % Desired Average Speed [m/s]  

%% Battery Parameters
Emax = 60*1000*3600;                 % Energy Capacity [60 kwh]
soe_init = 0.5;
OCV = @(soe) 625 + 0.1746*soe + 0.01*soe^2;                           % Nominal Voltage [V]
Req = @(soe) 0.15 - 0.05*soe;                                         % Equivalent Resistance [Ohms]

%% Powertrain + Gearbox + Auxiliary Losses
[Fmesh, Vmesh, Lmesh] = LoadPtlossData(Fmax);
[Z_fit, fitresult, gof] = FitEffMap(Fmesh, Vmesh, Lmesh);
PtLoss = @(v, f) fitresult.p00 + fitresult.p10*v + fitresult.p01*f + fitresult.p20*v^2 + fitresult.p11*v*f + fitresult.p02*f^2 + fitresult.p30*v^3 + ...
                     fitresult.p21*v^2*f + fitresult.p12*v*f^2 + fitresult.p40*v^4 + fitresult.p31*v^3*f + fitresult.p22*v^2*f^2 + ...
                     fitresult.p50*v^5 + fitresult.p41*v^4*f + fitresult.p32*v^3*f^2;
%% Build Optimal Control Problem
s0 = yop.independent0();                % initial state
sf = yop.independentf();                % terminal state
s  = yop.independent();
b = yop.state('scaling', 1e2);          % State, Speed Squared: b(s) = v^2(s) [m/s]^2
sig = yop.state('scaling', 1);          % State, SOE: sig(s) [-]
Fp = yop.control('scaling', 1e5);       % Control input, Propulsive(Tractive/Regenerative) force: Fp [N]
x = [b;sig];
[f, y] = Train(s, b, sig, Fp, theta, mass_fnc, PtLoss, OCV, Req, Emax);
vtob = @(v) (v)^2;                      % Speed to b(s) or v^2(s), input v is in kph

%% Optimal Control Problem Definition
tf = int(1/y.v);                        % Time to Travel
t_mean = sf/(vmean);                % Desired max Time to travel

ocp = yop.ocp();
ocp.min(-sig(sf));
ocp.st(...
    s0 == 0, sf == slen, ...
    der(x) == f, ...
    b(s0) == vtob(vmin), ...
    b(sf) == vtob(vmin), ...
    vtob(vmin) <= b(s) <= vtob(v_ub(s)), ...
    0.0 <= sig(s) <= 1.0, ...
    tf <= t_mean, ...
    Fmin <= Fp <= Fmax, ...
    sig(s0) == soe_init ...
    );

sol = ocp.solve('ival', 400, 'dx', 2);

Plot_DwnHill_Strategy(s, Fp, theta, slen, v_ub, sol, y, vmin, Fmin, Fmax, sig, Emax, soe_init, Fmesh, Vmesh, Lmesh, Z_fit, fitresult, OCV, Req);

%% Train State Space Dynamic model
function [dx, y] = Train(s, b, sig, Fp, theta, mass_fnc, PtLoss, OCV, Req, Emax)
g   = 9.81;                             % Gravitational acc [m/s^2]
mass = 40000;                           % Mass of train [kg]
m   = mass_fnc(s, mass);                % Mass of train [kg]
Af  = 10;                               % Frontal area [m^2]
rho = 1.292;                            % Density of air [kg/m^3]
cd  = 0.5;                              % Drag coefficient [-]
cr = 0.006;                             % Rolling coefficient [-]
v = sqrt(b);                            % Function of state b
Ploss = PtLoss;

Fr  = m(s) * g * cr * cos(theta(s));    % Rolling resistance [N]
Fa  = 0.5 * rho * Af * cd * b;          % Aerodynamic resistance [N]
Fg  = m(s) * g * sin(theta(s));         % Force due to gravity [N] 
db = 2*(Fp - Fr - Fa - Fg)/m(s);        % Acceleration [N]
dsig = -(OCV(sig)^2 - OCV(sig)*sqrt(OCV(sig)^2 - 4*Req(sig)*(Fp*v + Ploss(v, Fp))))/(2*Emax*v*Req(sig));

dx = [db; dsig];                         % State: [b(s); sig(s)]
Pe = v*Fp;                              % Vehicle power [W]
y.v = v;
y.m = m(s);
y.Pe = Pe;
end