function Plot_DwnHill_Strategy(s, Fp, theta, slen, v_ub, sol, y, vmin, Fmin, Fmax, sig, Emax, soe_init, Fmesh, Vmesh, Lmesh, Z_fit, fitresult, OCV, Req)
%%
s_values = 0: slen;
altitude = cumsum(tan(arrayfun(theta, s_values)));
PlossOper = feval(fitresult, sqrt(sol.sol_x(1,:)), sol.sol_u(1,:));
OCV_sig = arrayfun(OCV, sol.sol_x(2,:));
Req_sig = arrayfun(Req, sol.sol_x(2,:));

Fcns = -(OCV_sig.^2 - OCV_sig.*sqrt(OCV_sig.^2 - 4*Req_sig.*(sol.sol_u(1,:).*sqrt(sol.sol_x(1,:)) + PlossOper)))./(2*sqrt(sol.sol_x(1,:)).*Req_sig);
EgyCns   = Emax*soe_init + cumsum((Fcns.*[0 diff(sol.value(s))]));
sig_val = EgyCns./Emax;

figure(1);
set(gcf, 'Position', [100, 100, 2000, 1600]);
subplot(221); 
plot(s_values, arrayfun(theta, s_values), 'DisplayName', 'Slope');
hold on
ylabel('\theta [rad] ');
yyaxis right
plot(s_values, altitude, 'DisplayName', 'Height');
xline(0, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
xline(slen/2, '--k', 'DisplayName', 'Port', 'LineWidth', 1.5);
xline(slen, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
legend('show');
title('Slope angle/Height');
xlabel('s [m]');
ylabel('h [m] ');
set(gca, 'FontSize', 12);

subplot(222); 
hold on
sol.plot(s, y.v*3.6, 'DisplayName', 'Vehicle Speed');
plot(s_values, vmin*ones(1,length(s_values))*3.6, '--', 'Color', 'r', 'DisplayName', 'Speed Lower limit');
plot(s_values, arrayfun(v_ub, s_values)*3.6, '--', 'Color', 'b', 'DisplayName', 'Speed Upper limit');
xline(0, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
xline(slen/2, '--k', 'DisplayName', 'Port', 'LineWidth', 1.5);
xline(slen, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
legend('show');
title('Vehicle speed');
xlabel('s [m]');
ylabel('v [kph]');
set(gca, 'FontSize', 12);

subplot(223);
plot(s_values, Fmin*ones(1,length(s_values)), '--', 'Color', 'r', 'DisplayName', 'Regenerative Force limit');
hold on;
plot(s_values, Fmax*ones(1,length(s_values)), '--', 'Color', 'b', 'DisplayName', 'Driving Force limit');
sol.stairs(s, Fp, 'DisplayName', 'Longitudinal Force');
xline(0, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
xline(slen/2, '--k', 'DisplayName', 'Port', 'LineWidth', 1.5);
xline(slen, '--c', 'DisplayName', 'Mine Site', 'LineWidth', 1.5);
legend('show');
title('Longitudinal Force');
set(gca, 'FontSize', 12);


subplot(224); 
plot(sol.sol_t, EgyCns/(1000*3600), 'DisplayName', 'Energy Consumption');
ylabel('Energy [kwh]');
hold on
xline(slen/2, '--');
yyaxis right
sol.stairs(s, sig, 'DisplayName', 'SOE');
% plot(sol.sol_t, sig_val, 'DisplayName', 'SOE_plant');
ylabel('SOE [-]');
title('Energy state');
xlabel('s [m]');
set(gca, 'FontSize', 12);
print -dpng 'train_Driving_Strategy'

figure(2);
set(gcf, 'Position', [100, 100, 2000, 1600]);
scatter3(Vmesh(:)*3.6, Fmesh(:), Lmesh(:), 'filled', 'DisplayName', 'given Data');
hold on;
mesh(Vmesh*3.6, Fmesh, Z_fit, 'DisplayName', 'Fitted Map');
scatter3(sqrt(sol.sol_x(1,:))*3.6, sol.sol_u, PlossOper, '.k', 'DisplayName', 'Chosen Operating Point');
colorbar;
colormap summer
ylabel('Propulisve Force [N]');
xlabel('Speed [kph]');
zlabel('Losses');
title('Fitted Map and Given Data');
legend('show');
set(gca, 'FontSize', 20);
print -dpng 'Fitted_Map_Given_Data'

% Caluclate efficiency
eff_drv = (Fmesh.*Vmesh)./((Fmesh.*Vmesh) + Z_fit);
eff_coast = ((Fmesh.*Vmesh) + Z_fit)./(Fmesh.*Vmesh);
eff = [eff_coast(1:5, :) ; eff_drv(6:10,:)];
figure(3);
set(gcf, 'Position', [100, 100, 2000, 1600]);
mesh(Vmesh*3.6, Fmesh, eff, 'DisplayName', 'Efficiency Map')
hold on;
scatter3(sqrt(sol.sol_x(1,:))*3.6, sol.sol_u, 1, '.k','DisplayName', 'Chosen operation Point')
colorbar;
colormap hot
view(2)
xlabel('Speed [kph]')
ylabel('Propulsive force [N]')
title('Powertrain Efficiency Map')
set(gca, 'FontSize', 20);
print -dpng 'Powertrain_Efficiency_Map'

figure(4);
set(gcf, 'Position', [100, 100, 2000/4, 1600/4]);
OCV_Vs_soe = arrayfun(OCV, 0:0.05:1);
Req_Vs_soe = arrayfun(Req, 0:0.05:1);
plot(0:0.05:1, OCV_Vs_soe, 'DisplayName', 'OCV Vs SOE');
ylabel('OCV [V]');
hold on
yyaxis right
plot(0:0.05:1, Req_Vs_soe, 'DisplayName', 'Req Vs SOE');
ylabel('Req [Ohms]');
title('Battery Model');
xlabel('SOE [-]');
set(gca, 'FontSize', 12);
print -dpng 'Battery_SOE'

disp(sprintf('Total Energy Consumption %d kwh', EgyCns(end)-EgyCns(1)))
