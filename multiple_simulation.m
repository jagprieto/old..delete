% Configure
configuration;

PARAMETERS.PLOT.FONT_SIZE = 16;
PARAMETERS.PLOT.CREATE_PDF = 3;
PARAMETERS.SIMULATION.NUM_SIMULATIONS = 100;
PARAMETERS.SIMULATION.SCENARIO = 1;
if PARAMETERS.SIMULATION.SCENARIO == 1
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-3;
    PARAMETERS.DISTURBANCE.TYPE = 1;
elseif PARAMETERS.SIMULATION.SCENARIO == 2
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-2;
    PARAMETERS.DISTURBANCE.TYPE = 1;
elseif PARAMETERS.SIMULATION.SCENARIO == 3
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-3;
    PARAMETERS.DISTURBANCE.TYPE = 2;
else
    PARAMETERS.SIMULATION.SAMPLING_TIME = 1e-2;
    PARAMETERS.DISTURBANCE.TYPE = 2;
end
PARAMETERS.SIMULATION.TIME = 0:PARAMETERS.SIMULATION.SAMPLING_TIME:PARAMETERS.SIMULATION.TOTAL_TIME;
PARAMETERS.SIMULATION.TOTAL_STEPS = size(PARAMETERS.SIMULATION.TIME, 2);

t_s_bound = 0.001;
% x_1_max = pi/4;
% x_2_max = 2.0*pi;
mean_steady_error_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_huijie = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
mean_steady_error_asnsta = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_asnsta = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_asnsta = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_asnsta = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);

mean_steady_error_polyakov = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
var_steady_error_polyakov = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
max_control_polyakov = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);
ts_polyakov = zeros(1,PARAMETERS.SIMULATION.NUM_SIMULATIONS);

simulations = 1:PARAMETERS.SIMULATION.NUM_SIMULATIONS;

for sim_number = 1:PARAMETERS.SIMULATION.NUM_SIMULATIONS
    sim_number
    x_1_0 = PARAMETERS.PLANT.X_1_MAX*(2*rand(1)-1);
    x_2_0 = PARAMETERS.PLANT.X_2_MAX*(2*rand(1)-1);
    PARAMETERS.PLANT.INITIAL_STATE = [x_1_0 x_2_0]';
    SIMULATION_DATA = run_simulation(PARAMETERS);
    error_asnsta = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.ASNSTA.STATE(1,:);
    error_huijie = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HUIJIE.STATE(1,:);
    error_polyakov = SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.POLYAKOV.STATE(1,:);
    from_step = ceil(PARAMETERS.SIMULATION.TOTAL_STEPS/2);
    
    polyakov_steady_error = error_polyakov(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);    
    mean_steady_error_polyakov(1,sim_number) = abs(mean(polyakov_steady_error)); 
    var_steady_error_polyakov(1,sim_number) = abs(var(polyakov_steady_error));
    max_control_polyakov(1,sim_number) = max(abs(SIMULATION_DATA.POLYAKOV.CONTROL(1,:)));
    
    huijie_steady_error = error_huijie(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    mean_steady_error_huijie(1,sim_number) = abs(mean(huijie_steady_error)); 
    var_steady_error_huijie(1,sim_number) = abs(var(huijie_steady_error));
    max_control_huijie(1,sim_number) = max(abs(SIMULATION_DATA.HUIJIE.CONTROL(1,:)));
    
    asnsta_steady_error = error_asnsta(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    mean_steady_error_asnsta(1,sim_number) = abs(mean(asnsta_steady_error));
    var_steady_error_asnsta(1,sim_number) = abs(var(asnsta_steady_error));
    max_control_asnsta(1,sim_number) = max(abs(SIMULATION_DATA.ASNSTA.CONTROL(1,:)));

    ts_polyakov_index = find(abs(error_polyakov) < t_s_bound);
    ts_polyakov(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_polyakov_index(1));
    ts_asnsta_index = find(abs(error_asnsta) < t_s_bound);
    ts_asnsta(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_asnsta_index(1));
    ts_huijie_index = find(abs(error_huijie) < t_s_bound);
    ts_huijie(1,sim_number) = PARAMETERS.SIMULATION.TIME(ts_huijie_index(1));
end

legends = {'NFDFXT', 'NSTSM', 'ASNSTA'};
% legends = {'Gonzalez-Prieto', 'Huijie', 'Polyakov'};
fig100 = figure(100);
clf(fig100);
title ('Simulation', 'FontSize',PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
subplot(4,1,1);
plot(simulations, mean_steady_error_polyakov, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, mean_steady_error_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, mean_steady_error_asnsta, '--.r', 'LineWidth', 0.5,'MarkerSize',20);

% xlabel ('Simulation', 'FontSize',PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel ('$\bar{e}$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Steady Steate Error Mean', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);  
legend(legends);

subplot(4,1,2);
plot(simulations, var_steady_error_polyakov, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, var_steady_error_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, var_steady_error_asnsta, '--.r', 'LineWidth', 0.5,'MarkerSize',20);

% xlabel ('Simulation', 'FontSize',PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel ('$\hat{Var}(\bar{e})$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Steady Steate Error Variance', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);  
% legend(legends);

subplot(4,1,3);
plot(simulations, max_control_polyakov, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, max_control_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, max_control_asnsta, '--.r', 'LineWidth', 0.5,'MarkerSize',20);

% xlabel ('Simulation', 'FontSize',PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel ('$\max{|u(t)|}$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Maximum control action', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);  
legend(legends);

subplot(4,1,4);
plot(simulations, ts_polyakov, '--.k', 'LineWidth', 0.5,'MarkerSize',20);
grid on;
hold on;
plot(simulations, ts_huijie, '--.b', 'LineWidth', 0.5,'MarkerSize',20);
plot(simulations, ts_asnsta, '--.r', 'LineWidth', 0.5,'MarkerSize',20);
% xlabel ('Simulation', 'FontSize',PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
ylabel ('$t_s$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
title('Settling time', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
xlim([1, PARAMETERS.SIMULATION.NUM_SIMULATIONS]);  
% legend(legends);

if PARAMETERS.PLOT.CREATE_PDF
    graph_file_path = strcat('../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_multiple_simulation.pdf')
    export_fig(graph_file_path, '-transparent', '-nocrop');
end