% Configure
configuration;

PARAMETERS.PLOT.CREATE_PDF = 0;
PARAMETERS.SIMULATION.SCENARIO = 1;
if PARAMETERS.SIMULATION.SCENARIO == 2
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

% Run simulation
SIMULATION_DATA = run_simulation(PARAMETERS);

% Plot simulation
plot_simulation(SIMULATION_DATA, PARAMETERS);

% % Check basic statistics
% max_control_huejie = max(abs(SIMULATION_DATA.HUIJIE.CONTROL(1,:)))
% max_control_asnsta = max(abs(SIMULATION_DATA.ASNSTA.CONTROL(1,:)))
% max_x1_huejie = max(abs(SIMULATION_DATA.HUIJIE.STATE(1,:)))
% max_x1_asnsta = max(abs(SIMULATION_DATA.ASNSTA.STATE(1,:)))
% max_x2_huejie = max(abs(SIMULATION_DATA.HUIJIE.STATE(2,:)))
% max_x2_asnsta = max(abs(SIMULATION_DATA.ASNSTA.STATE(2,:)))
% 
% from_step = ceil(PARAMETERS.SIMULATION.TOTAL_STEPS/2);
% huijie_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.HUIJIE.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
% asnsta_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.ASNSTA.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
% 
% mean_steady_error_huejie = mean(huijie_steady_error)
% mean_steady_error_asnsta = mean(asnsta_steady_error)
% var_steady_error_huejie = var(huijie_steady_error)
% var_steady_error_asnsta = var(asnsta_steady_error)
