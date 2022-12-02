function [SIMULATION_DATA, PARAMETERS] = run_simulation(PARAMETERS)
     
    % Prepare simulation data
    SIMULATION_DATA = {};
    SIMULATION_DATA.REFERENCE = zeros(3, PARAMETERS.SIMULATION.TOTAL_STEPS); % x1d,x2d, dot_x2d
    SIMULATION_DATA.TIME = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);
    
    % HUIJIE
    SIMULATION_DATA.HUIJIE.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.HUIJIE.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    huijie_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.HUIJIE.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    huijie_control_state = zeros(20,1);
    SIMULATION_DATA.HUIJIE.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);
       
    % POLYAKOV
    SIMULATION_DATA.POLYAKOV.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.POLYAKOV.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    polyakov_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.POLYAKOV.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    polyakov_control_state = zeros(20,1);
    SIMULATION_DATA.POLYAKOV.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);

    % ASNSTA
    SIMULATION_DATA.ASNSTA.DISTURBANCE = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    SIMULATION_DATA.ASNSTA.STATE = zeros(2, PARAMETERS.SIMULATION.TOTAL_STEPS); 
    asnsta_state = PARAMETERS.PLANT.INITIAL_STATE; 
    SIMULATION_DATA.ASNSTA.CONTROL_STATE = zeros(20, PARAMETERS.SIMULATION.TOTAL_STEPS);
    asnsta_control_state = zeros(20,1);
    SIMULATION_DATA.ASNSTA.CONTROL = zeros(1, PARAMETERS.SIMULATION.TOTAL_STEPS);

    % Run simulation
    simulation_time = 0.0;
   
    for simulation_step = 1:PARAMETERS.SIMULATION.TOTAL_STEPS
        [reference] = function_calculate_reference(simulation_time, PARAMETERS);

        % Save data
        SIMULATION_DATA.TIME (:, simulation_step) = simulation_time;
        SIMULATION_DATA.REFERENCE(:, simulation_step) = reference;
        SIMULATION_DATA.HUIJIE.STATE(:, simulation_step) = huijie_state;
        SIMULATION_DATA.POLYAKOV.STATE(:, simulation_step) = polyakov_state;
        SIMULATION_DATA.ASNSTA.STATE(:, simulation_step) = asnsta_state;

        % ASNSTA
        [asnsta_control, asnsta_control_state] = function_control_asnsta_v4(simulation_time, reference, asnsta_state, asnsta_control_state, PARAMETERS);
        asnsta_disturbance = function_calculate_disturbance(simulation_time, asnsta_state, PARAMETERS);    
        [asnsta_state] = function_calculate_system_dynamics(simulation_time, asnsta_state, asnsta_control, asnsta_disturbance, PARAMETERS);
        SIMULATION_DATA.ASNSTA.CONTROL(:, simulation_step) = asnsta_control;
        SIMULATION_DATA.ASNSTA.CONTROL_STATE(:, simulation_step) = asnsta_control_state;
        SIMULATION_DATA.ASNSTA.DISTURBANCE(:, simulation_step) = asnsta_disturbance;
        
       
        % POLYAKOV
        [polyakov_control, polyakov_control_state] = function_control_polyakov(simulation_time, reference, polyakov_state, polyakov_control_state, PARAMETERS);
        polyakov_disturbance = function_calculate_disturbance(simulation_time, polyakov_state, PARAMETERS);    
        [polyakov_state] = function_calculate_system_dynamics(simulation_time, polyakov_state, polyakov_control, polyakov_disturbance, PARAMETERS);
        SIMULATION_DATA.POLYAKOV.CONTROL(:, simulation_step) = polyakov_control;
        SIMULATION_DATA.POLYAKOV.CONTROL_STATE(:, simulation_step) = polyakov_control_state;
        SIMULATION_DATA.POLYAKOV.DISTURBANCE(:, simulation_step) = polyakov_disturbance;

         % HUIJIE
        [huijie_control, huijie_control_state] = function_control_huijie(simulation_time, reference, huijie_state, huijie_control_state, PARAMETERS);
        huijie_disturbance = function_calculate_disturbance(simulation_time, huijie_state, PARAMETERS);    
        [huijie_state] = function_calculate_system_dynamics(simulation_time, huijie_state, huijie_control, huijie_disturbance, PARAMETERS);
        SIMULATION_DATA.HUIJIE.CONTROL(:, simulation_step) = huijie_control;
        SIMULATION_DATA.HUIJIE.CONTROL_STATE(:, simulation_step) = huijie_control_state;
        SIMULATION_DATA.HUIJIE.DISTURBANCE(:, simulation_step) = huijie_disturbance;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update time
        simulation_time = simulation_time + PARAMETERS.SIMULATION.SAMPLING_TIME;
    end
end
        