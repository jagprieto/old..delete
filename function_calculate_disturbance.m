function disturbance = function_calculate_disturbance(time, state, PARAMETERS)
    if PARAMETERS.DISTURBANCE.TYPE == 1
        disturbance = sin(PARAMETERS.DISTURBANCE.FREQUENCY_MAX*state(1,1)) + cos(state(2,1));
    else
        disturbance = 1.65*cos(1.37*PARAMETERS.DISTURBANCE.FREQUENCY_MAX*time + 0.36)*exp(sin(2.21*PARAMETERS.DISTURBANCE.FREQUENCY_MAX*time + 0.13));  
    end
end