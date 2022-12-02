function [control, control_state_new] = function_control_huijie(time, reference, state, control_state, PARAMETERS)
    % Parameters
%     L1 = 3;
%     L2 = 1.5;
%     alfa = 0.8;
%     beta = 1.2;
%     eta = 1.0e-3;

    alfa1 = 2.0;
    alfa2 = 2.0;
    alfa3 = 2.0;    
    gamma1 = 9.0/5.0;
    gamma2 = 7.0/9.0;
    epsilon = 0.05;
    k1 = 1.0;
    k2 = 1.0;
    k3 = 1.0;
    K = 2.0;
    delta = 10e-4;
   
    % Read states and create errors
    e1 = state(1,1) - reference(1,1);
    e2 = state(2,1) - reference(2,1);

    % Sliding surface
    [h_e1, doth_e1] = function_huijie_h(e1, e2, gamma2, epsilon);
    s = e2 + alfa1*function_fractional_power(e1, gamma1) + alfa2*e1 + alfa3*h_e1;
    chi = function_huijie_chi(s, delta);
    huijie_disturbance = 1*function_calculate_disturbance(time, state, PARAMETERS); 
    f = function_calculate_f(time, state, PARAMETERS);
    g = function_calculate_g(time, state, PARAMETERS);
    control = (-1/g)*(f - reference(3,1) + (alfa1*gamma1*abs(e1)^(gamma1-1) + alfa2)*e2 + alfa3*doth_e1 ...
        + k1*function_fractional_power(s, alfa1) + k2*s  + k3*function_fractional_power(s, alfa2) + huijie_disturbance + K*chi);

    control_state_new = control_state;
    control_state_new(5) = huijie_disturbance;
end

