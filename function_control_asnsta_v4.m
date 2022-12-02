function [control, control_state_new] = function_control_asnsta_v4(time, reference, state, control_state, PARAMETERS)
%     disp('.....................................................')    
    % Read states 
    x_1 = state(1,1);
    x_2 = state(2,1);
    x_1_r = reference(1,1);
    x_2_r = reference(2,1);
    dot_x_2_r = reference(3,1);

    %%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    epsilon = tanh(1.0);
    zeta = 2.0;
    b = (zeta - 1.0) / zeta;
    scale_factor = PARAMETERS.CONTROL.SCALE_FACTOR; 
    t_z = PARAMETERS.SIMULATION.SETTLING_TIME;
    t_s_e = scale_factor*t_z;
    t_s_s = scale_factor*t_s_e;
    t_sigma = 2*PARAMETERS.SIMULATION.SAMPLING_TIME;
    rho_e = PARAMETERS.CONTROL.ERROR_PRECISION + 2*PARAMETERS.SIMULATION.SAMPLING_TIME;
    rho_z_max = rho_e/scale_factor;
    rho_z_min = scale_factor*rho_e;
    rho_s_max = scale_factor*rho_z_max;
    rho_s_min = scale_factor*rho_z_min; 
    rho_sigma = scale_factor*rho_s_min;   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
    %%%%%%%%%%%%%%%%%%%%%%%% READ CONTROL STATE %%%%%%%%%%%%%%%%%%%%%%
    if time < PARAMETERS.SIMULATION.SAMPLING_TIME
        d_estimate = 0;
        x_1_c = x_1;
        x_2_estimate = x_2;
        z = x_1_r - x_1_c;
        z_0 = max(rho_z_max/(scale_factor*scale_factor), abs(z));
        v_z_0 = (1.0/zeta)*(abs(z_0)^zeta);
    else
        x_1_c = control_state(1); 
        d_estimate = control_state(3); 
        x_2_estimate = control_state(4); 
        v_z_0 = control_state(9);
    end

    %%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%
    sigma_0 = rho_sigma/(scale_factor*scale_factor);
    v_sigma_0 = (1.0/zeta)*(abs(sigma_0)^zeta);
    e_0 = rho_e/(scale_factor*scale_factor);
    v_e_0 = (1.0/zeta)*(abs(e_0)^zeta);
    c_e_2 = (log(2)*(v_e_0^(1-b)))/(t_s_e*(1-b));
    c_e_1 = c_e_2 / (v_e_0^(1-b));
    lambda_e = c_e_1/(zeta);
    kappa_e = (c_e_2/(epsilon*zeta^b));
    gamma_e = min(atanh(epsilon)/rho_e, ((1.0/PARAMETERS.SIMULATION.SAMPLING_TIME) - lambda_e) / kappa_e);
    c_z_2 = (log(2)*(v_z_0^(1-b)))/(t_z*(1-b));
    c_z_1 = c_z_2 / (v_z_0^(1-b));
    lambda_z = c_z_1/(zeta);    
    c_sigma_2 = (log(2)*(v_sigma_0^(1-b)))/(t_sigma*(1-b));
    c_sigma_1 = c_sigma_2 / (v_sigma_0^(1-b));
    lambda_sigma = c_sigma_1/(zeta);
    alfa_sigma = lambda_sigma*lambda_sigma/4.0;
    p1 = (alfa_sigma+1)/(2*lambda_sigma);
    p2 = (lambda_sigma^2 +  alfa_sigma + 1) / (2*lambda_sigma*alfa_sigma);
    p3 = (4*p2*p1-1.0)/4*p2;
    p4 = 4*(lambda_sigma^3 + lambda_sigma)/(5*lambda_sigma^2+4.0);       

    %%%%%%%%%%%%%%%%%%%%%%%% DYNAMICS FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%
    % Known functions
    f = function_calculate_f(time, state, PARAMETERS);
    g = function_calculate_g(time, state, PARAMETERS);
 
    %%%%%%%%%%%%%%%%%%%%%%%% STATE ERROR %%%%%%%%%%%%%%%%%%%%%%
    % Virtual control error
    z = x_1_r - x_1_c;

    % Adaptive parameters
    kappa_z = c_z_2/(epsilon*zeta^b) + (abs(z)/epsilon);
    rho_z = (rho_z_max - rho_z_min)*(1.0 - exp(-abs(z))) + rho_z_min; % Adaptive value of rho_z
    gamma_z = min(atanh(epsilon)/rho_z, ((1.0/PARAMETERS.SIMULATION.SAMPLING_TIME) - lambda_z) / kappa_z);

    % Virtual control state
    x_2_c = lambda_z*z + kappa_z*tanh(gamma_z*z) + x_2_r; 
    dot_z = x_2_r - x_2_c;
    dot_gamma_z = -atanh(epsilon)*(rho_z_max - rho_z_min)*(z*dot_z*exp(-abs(z))) / rho_z^2;
    dot_x_2_c = lambda_z*dot_z + kappa_z*gamma_z*((sech(gamma_z*z))^2)*dot_z + dot_x_2_r + kappa_z*((sech(gamma_z*z))^2)*dot_gamma_z*z;   
    
    % Tracking error
    e = x_1_c - x_1; 
    dot_e = x_2_c - x_2;  
    
    %%%%%%%%%%%%%%%%%%%%%%% DISTURBANCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%
    % Velocity estimation
    sigma = x_2 - x_2_estimate;

    % Adaptive parameters
    rho_omega = sqrt(PARAMETERS.DISTURBANCE.FREQUENCY_MAX*rho_sigma) + 2*PARAMETERS.DISTURBANCE.FREQUENCY_MAX*p2;
    if abs(sigma) > rho_sigma
        c_sigma_3 = rho_omega*(1.0+sqrt(abs(sigma)/rho_sigma));
    else
        c_sigma_3 = 2*rho_omega;
    end
    kappa_sigma = c_sigma_2/(epsilon*zeta^b) + (c_sigma_3)/(epsilon); 
    gamma_sigma = min(atanh(epsilon)/rho_sigma, ((1.0/PARAMETERS.SIMULATION.SAMPLING_TIME) - lambda_sigma) / kappa_sigma);
    upsilon_1 = PARAMETERS.DISTURBANCE.FREQUENCY_MAX / (2*p4*rho_sigma);
    upsilon_2 = max(0.0, ((2.0*(PARAMETERS.DISTURBANCE.FREQUENCY_MAX*p2)^2)/(kappa_sigma*epsilon*rho_sigma)) - p3);
    upsilon = max(upsilon_1, upsilon_2);
    sum_sigma = 0.5 + ((p3+upsilon)*kappa_sigma*epsilon/rho_z) + p4*upsilon;
    mu_sigma = (PARAMETERS.DISTURBANCE.FREQUENCY_MAX/(4*sum_sigma))*(-1.0 + sqrt(1 + 32*sum_sigma*p2*p2));
    if time < PARAMETERS.SIMULATION.SAMPLING_TIME  
        beta_sigma = kappa_sigma;
    else
        beta_sigma = control_state(11);
    end
    c_sigma = ((kappa_sigma*gamma_sigma*lambda_sigma*rho_sigma) / (2*PARAMETERS.DISTURBANCE.FREQUENCY_MAX)) + 1.0;
    delta_sigma = max(abs(c_sigma/(beta_sigma*rho_sigma)), ((1.0/PARAMETERS.SIMULATION.SAMPLING_TIME^2) - alfa_sigma) / beta_sigma);
    if abs(sigma) > mu_sigma
        denominador = 2*p2*tanh(delta_sigma*abs(sigma));
    else
        denominador = 2*p2*tanh(delta_sigma*mu_sigma);
    end
    numerador = kappa_sigma*tanh(gamma_sigma*abs(sigma)) + 2*upsilon*abs(sigma);
    beta_sigma = numerador / denominador; 
   
    %%%%%%%%%%%%%%%%%%%%%%%% SLIDING SURFACE %%%%%%%%%%%%%%%%%%%%%%
    % Sliding surface
    s = dot_e + lambda_e*e + kappa_e*tanh(gamma_e*e);    
    if time < PARAMETERS.SIMULATION.SAMPLING_TIME
        s_0 = max(rho_s_max/(scale_factor*scale_factor), abs(dot_e));
        v_s_0 = (1.0/zeta)*(abs(s_0)^zeta);
        s_c = s_0;
    else
        v_s_0 = control_state(10);
        s_c = control_state(2);
    end

    % Adaptive parameters
    c_s_2 = (log(2)*(v_s_0^(1-b)))/(t_s_s*(1-b));
    c_s_1 = c_s_2 / (v_s_0^(1-b));
    lambda_s = c_s_1/zeta;
    pi_gain = c_e_1 / c_s_1;
    c_s_3 = 1/pi_gain;
    rho_s = (rho_s_max - rho_s_min)*(1.0 - exp(-abs(s))) + rho_s_min;    
    kappa_s = c_s_2/(epsilon*zeta^b)  + (c_s_3*abs(e)/epsilon) + (c_sigma_3)/(epsilon); 
    gamma_s = min(atanh(epsilon)/rho_s, ((1.0/PARAMETERS.SIMULATION.SAMPLING_TIME) - lambda_s) / kappa_s);
    dot_s_c = -lambda_s*s_c - kappa_s*tanh(gamma_s*s_c);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%% CONTROL %%%%%%%%%%%%%%%%%%%%%%
    control = (1/g)*(dot_x_2_c - f  + dot_e*(lambda_e + kappa_e*gamma_e*((sech(gamma_e*e))^2)) + lambda_s*s + kappa_s*tanh(gamma_s*s) - d_estimate);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%% DISTURBANCE OBSERVER %%%%%%%%%%%%%%%%%%%%%%
    dot_x_2_estimate = f + g*control + lambda_sigma*sigma + kappa_sigma*tanh(gamma_sigma*sigma) + d_estimate;    
    x_2_estimate = x_2_estimate + dot_x_2_estimate*PARAMETERS.SIMULATION.SAMPLING_TIME;       
    dot_d_estimate = alfa_sigma*sigma + beta_sigma*tanh(delta_sigma*sigma);    
    d_estimate = d_estimate + dot_d_estimate*PARAMETERS.SIMULATION.SAMPLING_TIME; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Save control states
    control_state_new = control_state; 
    control_state_new(1) = x_1_c + x_2_c*PARAMETERS.SIMULATION.SAMPLING_TIME; 
    control_state_new(2) = s_c + dot_s_c*PARAMETERS.SIMULATION.SAMPLING_TIME; 
    control_state_new(3) = d_estimate;
    control_state_new(4) = x_2_estimate;  
    control_state_new(5) = s;
    control_state_new(6) = e;
    control_state_new(7) = z;
    control_state_new(8) = sigma;
    control_state_new(9) = v_z_0;
    control_state_new(10) = v_s_0;
    control_state_new(11) = beta_sigma;
end

