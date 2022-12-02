function [tau, state_gtfc_control_new] = function_control_gftc(time, reference, dot_reference, ddot_reference, state_gtfc, state_gtfc_control, INERTIA_MATRIX_INVERSE, SAMPLING_TIME)
    % Read states data
    x1 = state_gtfc(1:3); % x, y, yaw
    x2 = state_gtfc(4:6); % dot_x, dot_y, dot_yaw
    yaw = x1(3); 
    u = state_gtfc(7); 
    v = state_gtfc(8);
    r = state_gtfc(9);
    nu = 100*SAMPLING_TIME;

    % System matrices and kwon dynamics
    ROTATION_MATRIX = function_calculate_rotation_matrix(yaw);
    CONTROL_MATRIX = function_calculate_control_matrix(ROTATION_MATRIX, INERTIA_MATRIX_INVERSE);
    CONTROL_MATRIX_INVERSE = inv(CONTROL_MATRIX);
    GAMMA = function_calculate_known_dynamics(u, v, r, ROTATION_MATRIX, CONTROL_MATRIX);

%     disp('.........................');
    
    % Disturbance observer
%    K = 15;
%    L = 10;
%    gamma1 = 0.75;
%    beta1 = 0.999;
%    beta2 = 2*beta -1;
   
   disturbance = function_calculate_lumped_uncertainty(time, u, v, r);
   disturbance = -CONTROL_MATRIX*disturbance;
   
   epsilon = state_gtfc_control(1:3);
   x2_estimation = state_gtfc_control(4:6);
   tau_prev = state_gtfc_control(7:9);
  
   K1 = 0.15*eye(3);
   K1(3,3) = 8*K1(3,3);
   K2 = 20*K1;   
   z = x2 - x2_estimation;
   dot_epsilon = K2*tanh(1*z);
   epsilon = epsilon + dot_epsilon*SAMPLING_TIME;
%    dot_x2_estimation = K1*sqrt(abs(z)).*sign(z) + epsilon + CONTROL_MATRIX*tau_prev + GAMMA;
   dot_x2_estimation = K1*sqrt(abs(z)).*tanh(1*z) + epsilon + CONTROL_MATRIX*tau_prev + GAMMA;
   x2_estimation = dot_x2_estimation + dot_epsilon*SAMPLING_TIME;
   disturbance_estimation = K1*sqrt(abs(z)).*sign(z) + epsilon;
%     gain = 0;
%    disturbance_estimation = gain*disturbance + (1-gain)*disturbance_estimation;
gain = 0.8;    
disturbance_estimation(3) = gain*disturbance(3) + (1-gain)*disturbance_estimation(3);

%    phi = 5*x2; 
%    dot_phi_x2 = [5 0 0; 0 5 0; 0 0 5];
%    epsilon1 = state_gtfc_control(1);
%    epsilon2 = state_gtfc_control(2);
%    int_e = state_gtfc_control(3); 
%    int_epsilon2 = state_gtfc_control(4:7);  
%    tau_prev = state_gtfc_control(4);  
% 
%    dot_int_epsilon2 = K*sign(epsilon2) + L*epsilon2^gamma1;
%    int_epsilon2 = int_epsilon2 + dot_int_epsilon2*SAMPLING_TIME;
%    estimated_disturbance = int_e + phi + int_epsilon2;  
%    dot_int_e = -dot_phi_x2*(CONTROL_MATRIX*tau_prev + GAMMA + estimated_disturbance);
%    int_e = int_e + dot_int_e*SAMPLING_TIME;
%    
%    k1 = 10;
%    k2 = 20;
%    dot_epsilon2 = -k2*abs(epsilon1-)
%     
   % Finite time control
   k1 = 10;
   k2 = 20;
   l1 = 10;
   l2 = 10;
   gamma2 = 0.75;
   z1 = x1 - reference;
   dot_z1 = x2 - dot_reference;
   
   if abs(z1) > nu
       alfa1 = -k1*z1 + dot_reference - l1*((abs(z1)).^gamma2).*sign(z1);
       dot_alfa1 = -k1*dot_z1 + ddot_reference - l1*gamma2.*((abs(z1)).^(gamma2-1)).*sign(z1).*dot_z1;
   else
       alfa1 = -k1*z1 + dot_reference;
       dot_alfa1 = -k1*dot_z1 + ddot_reference;
   end
   z2 = x2 - alfa1;
   tau = CONTROL_MATRIX_INVERSE*(-GAMMA - disturbance_estimation + dot_alfa1 - z1 - k2*z2 - l2*((abs(z2)).^gamma2).*sign(z2));

   state_gtfc_control_new = state_gtfc_control;
   state_gtfc_control_new(1:3) = epsilon;
   state_gtfc_control_new(4:6) = x2_estimation;
   state_gtfc_control_new(7:9) = tau;
   state_gtfc_control_new(10:12) = disturbance_estimation;
 
end

