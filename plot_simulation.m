
% Plot simulation 
function plot_simulation(SIMULATION_DATA, PARAMETERS)
%     legends = {'Polyakov', 'Huijie', 'Gonzalez-Prieto'};
    legends = {'NFDFXT', 'NSTSM', 'ASNSTA'};

    % Plot  errors
    fig1 = figure(1);
    clf(fig1);
    subplot(2,1,1);
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.POLYAKOV.STATE(1,:) ,'--', 'Color', 'b', 'LineWidth', 2.0);
    grid on;
    hold on;
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.HUIJIE.STATE(1,:) ,':', 'Color', 'k', 'LineWidth', 2.0);
    plot(SIMULATION_DATA.TIME(1,:), SIMULATION_DATA.REFERENCE(1,:) - SIMULATION_DATA.ASNSTA.STATE(1,:),'-', 'Color', 'r', 'LineWidth', 2.0);
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('$e_x(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Position error: $e_x(t) = x^d_1(t)-x_1(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
    legend(legends);    
    subplot(2,1,2);
    plot(SIMULATION_DATA.TIME(1,:),SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.POLYAKOV.STATE(2,:) ,'--', 'Color', 'b', 'LineWidth', 2.0);
    grid on;
    hold on;
    plot(SIMULATION_DATA.TIME(1,:),SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.HUIJIE.STATE(2,:) ,':', 'Color', 'k', 'LineWidth', 2.0);
    plot(SIMULATION_DATA.TIME(1,:),  SIMULATION_DATA.REFERENCE(2,:) - SIMULATION_DATA.ASNSTA.STATE(2,:) ,'-', 'Color', 'r', 'LineWidth', 2.0);
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('$\dot{e}_{x}(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Velocity error: $\dot{e}_{x} = x^d_2(t)-x_2(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  

    if PARAMETERS.PLOT.CREATE_PDF
        graph_file_path = strcat('../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_errors.pdf')
        export_fig(graph_file_path, '-transparent', '-nocrop');
    end
    
    % Plot control and steady state error
    fig2 = figure(2);
    clf(fig2);
    subplot(2,1,1);
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.POLYAKOV.CONTROL(1,:),'--', 'Color', 'b', 'LineWidth', 2.0); 
    grid on;
    hold on; 
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.HUIJIE.CONTROL(1,:),':', 'Color', 'k', 'LineWidth', 2.0); 
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.ASNSTA.CONTROL(1,:),'-', 'Color', 'r', 'LineWidth', 2.0);    
    xlabel('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('u(t)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Control effort: u(t)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
    legend(legends);   
%     subplot(2,1,2);
    
    from_step = ceil(PARAMETERS.SIMULATION.TOTAL_STEPS/2);
    total_points = PARAMETERS.SIMULATION.TOTAL_STEPS - from_step;
    polyakov_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.POLYAKOV.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    huijie_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.HUIJIE.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    asnsta_steady_error = SIMULATION_DATA.REFERENCE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS) - SIMULATION_DATA.ASNSTA.STATE(1,from_step:PARAMETERS.SIMULATION.TOTAL_STEPS);
    subplot(2,3,4);
    histogram(asnsta_steady_error, 50, 'FaceColor', 'r', 'Normalization','probability');
    grid on;
    hold on;
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    subplot(2,3,5);
    histogram(huijie_steady_error, 50, 'FaceColor', 'k', 'Normalization','probability');%, 'FaceAlpha', 0.5);
    grid on;
    hold on;
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    subplot(2,3,6);
    histogram(polyakov_steady_error, 50, 'FaceColor', 'b', 'Normalization','probability');%, 'FaceAlpha', 0.25);
    grid on;
    hold on;
%     title('Steady state error distribution', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlabel ('Steady state error', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('Frequency', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
%     legend(legends);    
    if PARAMETERS.PLOT.CREATE_PDF
        set(gcf, 'color', 'white') 
        set(gca, 'color','white');
        graph_file_path = strcat('../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_control_steady_state_error.pdf')
        export_fig(graph_file_path, '-nocrop'); %, '-transparent'
    end

    % Plot error variables
    fig3 = figure(3);
    clf(fig3);
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.ASNSTA.CONTROL_STATE(5,:),'-', 'Color', 'r', 'LineWidth', 2.0);  
    grid on;
    hold on;
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.ASNSTA.CONTROL_STATE(6,:),':', 'Color', 'k', 'LineWidth', 2.0);  
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.ASNSTA.CONTROL_STATE(7,:),'-', 'Color', 'k', 'LineWidth', 2.0);  
    plot(SIMULATION_DATA.TIME, SIMULATION_DATA.ASNSTA.CONTROL_STATE(8,:),'-', 'Color', 'g', 'LineWidth', 2.0);  
    xlabel ('Time (s)', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('$\dot{e}_{x}(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    title('Evolution of error variables', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    ylabel ('s(t),$e(t)$,z(t),$\sigma(t)$', 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    xlim([0.0, PARAMETERS.SIMULATION.TOTAL_TIME]);  
    legend({'s(t)','$e(t)$','z(t)','$\sigma(t)$'}, 'FontSize', PARAMETERS.PLOT.FONT_SIZE,'Interpreter','latex');
    if PARAMETERS.PLOT.CREATE_PDF
        graph_file_path = strcat('../MANUSCRIPT/GRAPHICS/scenario_', num2str(PARAMETERS.SIMULATION.SCENARIO), '_s_e_z_sigma.pdf')
        export_fig(graph_file_path, '-transparent', '-nocrop');
    end
     

end

