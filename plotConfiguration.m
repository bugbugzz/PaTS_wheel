%% Helper Functions

function plotConfiguration(qSol, time, L_vals, scale, plot_stepsize)
    % renders the main animation of the folding wheel

    figure('Name', 'PaTS-Wheel Mechanism Animation', 'NumberTitle', 'off', 'Color', 'w');
    set(gcf, 'Position', [100, 100, 950, 550]); 

    % local math to figure out the exact visual triangles
    L6 = 0.0170; claw_out = 0.0320; claw_in  = 0.0250;
    x_c = (L6^2 + claw_out^2 - claw_in^2) / (2 * L6);
    y_c = sqrt(claw_out^2 - x_c^2);
    
    L7 = 0.0180; pad_out = 0.0375; pad_in  = 0.0260;
    x_p = (L7^2 + pad_out^2 - pad_in^2) / (2 * L7);
    y_p = sqrt(pad_out^2 - x_p^2);

    for i = 1:plot_stepsize:length(time)
        q = qSol(:, i);
        x1=q(1); y1=q(2); th1=q(3);    
        x2=q(4); y2=q(5); th2=q(6);    
        x4=q(10);y4=q(11);th4=q(12);   
        th3=q(9); th5=q(15); 

        O  = [0, 0];                        
        b1 = [-L_vals(1), 0];               
        a4 = [L_vals(3), 0];                

        b3 = [x1 + L_vals(4)*cos(th1 + pi - 1.4), y1 + L_vals(4)*sin(th1 + pi - 1.4)];
        a2 = [x1 + L_vals(2)*cos(th1), y1 + L_vals(2)*sin(th1)]; 
        b2 = [x2 + 0.5*L_vals(5)*cos(th2), y2 + 0.5*L_vals(5)*sin(th2)]; 
        a3 = [x4 + 0.5*L_vals(8)*cos(th4), y4 + 0.5*L_vals(8)*sin(th4)]; 

        c = b2 + (x_c * scale) * [cos(th3), sin(th3)] + (y_c * scale) * [-sin(th3), cos(th3)];
        p = a3 + (x_p * scale) * [-cos(th5), -sin(th5)] + (y_p * scale) * [-sin(th5), cos(th5)];

        clf; hold on; grid on; axis equal;
        axis([-0.25 0.25 -0.05 0.25]); 
        title(sprintf('PaTS-Wheel Simulation | Time = %.2f s', time(i)));
        xlabel('X Position (m)'); ylabel('Y Position (m)');

        % dashed lines tracing to the tips
        plot([b2(1), c(1)], [b2(2), c(2)], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
        plot([b3(1), c(1)], [b3(2), c(2)], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
        plot([a3(1), p(1)], [a3(2), p(2)], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
        plot([a2(1), p(1)], [a2(2), p(2)], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);

        % draw the main solid links
        plot([b3(1), O(1), a2(1)], [b3(2), O(2), a2(2)], 'k-', 'LineWidth', 4);
        plot([b1(1), b2(1)], [b1(2), b2(2)], 'b-', 'LineWidth', 4);
        plot([b2(1), b3(1)], [b2(2), b3(2)], 'c-', 'LineWidth', 4);
        plot([a4(1), a3(1)], [a4(2), a3(2)], 'r-', 'LineWidth', 4);
        plot([a3(1), a2(1)], [a3(2), a2(2)], 'm-', 'LineWidth', 4);

        % draw the ground line
        plot([-0.20, 0.20], [0, 0], 'k--', 'LineWidth', 1.5);

        % add hinges and markers
        joints_x = [O(1), b1(1), a4(1), b3(1), a2(1), b2(1), a3(1)];
        joints_y = [O(2), b1(2), a4(2), b3(2), a2(2), b2(2), a3(2)];
        plot(joints_x, joints_y, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'LineWidth', 1.5); 
        plot(joints_x, joints_y, 'k.', 'MarkerSize', 5); 

        plot((b1(1)+b2(1))/2, (b1(2)+b2(2))/2, 'k+', 'MarkerSize', 8, 'LineWidth', 2);
        plot((a4(1)+a3(1))/2, (a4(2)+a3(2))/2, 'k+', 'MarkerSize', 8, 'LineWidth', 2);
        plot((b2(1)+b3(1))/2, (b2(2)+b3(2))/2, 'k+', 'MarkerSize', 8, 'LineWidth', 2);
        plot((a3(1)+a2(1))/2, (a3(2)+a2(2))/2, 'k+', 'MarkerSize', 8, 'LineWidth', 2);

        plot(c(1), c(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2); 
        plot(p(1), p(2), 'm*', 'MarkerSize', 10, 'LineWidth', 2); 

        % text labels
        text(0, -0.02, 'Link 1 (Coupler)', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
        text((b1(1)+b2(1))/2 - 0.01, (b1(2)+b2(2))/2, 'Link 2 (Claw Support)', 'HorizontalAlignment', 'right', 'FontSize', 11, 'FontWeight', 'bold');
        text((a4(1)+a3(1))/2 + 0.01, (a4(2)+a3(2))/2, 'Link 4 (Pad Support)', 'HorizontalAlignment', 'left', 'FontSize', 11, 'FontWeight', 'bold');
        text((b2(1)+b3(1))/2, (b2(2)+b3(2))/2 + 0.02, 'Link 3 (Claw Tip)', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
        text((a3(1)+a2(1))/2, (a3(2)+a2(2))/2 + 0.02, 'Link 5 (Pad Tip)', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

        drawnow;
    end
end