clc;
clear;
close all;

%-------------------------------------------------------------------------%

Pp = [30, 0, 140];  
Rp = eye(3); 

q = inverse_kinematics(Pp, Rp);
disp('Joint angles (in radians):');
disp(q);
disp('Joint angles (in degrees):');
disp(q * 180 / pi);

figure;
set(gcf, 'Position', [100, 100, 1200, 500]); 

subplot(1, 2, 1);
plot_robot(Pp, Rp, q, 'top');

subplot(1, 2, 2);
plot_robot(Pp, Rp, q, 'front');

sgtitle('6-DOF Parallel Manipulator');

%-------------------------------------------------------------------------%

function q_numeric = inverse_kinematics(Pp, Rp)

    R1 = 250;                    
    Li = 222;         
    %li = 40;
    %di = 160; 
    alpha_i = deg2rad([30, 90, 150, 210, 270, 330]);  
    R8 = 153;                   
    beta_i = deg2rad([7.5, 112.5, 127.5, 232.5, 247.5, 352.5]); 

    px = Pp(1);
    py = Pp(2);
    pz = Pp(3);

    pEi = zeros(3, 6);
    for i = 1:6
        pEi(:, i) = [px; py; pz] + Rp * [R8 * cos(beta_i(i)); R8 * sin(beta_i(i)); 0];
    end

    syms q1 q2 q3 q4 q5 q6
    q = [q1, q2, q3, q4, q5, q6];

    eqns = sym(zeros(1, 6));
    for i = 1:6
        pKi = R1 * [cos(alpha_i(i) + q(i)); sin(alpha_i(i) + q(i)); 0];
        eqns(i) = (pEi(1, i) - pKi(1))^2 + (pEi(2, i) - pKi(2))^2 + (pEi(3, i) - pKi(3))^2 == Li^2;
    end

    sol = vpasolve(eqns, q);

    q_numeric = double([sol.q1, sol.q2, sol.q3, sol.q4, sol.q5, sol.q6]);
end

function plot_robot(Pp, Rp, q, view_type)

    R1 = 250; 
    alpha_i = [30, 90, 150, 210, 270, 330] * pi / 180; 
    R8 = 153; 
    beta_i = [7.5, 112.5, 127.5, 232.5, 247.5, 352.5] * pi / 180;

    px = Pp(1);
    py = Pp(2);
    pz = Pp(3);
    Rp = reshape(Rp, [3, 3]);

    pEi = zeros(3, 6);
    for i = 1:6
        pEi(:, i) = [px; py; pz] + Rp * [R8 * cos(beta_i(i)); R8 * sin(beta_i(i)); 0];
    end

    pKi = zeros(3, 6);
    for i = 1:6
        pKi(:, i) = R1 * [cos(alpha_i(i) + q(i)); sin(alpha_i(i) + q(i)); 0];
    end

    theta = linspace(0, 2*pi, 100);
    x_base = R1 * cos(theta);
    y_base = R1 * sin(theta);
    z_base = zeros(size(theta));

    x_platform = R8 * cos(theta) + px;
    y_platform = R8 * sin(theta) + py;
    z_platform = pz * ones(size(theta));

    hold on;
    grid on;
    axis equal;

    if strcmpi(view_type, 'top')

        view([0, 90]);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('Top View: 6-DOF Parallel Manipulator');

    elseif strcmpi(view_type, 'front')

        view([0, 0]);
        xlabel('X');
        ylabel('Z');
        zlabel('Y');
        title('Front View: 6-DOF Parallel Manipulator');
    else
        error('Invalid view type');
    end

    plot3(x_base, y_base, z_base, 'k--');
    plot3(x_platform, y_platform, z_platform, 'b--');

    for i = 1:6
        plot3([pKi(1, i), pEi(1, i)], [pKi(2, i), pEi(2, i)], [pKi(3, i), pEi(3, i)], 'r', 'LineWidth', 2);
        plot3(pKi(1, i), pKi(2, i), pKi(3, i), 'ro', 'MarkerFaceColor', 'r');
        plot3(pEi(1, i), pEi(2, i), pEi(3, i), 'bo', 'MarkerFaceColor', 'b');
    end

    plot3(px, py, pz, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

    %legend({'Base Circle', 'Platform Circle', 'Legs', 'Base Joints', 'Platform Joints', 'End-Effector'}, 'Location', 'Best');
    hold off;
end
