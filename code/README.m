% Harrison Hidalgo
% ECE 5725 - Final Project
% README

%% STEP 1
%%%%%First find ball and estimate state
    % Collect measurement with Camera
    % Feed measurement into SR_SPF_Ball.m
        
    % Repeat until covarience small enough
%% STEP 2
%%%%%Feed into system_iterator.m to find possible orientations
    % This calls path_tracker
        % This calls runge-kutta
            % This calls ball_calc.m
                % This calls A_ball.m and b_ball.m
%% STEP 3
%%%%%We feed these orientations into orientation_selector.m
    % First we check to see if the possible orientations are in our
    % workspace.
        % Use find_angles.m
    % Current orientation is compared with possible orientations. The
    % closest is selected.
    % Start with closest, maybe least square error
    
%% STEP 4
%%%%%We feed the selected orientation into inverse_kinematics.m

%% STEP 5
%%%%%The result from this is fed into the backboard control loop

%% Simulation
    % Simulate_Shot.m
    % Plot_Relevant_Points.m
    % Plot_Motor_Setup.m
    % Plot_Box.m
    % runge_kutta.m
    % A_ball.m
    % b_ball.m
    % ball_calc.m
    % did_it_collide.m
    % collision.m
    