clc; clear; close all;

% === Define Rocket Parts in Homogeneous Coordinates ===
rocket_body = [0 1 1 0;
               0 0 2 2];
           
rocket_nose = [0 1 0.5;
               2 2 3];
           
flame = [0   1    0.5;
        -0.2 -0.2 -1.0];

% === ENLARGE THE ROCKET ===
scale_factor = 2;
rocket_body = rocket_body * scale_factor;
rocket_nose = rocket_nose * scale_factor;
flame = flame * scale_factor;

% Initialize position
rocket_position = [0, 0];
show_flame = true; % New variable to control flame visibility

% === Allowed Commands ===
valid_commands = ["down", "up", "left", "right", "blastoff", "grow", "shrink", "hideflame", "showflame", "spin", "rotate", "pause", "orbit"];

% === Get User Commands First ===
disp('Enter commands for the rocket animation:');
disp('Options: down, up, left, right, blastoff, grow, shrink, hideflame, showflame, spin, rotate <angle>, pause <seconds>, orbit <central_mass>');
disp('Type ''done'' when finished.');
commands = {};

while true
    cmd = input('Command: ', 's');
    if strcmpi(cmd, 'done')
        break;
    end

    parts = strsplit(lower(strtrim(cmd)));
    if isempty(parts)
        disp('Empty command, please try again.');
        continue;
    end

    action = parts{1};
    if any(strcmp(action, valid_commands))
        commands{end+1} = cmd; %#ok<SAGROW>
    else
        disp(['Invalid command: "' action '". Please enter a valid command!']);
    end
end

% === Set up the Figure AFTER getting commands ===
figure('Color','k');
axis equal;
axis([-20 20 -20 40]);
axis off;
hold on;

% Create Static Stars Only ONCE
stars_x = rand(1, 50) * 40 - 20;
stars_y = rand(1, 50) * 60 - 20;
plot(stars_x, stars_y, 'w.', 'MarkerSize', 2);

% Display current date/time and user
text(-19, 38, 'Date: 2025-05-31 12:12:16', 'Color', 'w', 'FontSize', 8);
text(-19, 36, 'User: Abhishek-Kumar-Rai5', 'Color', 'w', 'FontSize', 8);

% === Execute User Commands ===
for i = 1:length(commands)
    parts = strsplit(lower(strtrim(commands{i})));
    if isempty(parts)
        continue;
    end
    
    action = parts{1};
    
    switch action
        case 'up'
            distance = getDistance(parts);
            rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, 0, distance, 0.0001, stars_x, stars_y, show_flame);

        case 'down'
            distance = getDistance(parts);
            rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, 0, -distance, 0.0001, stars_x, stars_y, show_flame);

        case 'left'
            distance = getDistance(parts);
            rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, -distance, 0, 0.005, stars_x, stars_y, show_flame);

        case 'right'
            distance = getDistance(parts);
            rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, distance, 0, 0.005, stars_x, stars_y, show_flame);

        case 'blastoff'
            rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, 0, 15, 0.000001, stars_x, stars_y, show_flame);

        case 'grow'
            scale_factor = 1.5; % Grow by 50%
            rocket_body = rocket_body * scale_factor;
            rocket_nose = rocket_nose * scale_factor;
            flame = flame * scale_factor;

        case 'shrink'
            scale_factor = 0.5; % Shrink to half
            rocket_body = rocket_body * scale_factor;
            rocket_nose = rocket_nose * scale_factor;
            flame = flame * scale_factor;

        case 'hideflame'
            show_flame = false;

        case 'showflame'
            show_flame = true;

        case 'spin'
            angle = 10; % Degrees per spin frame
            frames = 36; % Complete 360 rotation
            for k = 1:frames
                clf;
                axis([-20 20 -20 40]);
                axis off;
                axis equal;
                hold on;
                plot(stars_x, stars_y, 'w.', 'MarkerSize', 2);
                text(-19, 38, 'Date: 2025-05-31 12:12:16', 'Color', 'w', 'FontSize', 8);
                text(-19, 36, 'User: Abhishek-Kumar-Rai5', 'Color', 'w', 'FontSize', 8);

                theta = deg2rad(k * angle);
                R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
                
                body_t = (R * rocket_body) + rocket_position';
                nose_t = (R * rocket_nose) + rocket_position';
                flame_t = (R * flame) + rocket_position';

                drawRocket(body_t, nose_t, flame_t, show_flame);

                drawnow;
                pause(0.01);
            end

        case 'rotate'
            if length(parts) >= 2
                angle = str2double(parts{2});
                if isnan(angle)
                    disp('Invalid angle value, using default 45 degrees.');
                    angle = 45;
                end
            else
                disp('No angle provided, using default 45 degrees.');
                angle = 45;
            end
            
            theta = deg2rad(angle);
            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

            rocket_body = R * rocket_body;
            rocket_nose = R * rocket_nose;
            flame = R * flame;
            
            clf;
            axis([-20 20 -20 40]);
            axis off;
            axis equal;
            hold on;
            plot(stars_x, stars_y, 'w.', 'MarkerSize', 2);
            text(-19, 38, 'Date: 2025-05-31 12:12:16', 'Color', 'w', 'FontSize', 8);
            text(-19, 36, 'User: Abhishek-Kumar-Rai5', 'Color', 'w', 'FontSize', 8);
            
            body_t = rocket_body + rocket_position';
            nose_t = rocket_nose + rocket_position';
            flame_t = flame + rocket_position';
            
            drawRocket(body_t, nose_t, flame_t, show_flame);

            drawnow;
            pause(0.01);

        case 'pause'
            if length(parts) >= 2
                pause_time = str2double(parts{2});
                if isnan(pause_time)
                    pause_time = 2;
                end
            else
                pause_time = 2;
            end
            pause(pause_time);
            
        case 'orbit'
            % Get central mass parameter
            if length(parts) >= 2
                central_mass = str2double(parts{2});
                if isnan(central_mass)
                    disp('Invalid mass value, using default 100.');
                    central_mass = 100;
                end
            else
                disp('No mass provided, using default 100.');
                central_mass = 100;
            end
            
            % Perform orbital mechanics animation
            rocket_position = orbitTrajectory(rocket_body, rocket_nose, flame, rocket_position, central_mass, stars_x, stars_y, show_flame);
    end
end

disp('Animation Complete!');

%% === User-defined Functions ===

function rocket_position = translateRocket(rocket_body, rocket_nose, flame, rocket_position, dx, dy, speed, stars_x, stars_y, show_flame)
    boost = 5;
    boosted_dx = dx * boost;
    boosted_dy = dy * boost;
    new_position = rocket_position + [boosted_dx, boosted_dy];

    frames = 30;
    for t = 1:frames
        clf;
        axis([-20 20 -20 40]);
        axis off;
        axis equal;
        hold on;
        
        plot(stars_x, stars_y, 'w.', 'MarkerSize', 2);
        text(-19, 38, 'Date: 2025-05-31 12:12:16', 'Color', 'w', 'FontSize', 8);
        text(-19, 36, 'User: Abhishek-Kumar-Rai5', 'Color', 'w', 'FontSize', 8);
        
        intermediate_pos = rocket_position + (new_position - rocket_position) * (t/frames);
        
        body_t = rocket_body + intermediate_pos';
        nose_t = rocket_nose + intermediate_pos';
        flame_t = flame + intermediate_pos';
        
        drawRocket(body_t, nose_t, flame_t, show_flame);
        
        drawnow;
        pause(speed);
    end
    
    rocket_position = new_position;
end

function drawRocket(body_t, nose_t, flame_t, show_flame)
    fill(body_t(1,:),  body_t(2,:),  [0.5 0.5 1], 'EdgeColor', 'none');
    fill(nose_t(1,:),  nose_t(2,:),  [1 0 0],     'EdgeColor', 'none');
    if show_flame
        fill(flame_t(1,:), flame_t(2,:), [1 0.5 0], 'EdgeColor', 'none');
    end
end

function distance = getDistance(parts)
    if length(parts) >= 2
        distance = str2double(parts{2});
        if isnan(distance)
            distance = 5;
        end
    else
        distance = 5;
    end
end

function final_position = orbitTrajectory(rocket_body, rocket_nose, flame, initial_position, central_mass, stars_x, stars_y, show_flame)
    % Set up parameters for orbital mechanics
    G = 0.1;  % Gravitational constant (scaled for animation)
    
    % Define central body position and draw it
    central_position = [0; 0];
    central_radius = sqrt(central_mass) * 0.15; % Size related to mass
    
    % Initial conditions for orbit
    distance_to_center = norm(initial_position - central_position') - central_radius;
    
    % Calculate initial velocity for stable circular orbit using linear algebra
    position_vector = initial_position' - central_position;
    position_norm = norm(position_vector);
    
    % Unit vector perpendicular to position (for circular orbit)
    perpendicular_vector = [-position_vector(2); position_vector(1)] / position_norm;
    
    % Speed required for circular orbit: v = sqrt(GM/r)
    orbit_speed = sqrt(G * central_mass / position_norm);
    
    % Initial velocity vector
    initial_velocity = orbit_speed * perpendicular_vector;
    
    % Simulation parameters
    time_steps = 200;
    dt = 0.1;
    
    % Create trajectory using direct numerical integration
    % This is more stable than the eigenvalue method
    trajectory = zeros(2, time_steps);
    velocity = zeros(2, time_steps);
    
    % Initialize
    trajectory(:,1) = initial_position';
    velocity(:,1) = initial_velocity;
    
    % Store rotations for the rocket
    rotations = zeros(1, time_steps);
    rotations(1) = atan2(initial_velocity(2), initial_velocity(1)) + pi/2;
    
    % Numerically integrate the orbit using simple physics
    for t = 2:time_steps
        % Current position
        r = trajectory(:,t-1);
        v = velocity(:,t-1);
        
        % Distance to central body
        r_to_central = r - central_position;
        r_norm = norm(r_to_central);
        
        % Calculate gravitational acceleration
        a = -G * central_mass * r_to_central / r_norm^3;
        
        % Update velocity and position (simple Euler integration)
        v_new = v + a * dt;
        r_new = r + v_new * dt;
        
        % Store new values
        trajectory(:,t) = r_new;
        velocity(:,t) = v_new;
        
        % Calculate rotation angle to point in the direction of motion
        direction = v_new;
        rotations(t) = atan2(direction(2), direction(1)) + pi/2;
    end
    
    % Animation loop
    for t = 1:time_steps
        clf;
        axis([-20 20 -20 40]);
        axis off;
        axis equal;
        hold on;
        
        % Draw stars
        plot(stars_x, stars_y, 'w.', 'MarkerSize', 2);
        text(-19, 38, 'Date: 2025-05-31 12:12:16', 'Color', 'w', 'FontSize', 8);
        text(-19, 36, 'User: Abhishek-Kumar-Rai5', 'Color', 'w', 'FontSize', 8);
        
        % Draw central body (planet/sun)
        theta = linspace(0, 2*pi, 50);
        planet_x = central_position(1) + central_radius * cos(theta);
        planet_y = central_position(2) + central_radius * sin(theta);
        
        % Use a color gradient for central body
        if central_mass > 200
            % Sun-like (yellow-orange)
            fill(planet_x, planet_y, [1 0.8 0], 'EdgeColor', 'none');
        elseif central_mass > 50
            % Gas giant (blue-ish)
            fill(planet_x, planet_y, [0.2 0.4 0.8], 'EdgeColor', 'none');
        else
            % Earth-like (blue-green)
            fill(planet_x, planet_y, [0.2 0.6 0.8], 'EdgeColor', 'none');
        end
        
        % Draw orbit path (faint line)
        % Fixed the Alpha property issue by using color with lower intensity
        for i = 1:t
            if i > 5
                plot(trajectory(1, i-5:i), trajectory(2, i-5:i), 'w-', 'LineWidth', 0.5, 'Color', [1 1 1 0.3]);
            end
        end
        
        % Draw full trajectory in very light color
        plot(trajectory(1, 1:t), trajectory(2, 1:t), '-', 'LineWidth', 0.5, 'Color', [0.5 0.5 0.5]);
        
        % Get current position
        current_position = trajectory(:, t);
        
        % Calculate orientation matrix
        rotation_angle = rotations(t);
        R = [cos(rotation_angle), -sin(rotation_angle); 
             sin(rotation_angle), cos(rotation_angle)];
        
        % Apply rotation and translation to rocket
        body_t = R * rocket_body + current_position;
        nose_t = R * rocket_nose + current_position;
        flame_t = R * flame + current_position;
        
        % Draw the rocket
        drawRocket(body_t, nose_t, flame_t, show_flame);
        
        % Calculate orbital parameters for display
        current_velocity = velocity(:, t);
        speed = norm(current_velocity);
        radius = norm(current_position - central_position);
        period = 2 * pi * radius / speed;
        
        % Calculate stability using Kepler's laws
        energy = 0.5 * speed^2 - G * central_mass / radius;
        angular_momentum = radius * speed;
        
        % Determine orbit type using energy
        if abs(energy) < 0.01
            orbit_type = 'Circular';
        elseif energy < 0
            orbit_type = 'Elliptical';
        elseif abs(energy) < 0.1
            orbit_type = 'Parabolic';
        else
            orbit_type = 'Hyperbolic';
        end
        
        % Display orbital parameters
        text(-19, 34, sprintf('Orbital radius: %.2f', radius), 'Color', 'w', 'FontSize', 8);
        text(-19, 32, sprintf('Orbital speed: %.2f', speed), 'Color', 'w', 'FontSize', 8);
        text(-19, 30, sprintf('Orbital period: %.2f', period), 'Color', 'w', 'FontSize', 8);
        text(-19, 28, sprintf('Central mass: %.2f', central_mass), 'Color', 'w', 'FontSize', 8);
        text(-19, 26, ['Orbit type: ' orbit_type], 'Color', 'w', 'FontSize', 8);
        
        % Display angular momentum (conserved quantity)
        text(-19, 24, sprintf('Angular momentum: %.2f', angular_momentum), 'Color', 'w', 'FontSize', 8);
        
        drawnow;
        pause(0.01);
    end
    
    % Return the final position
    final_position = trajectory(:, end)';
end