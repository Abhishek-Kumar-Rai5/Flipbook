clc; clear; close all;

% Prompt user for sequence of animation commands
available_commands = {'lift', 'displace', 'place', 'rotatearm', 'extendarm', 'swingobject', 'raisepillar', 'tiltobject'};
disp("Available commands: " + strjoin(available_commands, ', '));
disp("Type commands one by one. Type 'start' to begin animation.");

% Animation parameters - defined early so we can use them
crane_arm_length = 10;
object_width = 2;
object_height = 1.2;

commands = {};
params = struct();

% Default starting values
current_state.displace = 0;
current_state.arm_angle = 0;
current_state.arm_scale = 1;
current_state.swing = false;
current_state.tilt = 0;
current_state.pillar_height = 8;

% Cable settings
cable_initial_length = 6;
cable_final_length = 2;
cable_extended_length = 9; % Extended length for placing objects
crane_base_width = 2;

% Track placed objects
placed_objects = []; % Track objects that have been placed

while true
    cmd = lower(strtrim(input('Enter command: ', 's')));
    if strcmpi(cmd, 'start')
        break;
    elseif ismember(cmd, available_commands)
        commands{end+1} = cmd;
        cmd_idx = length(commands);

        % Store the parameter and the index
        switch cmd
            case 'rotatearm'
                angle = input('Enter NEW angle of arm (degrees, positive anticlockwise, negative clockwise): ');
                params(cmd_idx).type = 'rotatearm';
                params(cmd_idx).start = current_state.arm_angle;
                params(cmd_idx).target = angle;
                current_state.arm_angle = angle; % Update current

            case 'extendarm'
                scale = input('Enter arm scale factor: ');
                params(cmd_idx).type = 'extendarm';
                params(cmd_idx).start = current_state.arm_scale;
                params(cmd_idx).target = scale;
                current_state.arm_scale = scale; % Update current

            case 'swingobject'
                params(cmd_idx).type = 'swingobject';
                params(cmd_idx).start = current_state.swing;
                params(cmd_idx).target = true;
                current_state.swing = true; % Update current

            case 'tiltobject'
                tilt = input('Enter object tilt angle (degrees): ');
                params(cmd_idx).type = 'tiltobject';
                params(cmd_idx).start = current_state.tilt;
                params(cmd_idx).target = tilt;
                current_state.tilt = tilt; % Update current

            case 'raisepillar'
                height = input('Enter new pillar height: ');
                params(cmd_idx).type = 'raisepillar';
                params(cmd_idx).start = current_state.pillar_height;
                params(cmd_idx).target = height;
                current_state.pillar_height = height; % Update current

            case 'displace'
                dist = input('Enter horizontal displacement: ');
                params(cmd_idx).type = 'displace';
                params(cmd_idx).start = current_state.displace;
                params(cmd_idx).target = dist;
                current_state.displace = dist; % Update current

            case 'lift'
                params(cmd_idx).type = 'lift';
                
            case 'place'
                params(cmd_idx).type = 'place';
                params(cmd_idx).place_x = current_state.displace + 0.5 * crane_base_width - 0.5 + ...
                                         crane_arm_length * current_state.arm_scale * 0.6 * cos(deg2rad(current_state.arm_angle));
        end

    else
        disp('Invalid command. Valid commands: ' + strjoin(available_commands, ', '));
    end
end

% Animation parameters
frames_per_command = 100;
frame_count = frames_per_command * length(commands);

% Prepare figure
figure('Position', [100, 100, 1000, 600]);
axis equal;
axis([-5 25 -1 18]);
hold on;

unit_rect = [0 1 1 0; 0 0 1 1; 1 1 1 1];

% For creating circle shape
n_circle = 20;
theta = linspace(0, 2*pi, n_circle);
unit_circle = [0.5*cos(theta); 0.5*sin(theta); ones(1, n_circle)];

% Reset state for animation
state.arm_angle = 0;
state.arm_scale = 1;
state.swing = false;
state.tilt = 0;
state.displace = 0;
state.pillar_height = 8;
cable_length = cable_initial_length;

% Building foundation coordinates
foundation_x = 5;
foundation_width = 5;
ground_y = 0;

% Track if an object is being held
object_held = true;

for t = 1:frame_count
    cla;

    current_command_index = ceil(t / frames_per_command);
    local_frame = mod(t-1, frames_per_command) + 1;
    if current_command_index > length(commands)
        break;
    end

    % Copy previous state unless changing
    cmd_type = params(current_command_index).type;
    cmd_info = params(current_command_index);

    progress = local_frame/frames_per_command;
    if progress > 1, progress = 1; end

    % Draw background sky
    set(gca, 'Color', [0.8 0.95 1]);
    
    % Draw sun
    sun_x = 20;
    sun_y = 16;
    sun_radius = 1.2;
    S_sun = [sun_radius, 0, 0; 0, sun_radius, 0; 0, 0, 1];
    T_sun = [1, 0, sun_x; 0, 1, sun_y; 0, 0, 1];
    sun = T_sun * S_sun * unit_circle;
    fill(sun(1,:), sun(2,:), [1 0.9 0.2]);
    
    % Clouds
    cloud_centers = [5 15; 14 17; 8 12];
    for i = 1:size(cloud_centers, 1)
        cloud_x = cloud_centers(i, 1);
        cloud_y = cloud_centers(i, 2);
        for j = 1:3
            radius = 0.6 + 0.2 * rand();
            offset_x = 0.7 * j;
            offset_y = 0.2 * sin(j);
            S_cloud = [radius, 0, 0; 0, radius, 0; 0, 0, 1];
            T_cloud = [1, 0, cloud_x + offset_x; 0, 1, cloud_y + offset_y; 0, 0, 1];
            cloud = T_cloud * S_cloud * unit_circle;
            fill(cloud(1,:), cloud(2,:), [0.95 0.95 0.95]);
        end
    end
    
    % Draw ground
    ground_width = 30;
    ground_color = [0.6 0.5 0.3];
    ground_height = 1;
    fill([-5 ground_width ground_width -5], [ground_y ground_y ground_y-ground_height ground_y-ground_height], ground_color);
    
    % Draw building foundation
    fill([foundation_x foundation_x+foundation_width foundation_x+foundation_width foundation_x], ...
         [ground_y ground_y ground_y+1 ground_y+1], [0.7 0.7 0.7]);
    
    % Update state based on current command
    switch cmd_type
        case 'rotatearm'
            state.arm_angle = cmd_info.start + progress * (cmd_info.target - cmd_info.start);
        case 'extendarm'
            state.arm_scale = cmd_info.start + progress * (cmd_info.target - cmd_info.start);
        case 'swingobject'
            state.swing = true;
        case 'tiltobject'
            state.tilt = cmd_info.start + progress * (cmd_info.target - cmd_info.start);
        case 'raisepillar'
            state.pillar_height = cmd_info.start + progress * (cmd_info.target - cmd_info.start);
        case 'displace'
            state.displace = cmd_info.start + progress * (cmd_info.target - cmd_info.start);
    end

    % Cable adjustment for lift/place
    if strcmp(commands{current_command_index}, 'lift')
        cable_length = cable_initial_length - progress * (cable_initial_length - cable_final_length);
    elseif strcmp(commands{current_command_index}, 'place')
        % Modified place command - extend cable more
        cable_length = cable_final_length + progress * (cable_extended_length - cable_final_length);
        
        % If we're at the end of placing, create a placed object
        if progress > 0.95 && object_held
            place_x = cmd_info.place_x;
            
            % Add to placed objects list
            new_placed = struct();
            new_placed.x = place_x;
            new_placed.y = ground_y + 0.6; % Place slightly above ground
            new_placed.width = object_width;
            new_placed.height = object_height;
            new_placed.color = [0.8 0.2 0.2];
            placed_objects = [placed_objects, new_placed];
            
            % Object is no longer held but still visible
            object_held = false;
        end
    end
    
    % Draw placed objects on ground
    if ~isempty(placed_objects)
        for i = 1:length(placed_objects)
            obj = placed_objects(i);
            S_obj = [obj.width, 0, 0; 0, obj.height, 0; 0, 0, 1];
            T_obj = [1, 0, obj.x - obj.width/2; 0, 1, obj.y; 0, 0, 1];
            box = T_obj * S_obj * unit_rect;
            fill(box(1,:), box(2,:), obj.color);
        end
    end

    % Draw crane base
    S_base = [crane_base_width, 0, 0; 0, 1, 0; 0, 0, 1];
    T_base = [1, 0, 0; 0, 1, ground_y; 0, 0, 1];
    base = T_base * S_base * unit_rect;
    fill(base(1,:), base(2,:), [0.5 0.5 0.5]);

    % Pillar
    S_pillar = [1, 0, 0; 0, state.pillar_height, 0; 0, 0, 1];
    T_pillar = [1, 0, 0.5 * crane_base_width - 0.5; 0, 1, ground_y + 1; 0, 0, 1];
    pillar = T_pillar * S_pillar * unit_rect;
    fill(pillar(1,:), pillar(2,:), [0.4 0.4 0.4]);

    % Arm
    angle_rad = deg2rad(state.arm_angle);
    R_arm = [cos(angle_rad), -sin(angle_rad), 0; sin(angle_rad), cos(angle_rad), 0; 0, 0, 1];
    S_arm = [crane_arm_length * state.arm_scale, 0, 0; 0, 1, 0; 0, 0, 1];
    T_arm = [1, 0, 0.5 * crane_base_width - 0.5; 0, 1, ground_y + state.pillar_height + 1; 0, 0, 1];
    arm = T_arm * R_arm * S_arm * unit_rect;
    fill(arm(1,:), arm(2,:), [0.3 0.3 0.3]);
    
    % Add warning light on top of crane
    light_radius = 0.3;
    S_light = [light_radius, 0, 0; 0, light_radius, 0; 0, 0, 1];
    T_light = [1, 0, 0.5 * crane_base_width - 0.5; 0, 1, ground_y + state.pillar_height + 1.5; 0, 0, 1];
    light = T_light * S_light * unit_circle;
    
    % Blinking light
    if mod(local_frame, 20) < 10
        light_color = [1 0.3 0.1]; % Orange
    else
        light_color = [0.7 0.2 0.2]; % Red
    end
    fill(light(1,:), light(2,:), light_color);

    % Hook position
    hook_local = [0.6; 0.5; 1];
    hook_global = T_arm * R_arm * S_arm * hook_local;
    hook_x = hook_global(1) + state.displace;
    extra_length = 0.3;
    hook_y_top = hook_global(2) + extra_length;
    hook_y_bottom = hook_y_top - (cable_length - extra_length);

    % Draw cable
    line([hook_x, hook_x], [hook_y_top, hook_y_bottom], 'Color', 'k', 'LineWidth', 2);
    
    % Draw hook
    hook_size = 0.4;
    hook_curve = 0.2;
    hook_points_x = [hook_x, hook_x, hook_x - hook_size/2, hook_x - hook_size, hook_x - hook_size + hook_curve];
    hook_points_y = [hook_y_bottom, hook_y_bottom - hook_size/2, hook_y_bottom - hook_size/2, hook_y_bottom - hook_size/3, hook_y_bottom];
    plot(hook_points_x, hook_points_y, 'k-', 'LineWidth', 3);

    % Object (with swing and tilt) - only if being held
    if object_held
        pivot_x = hook_x;
        pivot_y = hook_y_bottom;
        swing_theta = (state.swing) * deg2rad(5) * sin(2 * pi * local_frame / 60);
        tilt_rad = deg2rad(state.tilt);

        S_obj = [object_width, 0, 0; 0, object_height, 0; 0, 0, 1];
        T_obj = [1, 0, pivot_x - object_width/2; 0, 1, hook_y_bottom - object_height; 0, 0, 1];
        T_to_origin = [1, 0, -pivot_x; 0, 1, -pivot_y; 0, 0, 1];
        R_swing = [cos(swing_theta), -sin(swing_theta), 0; sin(swing_theta), cos(swing_theta), 0; 0, 0, 1];
        R_tilt = [cos(tilt_rad), -sin(tilt_rad), 0; sin(tilt_rad), cos(tilt_rad), 0; 0, 0, 1];
        T_back = [1, 0, pivot_x; 0, 1, pivot_y; 0, 0, 1];
        object = T_back * R_swing * R_tilt * T_to_origin * T_obj * S_obj * unit_rect;
        fill(object(1,:), object(2,:), [0.8 0.2 0.2]);
    end

    % Status information
    status_text = sprintf('Command: %s (%d/%d) - Progress: %.0f%%', ...
        commands{current_command_index}, current_command_index, length(commands), progress*100);
    title(status_text);
    
    
    pause(0.01);
end

% Final message
if ~isempty(commands)
    disp('Animation complete!');
    if ~isempty(placed_objects)
        disp(['Objects placed on ground: ' num2str(length(placed_objects))]);
    end
end