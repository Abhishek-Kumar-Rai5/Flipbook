clc;
clear;
close all;

%% 1. Define the original shape (square)
shape = [-1 1 1 -1; -1 -1 1 1]; % Square
original_shape = shape;

%% 2. Define transformations
transformations = {
    [-1 0; 0 1],            'Reflect Y-axis';     % (Proper) Reflection about Y-axis (flips x)
    [1 0; 0 -1],            'Reflect X-axis';     % Reflection about X-axis (flips y)
    [cos(pi/6) -sin(pi/6); sin(pi/6) cos(pi/6)], 'Rotate 30°';
    [1.5 0; 0 1.5],         'Scale x1.5';
    [1 0.7; 0 1],           'Shear X';
    [1 0; 0.7 1],           'Shear Y';
    eye(2),                 'Translate';          % Placeholder, apply separately
};

translation_vector = [2; 1]; % Translation offset

%% 3. Randomly select and apply a chain of transformations
max_steps = length(transformations);
while true
    num_steps = input(['How many transformations would you like to perform? (1–', num2str(max_steps), '): ']);
    if isnumeric(num_steps) && num_steps >= 1 && num_steps <= max_steps
        break;
    else
        disp('⚠️ Please enter a valid number within the allowed range.');
    end
end

chain_indices = randperm(max_steps, num_steps);

applied_chain = {};
applied_names = {};
current_shape = original_shape;

figure;
axis equal;
axis([-10 10 -10 10]);
hold on;
title('Watch the Transformation Sequence');

for i = 1:num_steps
    idx = chain_indices(i);
    T = transformations{idx,1};
    name = transformations{idx,2};
    
    % Apply transformation with animation
    for f = 1:10
        alpha = f / 10;
        if strcmp(name, 'Translate')
            interp_shape = current_shape + alpha * translation_vector;
        else
            interp_T = (1 - alpha) * eye(2) + alpha * T;
            interp_shape = interp_T * current_shape;
        end
        
        clf;
        fill(interp_shape(1,:), interp_shape(2,:), 'b');
        axis equal;
        axis([-10 10 -10 10]);
        pause(0.2); % FASTER FLIPBOOK SPEED
    end
    
    % Apply full transformation
    if strcmp(name, 'Translate')
        current_shape = current_shape + translation_vector;
    else
        current_shape = T * current_shape;
    end
    
    applied_chain{end+1} = T;
    applied_names{end+1} = name;
end

final_shape = current_shape;

pause(1);
clf;
title('Your Turn: Reverse the Transformations');

%% 4. Display menu for transformations
menu_list = {
    '1 = Reflect Y-axis', ...
    '2 = Reflect X-axis', ...
    '3 = Rotate 30°', ...
    '4 = Scale x1.5', ...
    '5 = Shear X', ...
    '6 = Shear Y', ...
    '7 = Translate (reverse direction)'
};
disp("Available Inverse Operations:");
disp(strjoin(menu_list, ' | '));

%% 5. User applies reverse steps
user_chain = {};
disp('Apply transformations in reverse order of how you think they happened.');
for i = 1:num_steps
    choice = input(['Step ', num2str(i), ' - Your inverse choice: ']);
    if all(choice >= 1 & choice <= 7)
        if choice == 7
            user_chain{end+1} = -translation_vector;
        else
            user_chain{end+1} = inv(transformations{choice, 1});
        end
    else
        disp('Invalid input, try again.');
        i = i - 1;
    end
end

%% 6. Apply user's reverse transformations
current_attempt = final_shape;
resultFig = figure('Name','Your Attempt','NumberTitle','off','Visible','on','WindowStyle','normal');
set(0, 'CurrentFigure', resultFig);
drawnow;

for i = 1:num_steps
    T = user_chain{i};
    
    for f = 1:10
        alpha = f / 10;
        if isvector(T)
            interp_shape = current_attempt + alpha * T;
        else
            interp_T = (1 - alpha) * eye(2) + alpha * T;
            interp_shape = interp_T * current_attempt;
        end
        
        clf;
        fill(interp_shape(1,:), interp_shape(2,:), 'g');
        title(['Your Inverse Step ', num2str(i)]);
        axis equal;
        axis([-10 10 -10 10]);
        pause(0.2); % FASTER FLIPBOOK SPEED
    end
    
    if isvector(T)
        current_attempt = current_attempt + T;
    else
        current_attempt = T * current_attempt;
    end
end

%% 7. Check match and display in a new figure window (forced)
difference = norm(current_attempt - original_shape);
tolerance = 1e-1;

finalFig = figure('Name', 'Final Result', ...
                  'NumberTitle', 'off', ...
                  'Visible', 'on', ...
                  'WindowStyle', 'normal');
              
fill(current_attempt(1,:), current_attempt(2,:), 'g');
axis equal;
axis([-10 10 -10 10]);
title('Your Final Reversed Shape');

if difference < tolerance
    disp("Success! You accurately reversed all transformations.");
    annotation('textbox', [0.3 0.8 0.4 0.1], 'String', '🎉 SUCCESS!', ...
        'FontSize', 16, 'FontWeight', 'bold', 'EdgeColor', 'none', 'Color', 'green', ...
        'Parent', gca);
else
    disp("Incorrect. Try again. Hint: Order and precision matter.");
    disp("Actual Transform Chain (in order):");
    disp(strjoin(applied_names, ' -> '));
    annotation('textbox', [0.3 0.8 0.4 0.1], 'String', 'INCORRECT!', ...
        'FontSize', 16, 'FontWeight', 'bold', 'EdgeColor', 'none', 'Color', 'red', ...
        'Parent', gca);
end