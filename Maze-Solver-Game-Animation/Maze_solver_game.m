clc; clear; close all;

% Maze Settings
nRows = 7; % Maze rows
nCols = 7; % Maze columns
wall_prob = 0.3; % Probability of wall (0 to 1)

start_pos = [2, 2]; % Start (row, col)
end_pos = [5, 6];   % End (row, col)

% Ask player how long they want to see the maze
display_time = input('Enter number of seconds to memorize the maze: ');

% Generate solvable maze
while true
    maze = generateMaze(nRows, nCols, wall_prob);
    maze(1,:) = 1; maze(end,:) = 1; maze(:,1) = 1; maze(:,end) = 1;
    maze(start_pos(1), start_pos(2)) = 0;
    maze(end_pos(1), end_pos(2)) = 0;
    if isMazeSolvable(maze, start_pos, end_pos)
        break;
    end
end

% Display maze for memorization
fig = figure('Name','Memorize the Maze!','NumberTitle','off');
hold on; axis equal off;
drawMaze(maze, nRows, nCols, start_pos, end_pos);
title('Memorize the Maze! (Red = Start, Green = End)', 'FontSize', 14);

pause(display_time); % Display for the entered seconds
close(fig); % Close maze

disp('Time Up! Now enter your moves from memory.');
disp('-----------------------------------------------');

% Collect moves
commands = {};
disp('Enter moves one-by-one: left / right / up / down');
disp('When done, type "start" to begin animation.');

while true
    move = lower(strtrim(input('Move: ', 's')));
    if strcmp(move, 'start')
        break;
    elseif ismember(move, {'left','right','up','down'})
        commands{end+1} = move;
    else
        disp('Invalid move! Try again.');
    end
end

% Reopen maze for movement animation
fig = figure('Name','Maze Animation','NumberTitle','off');
hold on; axis equal off;
drawMaze(maze, nRows, nCols, start_pos, end_pos);

% Draw player box
current_pos = start_pos;
player = drawPlayer(current_pos, nRows);

% Animate moves
for k = 1:length(commands)
    new_pos = nextPosition(current_pos, commands{k});
    if isValidMove(new_pos, maze, nRows, nCols)
        animateMove(player, current_pos, new_pos, nRows);
        current_pos = new_pos;
    else
        disp(['Invalid move at step ' num2str(k) ', move ignored.']);
    end
end

% Final check
if isequal(current_pos, end_pos)
    disp('Congratulations! You solved the maze!');
    title('YOU WON!', 'FontSize', 16);
else
    disp('You did not reach the goal.');
    title('YOU LOST', 'FontSize', 16);
end

% --- FUNCTIONS ---

function maze = generateMaze(nRows, nCols, wall_prob)
    maze = rand(nRows, nCols) < wall_prob;
end

function solvable = isMazeSolvable(maze, start_pos, end_pos)
    nRows = size(maze,1); nCols = size(maze,2);
    visited = zeros(nRows, nCols);
    queue = start_pos;
    
    while ~isempty(queue)
        pos = queue(1,:); queue(1,:) = [];
        if isequal(pos, end_pos)
            solvable = true;
            return;
        end
        visited(pos(1),pos(2)) = 1;
        
        moves = [-1 0; 1 0; 0 -1; 0 1];
        for i = 1:4
            new_r = pos(1)+moves(i,1);
            new_c = pos(2)+moves(i,2);
            if new_r>=1 && new_r<=nRows && new_c>=1 && new_c<=nCols ...
                    && maze(new_r,new_c)==0 && visited(new_r,new_c)==0
                queue(end+1,:) = [new_r, new_c];
            end
        end
    end
    solvable = false;
end

function drawMaze(maze, nRows, nCols, start_pos, end_pos)
    for row = 1:nRows
        for col = 1:nCols
            color = 'w';
            if maze(row,col)==1
                color = 'k';
            end
            fill([col-1 col col col-1], [nRows-row nRows-row nRows-row+1 nRows-row+1], color);
        end
    end
    % Mark end (green) and start (red)
    fill([end_pos(2)-1 end_pos(2) end_pos(2) end_pos(2)-1], ...
        [nRows-end_pos(1) nRows-end_pos(1) nRows-end_pos(1)+1 nRows-end_pos(1)+1], 'g');
    fill([start_pos(2)-1 start_pos(2) start_pos(2) start_pos(2)-1], ...
        [nRows-start_pos(1) nRows-start_pos(1) nRows-start_pos(1)+1 nRows-start_pos(1)+1], 'r');
end

function player = drawPlayer(pos, nRows)
    player = fill([pos(2)-1 pos(2) pos(2) pos(2)-1], ...
                  [nRows-pos(1) nRows-pos(1) nRows-pos(1)+1 nRows-pos(1)+1], 'b');
end

function new_pos = nextPosition(pos, move)
    switch move
        case 'left'
            new_pos = [pos(1), pos(2)-1];
        case 'right'
            new_pos = [pos(1), pos(2)+1];
        case 'up'
            new_pos = [pos(1)-1, pos(2)];
        case 'down'
            new_pos = [pos(1)+1, pos(2)];
    end
end

function valid = isValidMove(pos, maze, nRows, nCols)
    r = pos(1); c = pos(2);
    valid = r>=1 && r<=nRows && c>=1 && c<=nCols && maze(r,c)==0;
end

function animateMove(player, old_pos, new_pos, nRows)
    steps = 10;
    for t = linspace(0,1,steps)
        x = (1-t)*(old_pos(2)-1) + t*(new_pos(2)-1);
        y = (1-t)*(nRows-old_pos(1)) + t*(nRows-new_pos(1));
        set(player, 'XData', [x x+1 x+1 x], 'YData', [y y y+1 y+1]);
        pause(0.05);
    end
end