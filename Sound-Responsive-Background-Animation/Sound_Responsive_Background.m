% SOUND RESPONSIVE FLIPBOOK ANIMATION USING LINEAR ALGEBRA
% Author: Abhishek-Kumar-Rai5
% Date: 2025-05-29

clear; clc; close all;

%% Step 1: Load Audio File - Allow user to select their own file
disp('Please select an audio file (MP3 or WAV) from your system...');
[fileName, filePath] = uigetfile({'*.mp3;*.wav', 'Audio Files (*.mp3, *.wav)'}, ...
                                'Select an audio file for the flipbook animation');

% Check if user canceled the file selection
if isequal(fileName, 0) || isequal(filePath, 0)
    errordlg('No audio file was selected. Please run the program again and select a file.', 'File Selection Error');
    error('File selection was cancelled by the user.');
end

% Try to read the selected audio file with error handling
try
    [audioData, fs] = audioread(fullfile(filePath, fileName));
    disp(['Successfully loaded audio: "', fileName, '" from ', filePath]);
    disp(['Sample rate: ', num2str(fs), ' Hz, Duration: ', num2str(length(audioData)/fs), ' seconds']);
catch audioReadError
    errordlg(['Could not read the selected audio file. Error: ', audioReadError.message], 'Audio Read Error');
    error(['Audio file read error: ', audioReadError.message]);
end

% If stereo, convert to mono
if size(audioData, 2) > 1
    audioData = mean(audioData, 2);
    disp('Converted stereo audio to mono.');
end

% Normalize the audio
audioData = audioData / max(abs(audioData));

%% Step 2: Improved Audio Feature Extraction
% Parameters for Spectrogram
segmentLength = 1024;
overlap = 512;
window = hamming(segmentLength);
nfft = 2048;  % Higher frequency resolution

% Compute spectrogram
[S, F, T] = spectrogram(audioData, window, overlap, nfft, fs);
magnitudeSpectrum = abs(S);

% Extract multiple features for better responsiveness
% 1. Bass energy (for beats/rhythm)
bassIndices = find(F <= 200);  
bassEnergy = sqrt(mean(magnitudeSpectrum(bassIndices, :).^2));

% 2. Mid-range energy (for melody/vocals)
midIndices = find(F > 200 & F < 2000);
midEnergy = sqrt(mean(magnitudeSpectrum(midIndices, :).^2));

% 3. Transient detection (sudden changes)
energyDiff = [0, diff(bassEnergy)];
transients = max(0, energyDiff);  % Only keep positive changes

% 4. Beat detection using peak finding
[peaks, locations] = findpeaks(bassEnergy, 'MinPeakHeight', mean(bassEnergy)*1.2, ...
                              'MinPeakDistance', floor(0.3*fs/overlap));  % ~300ms between beats

% Normalize all features to [0,1] range with safety for division by zero
bassEnergy = (bassEnergy - min(bassEnergy)) / (max(bassEnergy) - min(bassEnergy) + eps);
midEnergy = (midEnergy - min(midEnergy)) / (max(midEnergy) - min(midEnergy) + eps);
transients = (transients - min(transients)) / (max(transients) - min(transients) + eps);

% Apply non-linear enhancement for more dramatic effect
bassEnergy = bassEnergy.^1.5;  % Enhance differences
transients = transients.^2;    % Emphasize sudden changes

%% Step 3: Prepare Visualization and Transformation Parameters
% Number of frames to generate
numFrames = length(bassEnergy);

% Create time mapping
frameTime = linspace(0, length(audioData)/fs, numFrames+1);
frameTime = frameTime(1:end-1);  % Remove last point

% Linear Algebra: Define base object (a 5-point star)
theta = linspace(0, 2*pi, 11);   % 10 edges + closure
r = ones(1, 11);
r(2:2:end) = 0.4;               % Inner radius

x = r .* cos(theta);
y = r .* sin(theta);

baseObject = [
    x;
    y
];


% Set up animation timing (variable speed)
minDelay = 0.01;  % Very fast motion on high energy
maxDelay = 0.3;   % Much slower motion on low energy
frameDelays = maxDelay - (bassEnergy * (maxDelay - minDelay));

% Mark frames with beats
isBeat = false(1, numFrames);
for i = 1:length(locations)
    isBeat(locations(i)) = true;
end

%% Step 4: Setup Visualization
figure('Name', ['Sound Responsive Flipbook: ', fileName], 'Position', [100 100 1000 800]);

% Flipbook animation subplot
subplot(3,1,1);
axis([-3 3 -3 3]);
axis equal;
grid on;
title('Sound-Driven Transformation', 'FontSize', 14);
hold on;
objectPlot = fill(baseObject(1,:), baseObject(2,:), 'b', 'FaceAlpha', 0.6);
xlabel('X'); ylabel('Y');
set(gca, 'XLim', [-3 3], 'YLim', [-3 3]);

% Waveform subplot
subplot(3,1,2);
t = (0:length(audioData)-1) / fs;
plot(t, audioData, 'b');
hold on;
waveIndicator = line([0, 0], [-1, 1], 'Color', 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title(['Audio Waveform: "', fileName, '"']);
set(gca, 'XLim', [0 t(end)]);

% Energy visualization subplot
subplot(3,1,3);
hold on;
plot(T, bassEnergy, 'b', 'LineWidth', 2);
plot(T, midEnergy, 'g', 'LineWidth', 1.5);
plot(T, transients, 'r', 'LineWidth', 1.5);
energyIndicator = line([0, 0], [0, 1], 'Color', 'k', 'LineWidth', 2);
legend('Bass Energy', 'Mid Energy', 'Transients');
xlabel('Time (s)');
ylabel('Normalized Energy');
title('Audio Features');
set(gca, 'XLim', [0 T(end)], 'YLim', [0 1.1]);

%% Step 5: Audio Playback with Synchronized Animation
% Create audio player with error handling
try
    player = audioplayer(audioData, fs);
    play(player);
    disp('Playing audio and starting animation...');
catch audioPlayError
    errordlg(['Could not play the audio. Error: ', audioPlayError.message], 'Audio Playback Error');
    error(['Audio playback error: ', audioPlayError.message]);
end

% Add info text about what the user is seeing
uicontrol('Style', 'text', 'String', ...
    'This flipbook animation uses linear algebra to transform shapes in response to audio. Enjoy!', ...
    'Position', [20 20 300 40], 'BackgroundColor', [1 1 1]);

% Main animation loop
currentFrame = 1;
startTime = tic;

% Create color map for visual effect
colors = [
    linspace(0, 1, numFrames)', ...    % Red channel
    linspace(1, 0, numFrames)', ...    % Green channel
    ones(numFrames, 1) * 0.8           % Blue channel
];

while currentFrame <= numFrames && isplaying(player)
    % Calculate elapsed time
    elapsedTime = toc(startTime);
    
    % Find appropriate frame for current time
    currentFrame = find(frameTime <= elapsedTime, 1, 'last');
    if isempty(currentFrame) || currentFrame > numFrames
        break;
    end
    
    % Get current energy values
    bass = bassEnergy(currentFrame);
    mid = midEnergy(currentFrame);
    trans = transients(currentFrame);
    
    % LINEAR ALGEBRA TRANSFORMATIONS
    % 1. Scaling (based on bass energy)
    scale = 0.5 + 2.5 * bass;
    
    % 2. Rotation (based on mid energy)
    theta = mid * 2 * pi;
    
    % 3. Shear (based on transients)
    shearX = trans * 0.8;
    shearY = trans * 0.5;
    
    % Create transformation matrices
    % Scaling matrix
    S = [scale 0; 0 scale];
    
    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Shear matrix
    H = [1 shearX; shearY 1];
    
    % Combined transformation matrix (order matters)
    % Matrix multiplication for combined effect
    T = R * H * S;
    
    % Apply linear transformation to base object
    transformedObject = T * baseObject;
    
    % Add beat effect (additional stretching on beats)
    if isBeat(currentFrame)
        pulseScale = 1.2 + 0.3 * sin(2*pi*5*elapsedTime);  % Pulsating effect
        transformedObject = pulseScale * transformedObject;
    end
    
    % Update plot
    subplot(3,1,1);
    set(objectPlot, 'XData', transformedObject(1,:), 'YData', transformedObject(2,:));
    
    % Set color based on frame (and add flash on beats)
    if isBeat(currentFrame)
        set(objectPlot, 'FaceColor', [1 1 1]);  % Flash white on beats
    else
        set(objectPlot, 'FaceColor', colors(currentFrame,:));
    end
    
    % Update waveform indicator
    subplot(3,1,2);
    set(waveIndicator, 'XData', [elapsedTime, elapsedTime]);
    
    % Update energy indicator
    subplot(3,1,3);
    set(energyIndicator, 'XData', [elapsedTime, elapsedTime]);
    
    % Dynamic title with current transformation info
    subplot(3,1,1);
    title(sprintf('Sound Responsive Animation (Scale: %.2f, Rotation: %.1f°)', ...
                  scale, theta*180/pi), 'FontSize', 12);
    
    % Variable delay based on sound energy (faster on high energy)
    actualDelay = frameDelays(currentFrame);
    if isBeat(currentFrame)
        actualDelay = minDelay;  % Very fast on beats
    end
    
    % Pause for visualization
    pause(actualDelay);
end

% If audio finishes before all frames shown
if ~isplaying(player) && currentFrame < numFrames
    disp('Audio playback ended before animation completed.');
else
    disp('Flipbook animation completed.');
end

% Ask if user wants to run again with a different audio file
answer = questdlg('Would you like to run the animation with a different audio file?', ...
    'Run Again?', 'Yes', 'No', 'No');
if strcmp(answer, 'Yes')
    % Clear and rerun
    close all;
    clear player;
    disp('Restarting program...');
    sound_responsive_flipbook;  % Call this script again (assuming it's saved with this name)
else
    disp('Thank you for using the Sound Responsive Flipbook Animation!');
end

% Optional: Export animation frames as images
% Uncomment below if you want to save frames for a physical flipbook
% mkdir('flipbook_frames');
% for i = 1:numFrames
%     % Create frame
%     figure('Visible', 'off', 'Position', [100 100 600 600]);
%     % Apply transformations as above
%     % Save frame
%     filename = sprintf('flipbook_frames/frame_%04d.png', i);
%     print('-dpng', '-r150', filename);
%     close;
% end