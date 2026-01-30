function group15project()
clc; clear; close all;
tic; % Start a timer for optional debug printing frequency control

%% 1. USER CONFIGURATION
YOUR_WEBCAM_NAME = 'Creative Live! Cam Sync 1080p V2'; % Confirm your webcam name
ARDUINO_COM_PORT = 'COM3'; % Confirm your Arduino COM port

%% 2. SETUP: Webcam, Arduino, Detectors
try
    cam = webcam(YOUR_WEBCAM_NAME);
    cam.Resolution = '1280x720'; % Set resolution (adjust as supported by your camera)
    camResolution = cam.Resolution;
    disp(['Camera Resolution: ' camResolution]);
catch ME
    error("Could not initialize camera '%s'. Use webcamlist to confirm the name and supported resolutions. Error: %s", ...
        YOUR_WEBCAM_NAME, ME.message);
end

disp("Step out of frame capturing background in 2 s...");
pause(2);

bg = zeros(size(snapshot(cam)), 'double');
for bgFrameIdx = 1:5
    bg = bg + double(snapshot(cam));
    pause(0.1);
end
backgroundFrame = uint8(bg/5);
grayBG = rgb2gray(backgroundFrame);
disp("Background captured.");

% Blob analysis for motion detection (identifying potential objects)
blobAnalyzer = vision.BlobAnalysis( ...
    'BoundingBoxOutputPort', true, ...
    'AreaOutputPort', false, ...
    'CentroidOutputPort', false, ...
    'MinimumBlobArea', 3000, ... % Adjust minimum area as needed for object size
    'ExcludeBorderBlobs', true);

try
    s = serialport(ARDUINO_COM_PORT, 9600);
    configureTerminator(s, "LF"); % Configure terminator for consistent communication
    pause(2); % Give Arduino time to initialize
    disp("Arduino connected.");
catch ME
    s = []; % Run without servo control if connection fails
    warning('group15project:SerialPortError', ...
        "Could not open serial port '%s'. Running without servo control. Error: %s", ...
        ARDUINO_COM_PORT, ME.message);
end

%% 3. PARAMETERS & STATE
% PID constants (tuned for responsiveness and stability)
Kp = 0.2;
Ki = 0.003;
Kd = 0.008;

% Initial servo angles (camera pointing straight ahead)
panAngle = 90;
tiltAngle = 90;

% Particle Filter parameters
NUM_PARTICLES     = 200; % Number of particles for robust tracking
MOTION_STD        = 20;  % Particle motion std (spread)
MEASUREMENT_STD   = 0.2; % Measurement likelihood std (histogram strictness)
CONFIDENCE_THRESH = 0.4; % Mean weight threshold to declare tracking lost

% System state variables
isTracking    = false; % Tracking mode flag
modelHist     = [];    % Histogram model of selected object
particles     = [];    % Particle set
targetBbox    = [];    % Bounding box of tracked object
detectedBboxes= [];    % Bounding boxes from motion detection (for selection)
lastFrame     = [];    % Last captured frame

% Graphics handles
h_click_marker = [];

%% 4. DISPLAY SETUP
fig = figure('Name', 'Motion + Particle Filter', 'WindowState', 'maximized');
fig.NumberTitle = 'off';
fig.MenuBar = 'none';

h_main = imshow(backgroundFrame);
set(gca, 'DataAspectRatioMode', 'manual', 'DataAspectRatio', [1 1 1]);
set(gca, 'XLim', [0.5, size(backgroundFrame, 2) + 0.5], ...
         'YLim', [0.5, size(backgroundFrame, 1) + 0.5]);

hold on;
h_particles     = plot(0, 0, 'g.', 'MarkerSize', 8);
h_bbox_tracking = rectangle('Position', [0 0 0 0], 'EdgeColor', 'g', 'LineWidth', 3);
h_center_dot    = plot(0, 0, 'r+', 'MarkerSize', 10, 'LineWidth', 2);

h_click_marker  = plot(0, 0, 'co', 'MarkerSize', 15, 'LineWidth', 3, 'Visible', 'off');
hold off;

set(fig, 'WindowButtonDownFcn', @selectTarget);

disp("Running... press q in the figure to quit.");

%% 5. MAIN LOOP
while true
    if ~ishandle(fig)
        disp('Figure closed manually, exiting main loop.');
        break;
    end

    try
        frame = snapshot(cam);
    catch ME
        warning("Camera error during snapshot: %s", ME.message);
        pause(0.1);
        continue;
    end

    lastFrame = frame;

    if isTracking
        runParticleFilter();
    else
        runMotionDetection();
    end

    if ishandle(h_main)
        drawnow('limitrate');
    else
        disp('Main image handle invalid, exiting main loop.');
        break;
    end

    % Quit on 'q'
    try
        if ~ishandle(fig)
            disp('Figure became invalid during character check. Exiting gracefully.');
            break;
        end
        currentCharacter = get(fig, 'CurrentCharacter');
        if ~isempty(currentCharacter) && lower(currentCharacter) == 'q'
            disp('''q'' pressed, exiting.');
            break;
        end
    catch ME_getChar
        if strcmp(ME_getChar.identifier, 'MATLAB:hg:InvalidObject') || ...
           strcmp(ME_getChar.identifier, 'MATLAB:HandleGraphics:InvalidHandle')
            disp('Figure was closed unexpectedly during character check. Exiting gracefully.');
        else
            warning('group15project:CharacterCheckError', ...
                'Unexpected error during character check: %s', ME_getChar.message);
        end
        break;
    end
end

%% 6. CLEANUP
disp("Exiting.");
if exist('cam', 'var') && isvalid(cam)
    clear cam;
end
if exist('s', 'var') && ~isempty(s) && isvalid(s)
    clear s;
end
close all;

%% === NESTED FUNCTIONS ===

function runMotionDetection()
    if ~ishandle(fig), return; end

    if all(ishandle([h_particles, h_bbox_tracking, h_center_dot, h_click_marker]))
        set([h_particles, h_bbox_tracking, h_center_dot, h_click_marker], 'Visible', 'off');
    end

    grayCurr = rgb2gray(lastFrame);
    diffF = imabsdiff(grayCurr, grayBG);

    mask = diffF > 20;
    mask = imopen(mask, strel('rectangle', [3, 3]));
    mask = imclose(mask, strel('rectangle', [15, 15]));
    mask = imfill(mask, 'holes');

    detectedBboxes = blobAnalyzer.step(mask);

    displayFrame = lastFrame;
    if ~isempty(detectedBboxes)
        displayFrame = insertShape(displayFrame, 'Rectangle', detectedBboxes, ...
            'Color', 'yellow', 'LineWidth', 3);
    end

    if ishandle(h_main)
        set(h_main, 'CData', displayFrame);
        title(sprintf('Detecting Motion %d blob(s). Click on an object to track.', size(detectedBboxes, 1)));
        drawnow;
    end
end

function runParticleFilter()
    if ~ishandle(fig), return; end

    if all(ishandle([h_particles, h_bbox_tracking, h_center_dot]))
        set([h_particles, h_bbox_tracking, h_center_dot], 'Visible', 'on');
        if ishandle(h_click_marker), set(h_click_marker, 'Visible', 'off'); end
    end

    frame = lastFrame;

    particles = particles + MOTION_STD * randn(NUM_PARTICLES, 2);
    particles(:,1) = clamp(particles(:,1), 1, size(frame,2));
    particles(:,2) = clamp(particles(:,2), 1, size(frame,1));

    w = zeros(NUM_PARTICLES, 1);

    for particleIdx = 1:NUM_PARTICLES
        px = particles(particleIdx,1);
        py = particles(particleIdx,2);

        bb = [px - targetBbox(3)/2, py - targetBbox(4)/2, targetBbox(3), targetBbox(4)];
        h2 = extractHistogram(frame, bb);
        d  = bhattacharyya_distance(modelHist, h2);
        w(particleIdx) = exp(-d^2 / (2 * MEASUREMENT_STD^2));
    end

    if sum(w) == 0 || mean(w) < CONFIDENCE_THRESH
        isTracking = false;
        disp("Lost track... redetecting.");
        return;
    end

    w = w ./ sum(w);

    % Weighted estimate BEFORE resampling
    mu = sum(particles .* w, 1);

    % Resample
    idx = randsample(1:NUM_PARTICLES, NUM_PARTICLES, true, w);
    particles = particles(idx,:);

    targetBbox(1:2) = mu - targetBbox(3:4)/2;

    [panAngle, tiltAngle] = pidControl( ...
        mu, [size(frame,2)/2, size(frame,1)/2], ...
        Kp, Ki, Kd, panAngle, tiltAngle, s);

    displayFrame = insertShape(frame, 'Rectangle', targetBbox, 'Color', 'green', 'LineWidth', 3);

    if ishandle(h_main)
        set(h_main, 'CData', displayFrame);
        title('Tracking with Particle Filter');
    end
    if ishandle(h_particles)
        set(h_particles, 'XData', particles(:,1), 'YData', particles(:,2));
    end
    if ishandle(h_center_dot)
        set(h_center_dot, 'XData', mu(1), 'YData', mu(2));
    end
    drawnow;
end

function selectTarget(~, ~)
    if ~ishandle(fig), return; end

    if isTracking || isempty(detectedBboxes)
        return;
    end

    if ~ishandle(gca), return; end

    cp = get(gca, 'CurrentPoint');
    pt = cp(1,1:2);

    if ishandle(h_click_marker)
        set(h_click_marker, 'XData', pt(1), 'YData', pt(2), 'Visible', 'on');
        drawnow;
    end

    for k = 1:size(detectedBboxes, 1)
        bb = double(detectedBboxes(k,:));

        if pt(1) >= bb(1) && pt(1) <= bb(1) + bb(3) && ...
           pt(2) >= bb(2) && pt(2) <= bb(2) + bb(4)

            targetBbox = bb;
            modelHist  = extractHistogram(lastFrame, bb);

            center = [bb(1) + bb(3)/2, bb(2) + bb(4)/2];
            particles = repmat(center, NUM_PARTICLES, 1) + 10 * randn(NUM_PARTICLES, 2);

            isTracking = true;
            disp("Target acquired... tracking.");
            return;
        end
    end
end

function x = clamp(x, a, b)
    x = min(max(x, a), b);
end

function [newPanAngle, newTiltAngle] = pidControl(currentCenter, targetCenter, ...
    Kp, Ki, Kd, currentPan, currentTilt, serialObj)

    persistent prevErrorPan prevErrorTilt integralPan integralTilt

    if isempty(prevErrorPan)
        prevErrorPan  = 0;
        prevErrorTilt = 0;
        integralPan   = 0;
        integralTilt  = 0;
    end

    errorX = targetCenter(1) - currentCenter(1);
    errorY = targetCenter(2) - currentCenter(2);

    integralPan  = clamp(integralPan  + errorX, -2000, 2000);
    integralTilt = clamp(integralTilt + errorY, -2000, 2000);

    derivativePan  = errorX - prevErrorPan;
    derivativeTilt = errorY - prevErrorTilt;

    outputPan  = Kp * errorX + Ki * integralPan  + Kd * derivativePan;
    outputTilt = Kp * errorY + Ki * integralTilt + Kd * derivativeTilt;

    newPanAngle  = clamp(currentPan  + outputPan,  0, 180);
    newTiltAngle = clamp(currentTilt - outputTilt, 0, 180);

    if mod(round(toc*10), 10) == 0
        fprintf('PID Debug: Errors(X,Y)=[%.1f, %.1f] | Outputs(Pan,Tilt)=[%.2f, %.2f] | Angles(Pan,Tilt)=[%.0f, %.0f]\n', ...
            errorX, errorY, outputPan, outputTilt, newPanAngle, newTiltAngle);
    end

    if ~isempty(serialObj) && isvalid(serialObj)
        try
            writeline(serialObj, sprintf('%.0f,%.0f', newPanAngle, newTiltAngle));
        catch ME
            warning('group15project:SerialWriteError', 'Error writing to serial port: %s', ME.message);
        end
    end

    prevErrorPan  = errorX;
    prevErrorTilt = errorY;
end

function hist = extractHistogram(frame, bbox)
    numHueBins = 16;
    numSatBins = 4;
    numValBins = 4;

    x1 = max(1, round(bbox(1)));
    y1 = max(1, round(bbox(2)));
    x2 = min(size(frame,2), round(bbox(1) + bbox(3) - 1));
    y2 = min(size(frame,1), round(bbox(2) + bbox(4) - 1));

    if x1 > x2 || y1 > y2 || isempty(frame) || (x2-x1+1)*(y2-y1+1) < 100
        hist = zeros(numHueBins * numSatBins * numValBins, 1);
        return;
    end

    region = frame(y1:y2, x1:x2, :);
    hsvRegion = rgb2hsv(region);

    H = hsvRegion(:,:,1); S = hsvRegion(:,:,2); V = hsvRegion(:,:,3);
    H = H(:); S = S(:); V = V(:);

    validIdx = ~(isnan(H) | isinf(H) | isnan(S) | isinf(S) | isnan(V) | isinf(V));
    H = H(validIdx); S = S(validIdx); V = V(validIdx);

    hIdx = ceil(H * numHueBins);
    sIdx = ceil(S * numSatBins);
    vIdx = ceil(V * numValBins);

    hIdx(hIdx == 0) = 1; hIdx(hIdx > numHueBins) = numHueBins;
    sIdx(sIdx == 0) = 1; sIdx(sIdx > numSatBins) = numSatBins;
    vIdx(vIdx == 0) = 1; vIdx(vIdx > numValBins) = numValBins;

    hist3D = zeros(numHueBins, numSatBins, numValBins);
    for p = 1:length(hIdx)
        hist3D(hIdx(p), sIdx(p), vIdx(p)) = hist3D(hIdx(p), sIdx(p), vIdx(p)) + 1;
    end

    hist = hist3D(:);
    totalPixels = sum(hist);

    if totalPixels > 0
        hist = hist / totalPixels;
    else
        hist = zeros(numHueBins * numSatBins * numValBins, 1);
    end
end

function d = bhattacharyya_distance(hist1, hist2)
    if length(hist1) ~= length(hist2)
        warning('group15project:HistogramMismatch', 'Histograms must have the same length for Bhattacharyya distance.');
        d = Inf;
        return;
    end

    hist1 = abs(hist1);
    hist2 = abs(hist2);

    if sum(hist1) == 0 || sum(hist2) == 0
        d = 1;
        return;
    end

    hist1 = hist1 / sum(hist1);
    hist2 = hist2 / sum(hist2);

    BC = sum(sqrt(hist1 .* hist2));
    d = sqrt(max(0, 1 - BC));
end

end


%[appendix]{"version":"1.0"}
%---
