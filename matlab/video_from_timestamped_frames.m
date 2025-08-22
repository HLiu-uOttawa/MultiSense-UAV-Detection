%% images_to_video_by_timestamp.m
% Combine images into a video using timestamps embedded in filenames.
% - Filename format example: frame_00000_2025-08-11_15-21-27.050.png
% - Extract timestamp "2025-08-11_15-21-27.050" (yyyy-MM-dd_HH-mm-ss.SSS)
% - Sort by timestamps
% - Determine base FPS from median frame interval
% - Use frame repetition to approximate variable intervals
% - Output: MP4 video with constant FPS

%% Select input folder and output file
imgDir = uigetdir(pwd, 'Select the folder containing images');
if imgDir==0, error('No folder selected.'); end
[videoFile, videoPath] = uiputfile('*.mp4', 'Save output video as', fullfile(imgDir, 'output.mp4'));
if isequal(videoFile,0), error('No output file selected.'); end
outPath = fullfile(videoPath, videoFile);

%% Collect images
imgExts = {'.png','.jpg','.jpeg','.bmp','.tif','.tiff'};
files = [];
for k = 1:numel(imgExts)
    files = [files; dir(fullfile(imgDir, ['*' imgExts{k}]))]; %#ok<AGROW>
end
files = files(~[files.isdir]);
if isempty(files), error('No images found in: %s', imgDir); end

%% Extract timestamps and sort
pat = '\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.\d{3}';
tstr = strings(numel(files),1);
for i = 1:numel(files)
    m = regexp(files(i).name, pat, 'match', 'once');
    if isempty(m)
        error('Filename "%s" does not contain a timestamp like yyyy-MM-dd_HH-mm-ss.SSS', files(i).name);
    end
    tstr(i) = string(m);
end
t = datetime(tstr, 'InputFormat','yyyy-MM-dd_HH-mm-ss.SSS');

[ t, sortIdx ] = sort(t);      % sort by time
files = files(sortIdx);

%% Compute frame intervals and base FPS
if numel(t) == 1
    base_fps = 30; % default if only one frame
else
    dts = seconds(diff(t));     % intervals in seconds
    dts = dts(dts > 0);         % ignore zero/negative gaps
    if isempty(dts)
        base_fps = 30;
    else
        dt_med = median(dts);   % robust median interval
        base_fps = max(1, round(1/dt_med));
    end
end
fprintf('Base FPS = %d (from median interval)\n', base_fps);

%% Prepare VideoWriter
firstImg = imread(fullfile(files(1).folder, files(1).name));
if ndims(firstImg)==2
    firstImg = repmat(firstImg, [1,1,3]); % convert grayscale to RGB
end
[h, w, ~] = size(firstImg);

vw = VideoWriter(outPath, 'MPEG-4');
vw.FrameRate = base_fps;
open(vw);

%% Write the first frame with repetition (duration until the 2nd frame)
if numel(t) > 1
    dt_first = seconds(t(2) - t(1));
    if dt_first <= 0
        dt_first = 1/base_fps;
    end
else
    dt_first = 1/base_fps;
end
rep_first = max(1, round(dt_first * base_fps));
for r = 1:rep_first
    writeVideo(vw, firstImg);
end

%% Main loop
MAX_REPEAT_FACTOR = 10;  % cap very large gaps
if numel(t) > 2
    dt_med = median(seconds(diff(t)));
else
    dt_med = 1/base_fps;
end
rep_med = max(1, round(dt_med * base_fps));

for i = 2:numel(files)
    I = imread(fullfile(files(i).folder, files(i).name));
    if ndims(I)==2
        I = repmat(I, [1,1,3]); % grayscale to RGB
    end
    if size(I,1) ~= h || size(I,2) ~= w
        I = imresize(I, [h, w]);
    end

    % Timestamp overlay (requires Computer Vision Toolbox for insertText)
    ts_str = datestr(t(i), 'yyyy-mm-dd HH:MM:SS.FFF'); % mm=month, MM=minute
    try
        I = insertText(I, [10,10], ts_str, 'FontSize', 18, ...
            'BoxOpacity', 0.1, 'TextColor', 'red');
    catch
        % 如果没有 Computer Vision Toolbox，可选择跳过叠字或自行实现文本绘制
    end

    % Duration of this frame = time until the NEXT frame (last frame uses median)
    if i < numel(files)
        dt = seconds(t(i+1) - t(i));
        if dt <= 0, dt = dt_med; end
    else
        dt = dt_med;
    end

    rep = max(1, round(dt * base_fps));
    rep = min(rep, MAX_REPEAT_FACTOR * rep_med);

    for r = 1:rep
        writeVideo(vw, I);
    end
end

close(vw);
fprintf('Done. Saved video: %s\n', outPath);
