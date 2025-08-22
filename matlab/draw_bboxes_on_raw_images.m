%% draw_bboxes_script_seq.m
% Overlay bounding boxes from bb_info onto images in raw, matching by order (not filename).
% - Supported formats:
%   (A) Pixel coords:
%       - xywh:  x y w h [label] [score]
%       - x1y1x2y2:  x1 y1 x2 y2 [label] [score]  (auto-detected)
%   (B) YOLO format: [class] cx cy w h [score]   (all in 0~1)
%
% Usage:
%   1) Put images in 'raw/' and text files in 'bb_info/'.
%   2) Make sure the counts are equal.
%   3) Run this script. Results are saved to 'out/'.

%% -------------------- User-configurable paths --------------------
% Select parent folder interactively
parentDir = uigetdir(pwd, 'Select the parent folder containing raw and bb_info');
if parentDir == 0
    error('No folder selected, script aborted.');
end

rawDir = fullfile(parentDir, 'raw');       % folder with input images
bbDir  = fullfile(parentDir, 'bb_info');   % folder with bbox text files
outDir = fullfile(parentDir, 'out');       % folder to save results

%% -------------------- Preparation --------------------
if ~isfolder(rawDir), error('Input image folder does not exist: %s', rawDir); end
if ~isfolder(bbDir),  error('Label folder does not exist: %s', bbDir); end
if ~isfolder(outDir), mkdir(outDir); end

imgExts = {'.jpg','.jpeg','.png','.bmp','.tif','.tiff'};
imgFiles = [];
for k = 1:numel(imgExts)
    imgFiles = [imgFiles; dir(fullfile(rawDir, ['*' imgExts{k}]))]; %#ok<AGROW>
end

txtFiles = dir(fullfile(bbDir, '*.txt'));

% Sort both lists to ensure consistent order (pair by order, not by name match)
[~, idx] = sort({imgFiles.name}); imgFiles = imgFiles(idx);
[~, idx] = sort({txtFiles.name}); txtFiles = txtFiles(idx);

if numel(imgFiles) ~= numel(txtFiles)
    error('Number of images (%d) and txt files (%d) must match!', numel(imgFiles), numel(txtFiles));
end

fprintf('Found %d images and %d txt files. Matching by order.\n', numel(imgFiles), numel(txtFiles));

%% -------------------- Main loop --------------------
for i = 1:numel(imgFiles)
    inPath  = fullfile(imgFiles(i).folder, imgFiles(i).name);
    bbPath  = fullfile(txtFiles(i).folder, txtFiles(i).name);
    [~, base, ~] = fileparts(imgFiles(i).name);
    outPath = fullfile(outDir, [base '.png']);  % unify to PNG

    I = imread(inPath);
    [H, W, ~] = size(I);

    % ---- Robust empty/invalid file check ----
    d = dir(bbPath);
    if isempty(d) || d.bytes == 0
        imwrite(I, outPath);
        fprintf('Info: empty labels -> copied image: %s  (paired with %s)\n', imgFiles(i).name, txtFiles(i).name);
        continue;
    end

    % Read label lines (skip blank-only lines)
    lines = readlines(bbPath);
    lines = lines(strlength(strtrim(lines)) > 0);

    boxes = [];
    labels = strings(0,1);

    for j = 1:numel(lines)
        tokens = split(strtrim(lines(j)));
        nums = str2double(tokens);
        isNum = ~isnan(nums);
        numVals = nums(isNum);

        % Need at least 4 numeric values to form a box
        if numel(numVals) < 4, continue; end

        % ---- Auto-detect YOLO vs Pixel ----
        % YOLO if the last four numeric values lie in [0,1]
        tail = numVals(end-3:end);
        isYOLO = all(tail >= 0 & tail <= 1);

        if isYOLO
            % YOLO: [class?] cx cy w h [score?]
            cx = tail(1); cy = tail(2); ww = tail(3); hh = tail(4);
            x = (cx - ww/2) * W;
            y = (cy - hh/2) * H;
            w = ww * W;
            h = hh * H;

            txt = "";
            if any(~isNum)
                txt = strjoin(tokens(~isNum));
            else
                if numel(numVals) > 4
                    txt = sprintf('cls=%g', numVals(1));
                end
                if numel(numVals) >= 6
                    sc = numVals(end-4);
                    if sc>=0 && sc<=1
                        if txt ~= "", txt = txt + " "; end
                        txt = txt + sprintf('p=%.2f', sc);
                    end
                end
            end

            boxes(end+1,:) = [x y w h]; %#ok<AGROW>
            labels(end+1,1) = txt; %#ok<AGROW>

        else
            % ------- Pixel mode with auto-detection: xywh vs x1y1x2y2 -------
            x1 = numVals(1); y1 = numVals(2); a3 = numVals(3); a4 = numVals(4);

            % Heuristic:
            % If a3,a4 look like bottom-right coords inside the image and > x1,y1,
            % treat as (x1,y1,x2,y2); otherwise treat as (x,y,w,h).
            is_x1y1x2y2 = (a3 > x1) && (a4 > y1) && (a3 <= W) && (a4 <= H);

            if is_x1y1x2y2
                x = x1; y = y1; w = a3 - x1; h = a4 - y1;
            else
                x = x1; y = y1; w = a3;      h = a4;
            end

            txt = "";
            if any(~isNum)
                txt = strjoin(tokens(~isNum));   % e.g., 'drone'
            else
                if numel(numVals) >= 5
                    txt = sprintf('id=%g', numVals(5));
                end
                if numel(numVals) >= 6
                    if txt ~= "", txt = txt + " "; end
                    txt = txt + sprintf('p=%.2f', numVals(6));
                end
            end

            boxes(end+1,:) = [x y w h]; %#ok<AGROW>
            labels(end+1,1) = txt; %#ok<AGROW>
        end
    end

    % ---- Draw results ----
    if ~isempty(boxes)
        % Clamp & min size
        boxes(:,1) = max(1, min(boxes(:,1), W-1));
        boxes(:,2) = max(1, min(boxes(:,2), H-1));
        boxes(:,3) = max(1, boxes(:,3));
        boxes(:,4) = max(1, boxes(:,4));

        J = insertShape(I, 'Rectangle', boxes, 'LineWidth', 2);
        % for k = 1:size(boxes,1)
        %     if strlength(strtrim(labels(k))) > 0
        %         pos = [boxes(k,1), max(1, boxes(k,2)-18)];
        %         J = insertText(J, pos, labels(k), 'FontSize', 18, 'BoxOpacity', 0.6);
        %     end
        % end
    else
        J = I;
        fprintf('Info: %s has no valid boxes. Copied original.\n', imgFiles(i).name);
    end

    imwrite(J, outPath);
end

fprintf('Done. Results saved to: %s\n', outDir);
