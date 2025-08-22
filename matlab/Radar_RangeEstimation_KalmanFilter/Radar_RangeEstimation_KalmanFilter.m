close all; clc;
% clear all;
%% --- Radar & File Parameters ---
fc_start            = 24e9;
fc_stop             = 24.75e9;
delta_f             = fc_stop - fc_start;
c                   = 3e8;
ramp_time           = 1e-3;
numSamples          = 1024;
MBW                 = 750e6;
offline_data_flag   = 0;
lambda              = c / fc_stop;
d                   = 0.00625;
range_threshold     = 85;                   % meters
Threshold_dB        = 4;                    % CFAR offset
%% --- File Listing ---
parentFolderPath = uigetdir("E:\\", "Please select the radar data folder");
if ~isequal(parentFolderPath,0)
    disp("Radar data folder selected: " + parentFolderPath);
else
    error("No folder was selected. The program will now terminate.");
end

% a list (struct array) of all the .txt files
fileList    = dir(fullfile(parentFolderPath,'*.txt'));
% the number of .txt files found in the folder
nFiles      = numel(fileList);
file_datenums = nan(nFiles,1);

% ts_all:stores the string format of human-readable timestamps
% corresponding to each radar data file (or frame).
ts_all       = strings(nFiles,1);

%% --- Timestamp Extraction ---
% Parse timestamps from filenames
for i = 1:nFiles
    name = fileList(i).name;
    tok  = regexp(name,'_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2}\.\d{3})','tokens');
    if ~isempty(tok)
        dtstr = [tok{1}{1} '_' tok{1}{2}];
        file_datenums(i) = datenum(dtstr,'yyyy-mm-dd_HH-MM-SS.FFF');
        ts_all(i) = datestr(file_datenums(i),'yyyy-mm-dd HH:MM:SS.FFF');
    else
        warning('Could not parse timestamp from %s', name);
    end
    fprintf('Parsed timestamp for %s\n', name);
end
% Sort files by timestamp
[~, sortIdx] = sort(file_datenums);
fileList     = fileList(sortIdx);
ts_all       = ts_all(sortIdx);
%% --- Concatenate IQ Data ---
Rx1 = []; Rx2 = [];
for i = 1:nFiles
    fullp = fullfile(parentFolderPath, fileList(i).name);
    data = readmatrix(fullp,'NumHeaderLines',3);
    sig1 = data(:,1) + 1j*data(:,2);
    sig2 = data(:,3) + 1j*data(:,4);
    Rx1  = [Rx1; sig1./max(abs(sig1))];
    Rx2  = [Rx2; sig2./max(abs(sig2))];
    fprintf('Loaded %s\n', fileList(i).name);
end
% disp(size(Rx1)); % 204800, 1
% disp(size(Rx2)); % 204800, 1
% disp(Rx1(1)); % 0.5841 + 0.7221i
% disp(Rx2(1)); % 0.6371 + 0.5487i
%% --- Reshape into [numSamples × Frames] ---
% Convert these two long 1-D signal vectors (Rx1, Rx2) and rearrange it 
% into a 2-D matrix with shape: 
%   Rows = numSamples (samples per pulse/frame) 
%   Columns = Frames (or Pulses)
numPulses = floor(length(Rx1)/numSamples); % deciding how many complete pulses (frames)
Rx1 = reshape(Rx1(1:numPulses*numSamples), numSamples, []);
Rx2 = reshape(Rx2(1:numPulses*numSamples), numSamples, []);
disp(size(Rx1)) % 1024, 200
disp(size(Rx2)) % 1024, 200

%% --- Pre‐Compute Axes & Preallocate ---
time_vector = (1:numPulses) * ramp_time;
range_bin   = c/(2*delta_f);
range_axis  = ((numSamples/2-1):-1:0) * range_bin;
N           = numPulses;
M           = numSamples/2;

avg_rangeFFT_db     = zeros(M, N);
range_threshold_map = zeros(M, N);
fixed_mask          = false(M, N);
range_measurements  = NaN(N,1);
SNR_vector          = zeros(N,1);
%% --- Kalman‐Filter Initialization ---
A = [1 ramp_time; 0 1];
H = [1 0];
Q = [1 0; 0 1e2];
R = 4;
P = eye(2)*500;
x = [range_axis(1); 0];

range_est_kalman = NaN(N,1);
%% --- Process in Chunks of 10 Frames ---
chunk_size = 10;
nChunks    = ceil(N/chunk_size);

for chunkIdx = 1:nChunks
    idx_start = (chunkIdx-1)*chunk_size + 1;
    idx_end   = min(chunkIdx*chunk_size, N);
    idx_range = idx_start:idx_end;

    % 1) FFT & normalization
    chunkFFT1 = fft(Rx1(:,idx_range), numSamples,1) ./ numSamples;
    chunkFFT2 = fft(Rx2(:,idx_range), numSamples,1) ./ numSamples;
    rFFT1 = chunkFFT1(numSamples/2+1:end,:);
    rFFT2 = chunkFFT2(numSamples/2+1:end,:);
    rFFT1 = rFFT1 ./ max(abs(rFFT1(:)));
    rFFT2 = rFFT2 ./ max(abs(rFFT2(:)));

    % 2) Path‐loss compensation
    rFFT1 = compensatePathloss_dB(rFFT1, range_axis, 1.5);
    rFFT2 = compensatePathloss_dB(rFFT2, range_axis, 1.5);

    % 3) Clutter rejection
    [rFFT1, ~] = Clutter_rejection(rFFT1, range_axis, time_vector(idx_range), 1, MBW);
    [rFFT2, ~] = Clutter_rejection(rFFT2, range_axis, time_vector(idx_range), 2, MBW);

    % 4) Reverse for plotting convention
    rFFT1 = rFFT1(:,end:-1:1);
    rFFT2 = rFFT2(:,end:-1:1);

    % 5) Compute dB‐map
    linDiff = (abs(rFFT1)-abs(rFFT2)).^2;
    linDiff = movmean2d(linDiff,3,17);
    dbMap   = 10*log10(abs(linDiff));

    % Store the full‐map for later plotting
    avg_rangeFFT_db(:,idx_range) = dbMap;

    % 6) detection + Kalman update per pulse
    for jj = 1:numel(idx_range)
        k = idx_range(jj);
        profile     = dbMap(:,jj);
        noise_floor = median(profile);
        thresh      = noise_floor + Threshold_dB;

        range_threshold_map(:,k) = thresh;
        fixed_mask(:,k)          = profile > thresh;

        detIdx = find(profile > thresh);
        if ~isempty(detIdx)
            [~, loc] = max(profile(detIdx));
            peakBin  = detIdx(loc);
            rv       = range_axis(peakBin);
            if rv <= range_threshold
                range_measurements(k) = rv;
            end
        end

        % Kalman‐predict
        x = A*x;
        P = A*P*A' + Q;

        % Kalman‐update if measurement exists
        if ~isnan(range_measurements(k))
            z = range_measurements(k);
            y = z - H*x;
            S = H*P*H' + R;
            K = P*H' / S;
            x = x + K*y;
            P = (eye(2)-K*H)*P;
        end
        range_est_kalman(k) = H*x;

        % Estimate SNR at track
        zval = interp1(range_axis, profile, range_est_kalman(k), 'linear', -60);
        SNR_vector(k) = zval - noise_floor;
    end
end
fprintf("Finish data analysis with kalman filtering...\n")
%% --- Visualization & Saving ---

% 1) 3D mesh of the average range‐FFT
figure;
mesh(time_vector, range_axis, avg_rangeFFT_db);
view(-23,70);
xlabel('Time (s)'); ylabel('Range (m)');
title('Average Range‐FFT');
clim([-50 -15]); zlim([-60 -5]);
axis tight;
hold on;

% 2) CFAR threshold surface
[TT, RR] = meshgrid(time_vector, range_axis);
surf(TT, RR, range_threshold_map, 'FaceAlpha',0.3, ...
    'EdgeColor','none','FaceColor','r');

% 3) Animated Kalman track
plot3_handle = plot3(NaN,NaN,NaN,'ko-','MarkerFaceColor','g', ...
    'MarkerSize',4,'LineWidth',1.5);
title('Kalman‐Filtered Realtime Track on Range‐FFT ');
for k = 1:length(range_est_kalman)
    r = range_est_kalman(k);
    if r>0 && r<=max(range_axis)
        z = interp1(range_axis, avg_rangeFFT_db(:,k), r, 'linear', -60);
        set(plot3_handle, ...
            'XData',[get(plot3_handle,'XData') time_vector(k)], ...
            'YData',[get(plot3_handle,'YData') r], ...
            'ZData',[get(plot3_handle,'ZData') z]);
        drawnow limitrate;
    end
end

% Export figures to PNG (300 DPI)
saveas(gcf, strcat("Radar_detection",".png"));
print(gcf, strcat("Radar_detection"),'-dpng','-r300');



%% --- Export Range Estimates Table ---
ts_use = ts_all(1:N);
T = table( ...
    ts_use(:), ...
    range_est_kalman, ...
    SNR_vector, ...
    'VariableNames',{'Timestamp','Range_m','SNR_dB'} );
writetable(T, strcat("range_estimates",".csv"));
fprintf('Wrote range_estimates.csv with %d entries.\n', N);

%% --- Raw vs. Kalman‐Filtered Plots ---
figure; hold on;
xlim([0 N]); ylim([0 range_threshold]);
title('Without Kalman Filtering');
plot(range_measurements);
saveas(gcf, strcat("Raw_range_estimates",".png"));
print(gcf, strcat("Raw_range_estimates"),'-dpng','-r300');

figure; hold on;
xlim([0 N]); ylim([0 range_threshold]);
title('With Kalman Filtering');
plot(range_est_kalman);
saveas(gcf, strcat("Kalman_filtered_range_estimates",".png"));
print(gcf, strcat("Kalman_filtered_range_estimates"),'-dpng','-r300');

%% --- SNR Time Series ---
figure;
plot(SNR_vector);
xlabel('Frame Index'); ylabel('SNR (dB)');
title('Estimated SNR over Time');
saveas(gcf, strcat("SNR_vector",".png"));
print(gcf, strcat("SNR_vector"),'-dpng','-r300');