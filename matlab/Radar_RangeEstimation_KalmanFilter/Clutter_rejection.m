
function [rangeFFT, range_axis, time_vector] = Clutter_rejection(rangeFFT, range_axis, time_vector, chnidx, MBW)
% Compute number of bins and pulses
[nBins, nPulses] = size(rangeFFT);

% Low‐range elimination width (bins)
elim_bins_1 = ceil(1/ (3e8/(2*MBW)));
elim_bins_2 = ceil(4/ (3e8/(2*MBW))); 
range_bins_1 = length(range_axis) - elim_bins_1 + 1 : length(range_axis);
range_bins_2 = 1:elim_bins_2;
% Overwrite high‐range bins with low‐level noise
rangeFFT(range_bins_1, :) = (1e-4/sqrt(2)) * ( ...
    randn(length(range_bins_1), nPulses) + 1i*randn(length(range_bins_1), nPulses) ...
    );
rangeFFT(range_bins_2, :) = (1e-4/sqrt(2)) * ( ...
    randn(length(range_bins_2), nPulses) + 1i*randn(length(range_bins_2), nPulses) ...
    );
end
