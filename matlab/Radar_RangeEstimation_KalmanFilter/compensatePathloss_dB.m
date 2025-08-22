function P_comp = compensatePathloss_dB(P, ranges, n)
scale_range_bins  = 2;
ranges = ranges(:);
if length(ranges) ~= size(P,1)
    error('Length of ranges vector must match number of rows in P.');
end

% build compensation factor
% this is an M×1 vector, raised to the nth power
C = [ranges(1:scale_range_bins).^ -n;ranges(scale_range_bins+1:end) .^ n];

% replicate across time dimension
P_comp = P .* C(:,ones(1,size(P,2)));
end
