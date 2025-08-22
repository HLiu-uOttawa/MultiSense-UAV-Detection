function B = movmean2d(A, winRow, winCol, varargin)
  omitnan = any(strcmpi(varargin,'omitnan'));

  if omitnan
    validMask = ~isnan(A);
    A_zeroed  = A;
    A_zeroed(~validMask) = 0;
  else
    validMask = true(size(A));
    A_zeroed  = A;
  end

  % build window
  window = ones(winRow, winCol);

  % compute sliding sums & counts
  sumVals   = conv2(A_zeroed, window, 'same');
  countVals = conv2(double(validMask), window, 'same');

  % divide, taking care of zero-counts
  B = sumVals ./ countVals;
  B(countVals==0) = NaN;  % in case window falls completely outside
end
