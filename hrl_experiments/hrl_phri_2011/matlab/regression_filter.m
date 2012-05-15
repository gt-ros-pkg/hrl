function [filtered] = regression_filter(signal, bw, degree, thresh)

diff_len = size(signal)-bw;
diffs = zeros(1, diff_len);
for i=[1:diff_len]
    p = polyfit([1:bw], signal(i:i+bw-1), degree);
    pred = polyval(p, bw);
    diff = signal(i+bw) - pred;
    diffs(i) = diff;
    if nargin == 4
        if abs(diff) > thresh
            signal(i+bw) = pred;
        end
    end
end
if stats
    filtered = diffs;
else
    filtered = signal;
end

end
