function [filtered] = regression_filter(signal, bw, degree, thresh)

diff_len = length(signal)-bw;
diffs = zeros(1, diff_len);
for i=[1:diff_len-1]
    p = polyfit([1:bw], signal(i:i+bw-1), degree);
    pred = polyval(p, bw);
    diff = signal(i+bw+1) - pred;
    diffs(i) = abs(diff);
    if nargin == 4
        if abs(diff) > thresh
            signal(i+bw+1) = pred;
        end
    end
end
if nargin == 4
    filtered = signal;
else
    filtered = diffs;
end

end
