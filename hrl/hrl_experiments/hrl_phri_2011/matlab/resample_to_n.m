function [ rs ] = resample_to_n( data, num_vals )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
b = data;
tr = linspace(1, length(b), num_vals)-1;
bts = timeseries(b);
rts = resample(bts, tr);
rs = rts.data;

end

