fcs_combo = NaN * zeros([10000, 1000]);
max_len = 0;
cur_fcs = 1;
for i=1:length(fcs_arr)
    for j=1:size(fcs_arr{i}, 2)
        cur_data = fcs_arr{i}(:, j);
        cur_data = cur_data(~isnan(cur_data));
        max_len = max(max_len, size(cur_data, 1));
        fcs_combo(1:length(cur_data), cur_fcs) = cur_data;
        cur_fcs = cur_fcs + 1;
    end
end
fcs_combo = fcs_combo(1:max_len, 1:cur_fcs-1);

clear combo_inds;
nfcs_combo = NaN * zeros([normal_len, 1000]);
cur_nfcs = 1;
for i=1:length(nfcs_arr)
    cur_data = nfcs_arr{i};
    cur_inds = cur_nfcs:(size(cur_data, 2)+cur_nfcs-1);
    nfcs_combo(:, cur_inds) = cur_data;
    combo_inds{i} = cur_inds;
    cur_nfcs = cur_nfcs + size(cur_data, 2);
end

nfcs_combo = nfcs_combo(:, 1:cur_nfcs-1);
