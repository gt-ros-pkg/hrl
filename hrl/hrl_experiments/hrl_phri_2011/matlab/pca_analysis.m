num_s_vals = 1;
one_reconst_num = 1;
mean_cent = repmat(mean(nfcs_arr{one_reconst_num}), normal_len, 1);
[u, s, vd] = svd(nfcs_arr{one_reconst_num} - mean_cent);
s_trunc = s;
s_trunc([num_s_vals+1:end], [num_s_vals+1:end]) = 0;
nfcs_one_reconst = u * s_trunc * vd' + mean_cent;

mean_cent_combo = repmat(mean(nfcs_combo), normal_len, 1);
[u, s, vd] = svd(nfcs_combo - mean_cent_combo);
s_trunc = s;
s_trunc([num_s_vals+1:end], [num_s_vals+1:end]) = 0;
nfcs_reconst = u * s_trunc * vd' + mean_cent_combo;
nfcs_reconst_err = nfcs_reconst - nfcs_combo;
mean_reconst_sq_err = mean(sum(nfcs_reconst_err.^2, 1) ./ normal_len)
max_reconsts = max(nfcs_reconst);
force_normized_streams = nfcs_combo;
for i=1:length(combo_inds)
    max_reconsts_means(i) = mean(max_reconsts(combo_inds{i}));
    max_reconsts_stds(i) = std(max_reconsts(combo_inds{i}));
    force_normized_streams(:, combo_inds{i}) = force_normized_streams(:, combo_inds{i}) ./ ...
                           repmat(max_reconsts_means(i), normal_len, length(combo_inds{i}));
end
