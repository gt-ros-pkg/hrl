num_s_vals = 1;
one_reconst_num = 4;
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
mean_reconst_sq_err = mean(sum(nfcs_reconst_err.^2, 1) ./ 100)
