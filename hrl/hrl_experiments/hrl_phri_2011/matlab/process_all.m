clear;
num_subs = 8;
tool = 'wipe_finger';
place = 'cheek';
% parameters
force_thresh = 0.5;
time_thresh = 20;
normal_len = 100;
norm_time_thresh = 50;

pr2_subjs = [1, 2, 4, 5, 6, 7, 8, 10];

rf_ptiles = NaN * zeros([num_subs, 4]);
rf_ptiles_dense = NaN * zeros([num_subs, 9901]);
sf_ptiles_dense = NaN * zeros([num_subs, 9901]);

run self_forces_analysis;
run pr2_forces_analysis;
[sf_rf_corr_coeff, sf_rf_corr_pval] = corr(sf_ptiles(:, 3), rf_ptiles(:, 3))
