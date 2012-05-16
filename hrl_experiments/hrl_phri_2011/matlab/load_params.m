
% parameters
force_thresh = 0.5;
time_thresh = 20;
normal_len = 100;
norm_time_thresh = 50;

rf_ptiles = NaN * zeros([num_subs, 4]);
rf_ptiles_dense = NaN * zeros([num_subs, 9901]);
%sf_ptiles_dense = NaN * zeros([num_subs, 9901]);
