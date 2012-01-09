clearvars -except tool tool_place total_mean_rmse total_std_upr_rmse total_std_lwr_rmse tool_places all_means all_stds tp_ind;
if ~exist('tool')
    tool = 'wipe_finger';
end
if ~exist('tool_place')
    tool_place = 'cheek';
end

if strcmp(tool, 'wipe_finger')
    pr2_subjs = [1, 2, 4, 5, 6, 7, 8, 10];
    doing_shaver = false;
else
    % shaver
    pr2_subjs = [1, 6, 8];
    doing_shaver = true;
end
tool
tool_place
pr2_subjs
num_subs = length(pr2_subjs)

% parameters
force_thresh = 0.5;
time_thresh = 20;
normal_len = 100;
norm_time_thresh = 50;

rf_ptiles = NaN * zeros([num_subs, 4]);
rf_ptiles_dense = NaN * zeros([num_subs, 9901]);
%sf_ptiles_dense = NaN * zeros([num_subs, 9901]);

run self_forces_analysis;
run pr2_forces_analysis;
[sf_rf_corr_coeff, sf_rf_corr_pval] = corr(sf_ptiles(:, 3), rf_ptiles(:, 3))
