
files = {'wipe_finger_cheek_workspace.mat', 'wipe_finger_chin_workspace.mat', ...
         'wipe_finger_nose_workspace.mat', 'shaver_all_workspace.mat'};

for file_ind=[1:4]
    load(files{file_ind});
    i = file_ind;
    all_mean_sqr_err(i) = nanmean(mean_sqr_err(:));
    all_mean_sqr_err_mean(i) = nanmean(mean_sqr_err_mean(:));
    rise_means(i) = nanmean(rise_rates(:));
    rise_stds(i) = nanstd(rise_rates(:));
    fall_means(i) = nanmean(fall_rates(:));
    fall_stds(i) = nanstd(fall_rates(:));
end
all_mean_sqr_err
all_mean_sqr_err_mean
rise_means
rise_stds
fall_means
fall_stds
