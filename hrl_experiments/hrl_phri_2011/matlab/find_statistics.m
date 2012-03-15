
clear
tool = 'wipe_finger';
tool_places = {'cheek', 'nose', 'chin'};

total_mean_rmse = zeros(1,100);
total_std_upr_rmse = zeros(1,100);
total_std_lwr_rmse = zeros(1,100);
all_means = NaN * zeros(4, 8);
all_stds = NaN * zeros(4, 8);
for tp_ind=[1:3]
    tool_place = tool_places{tp_ind};
    process_all;
    all_means(tp_ind, :) = max_reconsts_means;
    all_stds(tp_ind, :) = max_reconsts_stds;
    mean_err = sf_ptiles_dense - repmat(max_reconsts_means', 1, 100);
    mean_rmse = sum(mean_err.^2) / length(mean_err);
    total_mean_rmse = total_mean_rmse + mean_rmse;
    std_upr = max_reconsts_means + max_reconsts_stds;
    std_upr_err = sf_ptiles_dense - repmat(std_upr', 1, 100);
    std_upr_rmse = sum(std_upr_err.^2) / length(std_upr_err);
    total_std_upr_rmse = total_std_upr_rmse + std_upr_rmse;
    std_lwr = max_reconsts_means - max_reconsts_stds;
    std_lwr_err = sf_ptiles_dense - repmat(std_lwr', 1, 100);
    std_lwr_rmse = sum(std_lwr_err.^2) / length(std_lwr_err);
    total_std_lwr_rmse = total_std_lwr_rmse + std_lwr_rmse;
end

tool = 'shaver';
tool_place = 'all';
process_all;
all_means(4,[1,5,7]) = max_reconsts_means;
all_stds(4,[1,5,7]) = max_reconsts_stds;

[best_mean_err, best_mean_ptile] = min(total_mean_rmse)
[best_std_upr_err, best_std_upr_ptile] = min(total_std_upr_rmse)
[best_std_lwr_err, best_std_lwr_ptile] = min(total_std_lwr_rmse)

%plot statistics error trends
figure(100);
plt = subplot(3, 1, 1);
hold on;
plot(total_mean_rmse);
plot([best_mean_ptile, best_mean_ptile], [0, 100]);
set(plt, 'Xlim', [1, 100])
set(plt, 'Ylim', [0, 1.5])
plt = subplot(3, 1, 2);
hold on;
plot(total_std_upr_rmse);
plot([best_std_upr_ptile, best_std_upr_ptile], [0, 100]);
set(plt, 'Xlim', [1, 100])
set(plt, 'Ylim', [0, 1.5])
plt = subplot(3, 1, 3);
hold on;
plot(total_std_lwr_rmse);
plot([best_std_lwr_ptile, best_std_lwr_ptile], [0, 100]);
set(plt, 'Xlim', [1, 100])
set(plt, 'Ylim', [0, 1.5])

%plot force distributions
figure(101);
clf
hold on;
all_means_inds = [all_means(1,:); [1:8]];
all_means_inds_sorted = sortrows(all_means_inds', 1)';
sorted_inds = all_means_inds_sorted(2, :);
errorbar([1:8], all_means(1,sorted_inds), all_stds(1,sorted_inds), 'x');
errorbar([1:8]+0.15, all_means(2,sorted_inds), all_stds(2,sorted_inds), 'xr');
errorbar([1:8]+0.3, all_means(3,sorted_inds), all_stds(3,sorted_inds), 'xg');
errorbar([1:8]+0.45, all_means(4,sorted_inds), all_stds(4,sorted_inds), 'xm');
