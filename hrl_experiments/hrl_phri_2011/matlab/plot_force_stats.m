
files = {'wipe_finger_cheek_workspace.mat', 'wipe_finger_chin_workspace.mat', ...
         'wipe_finger_nose_workspace.mat', 'shaver_all_workspace.mat'};
inds = {[1:9], [1:9], [1:9], [1, 3, 6, 8]};
all_means = nan*ones(9,4);
all_stds = nan*ones(9,4);
all_maxs = nan*ones(9,4);
all_targets = {};
for file_ind=[1:4]
    load(files{file_ind});
    all_means(inds{file_ind},file_ind) = nanmean(height_params+0.5,2);
    all_stds(inds{file_ind},file_ind) = nanstd(height_params+0.5,1,2);
    all_maxs(inds{file_ind},file_ind) = nanmax(height_params+0.5,[],2);
    if file_ind ~= 4
        all_targets{file_ind} = height_params+0.5;
    else
        all_targets{file_ind} = nan*ones(9,size(height_params,2));
        all_targets{file_ind}(inds{file_ind},:) = height_params+0.5;
    end
end

[sorted_means,sorted_inds] = sortrows(all_means,[1]);

sorted_inds_mean = sorted_inds(1:end-1); % remove that one outlier
figure(100)
hold off;
for i=[1:4]
    subplot(1,8,2*i-1);
    errorbar(all_means(sorted_inds,i), all_stds(sorted_inds,i), 'xb');
    hold on;
    plot(all_means(sorted_inds,i), 'xr');
    plot(all_maxs(sorted_inds,i), 'vg');
    subplot(1,8,2*i);
    errorbar([nanmean(all_means(sorted_inds_mean,i))], [nanstd(all_means(sorted_inds_mean,i))], 'xb');
    hold on;
    plot([nanmean(all_means(sorted_inds_mean,i))], 'xr');
end

figure(101)
hold off;
resampled_targets = {};
for i=[1:4]
    for j=[1:size(all_targets{i},1)]
        d = all_targets{i}(j,:);
        d = d(~isnan(d));
        rs = prctile(d,linspace(0,100,size(all_targets{i},2)));
        resampled_targets{i}(j,:) = rs;
    end
    subplot(1,8,2*i-1);
    boxplot(all_targets{i}(sorted_inds,:)', 'labels', {'', '', '', '', '', '', '', '', ''});
    subplot(1,8,2*i);
    boxplot([resampled_targets{i}(:)], 'labels', {''});
end
