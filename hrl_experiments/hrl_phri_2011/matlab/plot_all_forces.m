tool = 'wipe_finger';
tool_place = 'cheek';
pr2_subjs = [1, 2, 4, 5, 6, 7, 8, 10];
subj_ind = 1;
for subj_id=pr2_subjs
    file = sprintf('sub%d_%s_%s_processed_norms.mat', subj_id, tool, tool_place);
    load(file);
    subplot(5, 2, subj_ind);
    plot(force_mag);
    subj_ind = subj_ind + 1;
end
