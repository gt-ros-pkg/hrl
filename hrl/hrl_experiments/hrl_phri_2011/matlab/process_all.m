num_subs = 8;
tool = 'wipe_finger';
place = 'cheek';
% parameters
force_thresh = 0.5;
time_thresh = 20;
normal_len = 100;
norm_time_thresh = 50;


for i=1:num_subs
    file = sprintf('sub%d_%s_%s_processed_norms.mat', i, tool, place);
    load(file);
    subj_num = i;
    run force_contact_proc;
end
run force_combo_proc;
run pca_analysis;
