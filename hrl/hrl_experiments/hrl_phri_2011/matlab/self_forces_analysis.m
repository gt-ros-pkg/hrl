
nfcs_arr = {};
sf_ptiles = NaN * zeros([num_subs, 4]);
subj_ind = 1;
for subj_id=pr2_subjs
    file = sprintf('sub%d_%s_%s_processed_norms.mat', subj_id, tool, tool_place);
    load(file);
    subj_num = subj_ind;
    run force_contact_proc;
    subj_ind = subj_ind + 1;
end
run force_combo_proc;
run pca_analysis;
