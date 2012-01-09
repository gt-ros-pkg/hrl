
pr2_tool = 'wipe_finger';

rfc_errs = NaN * zeros([num_subs, 4]);
rfc_err_times = NaN * zeros([num_subs, 4]);

subj_ind = 1;
for subj_id = pr2_subjs
    file = sprintf('sub%d_%s_pr2_treatment_train.mat', subj_id, pr2_tool);
    load(file);
    f_sens_mag = sqrt(f_sens_x.^2 + f_sens_y.^2 + f_sens_z.^2);
    fsx = f_sens_x(f_sens_mag > 0.5);
    f_sens_mag = f_sens_mag(f_sens_mag > 0.5);
    rf_ptiles(subj_ind, :) = prctile(f_sens_mag, [25, 50, 75, 95]);
    rf_ptiles_dense(subj_ind, :) = prctile(f_sens_mag, [1:0.01:100]);
    rfx_ptiles(subj_ind, :) = prctile(fsx, [25, 50, 75, 95]);
    rfx_mags_train{subj_ind} = NaN * zeros([1, 6000]); 
    rfx_mags_train{subj_ind}(1:length(fsx)) = fsx;

    rp_err{subj_ind} = NaN * zeros([5, 6000]);
    rp_err{subj_ind}(5, 1:length(pos_err_x)) = pos_err_x;
    
    rfx_mags{subj_ind} = NaN * zeros([4, 6000]);
    for j=1:4
        file = sprintf('sub%d_%s_pr2_treatment_%d.mat', subj_id, pr2_tool, j);
        load(file);
        f_sens_mag = sqrt(f_sens_x.^2 + f_sens_y.^2 + f_sens_z.^2);
        fsx = f_sens_x(f_sens_mag > 0.5);
        f_sens_mag = f_sens_mag(f_sens_mag > 0.5);
        %fcs_combo = NaN * zeros([10000, 1000]);
        rfx_mags{subj_ind}(j, 1:length(fsx)) = fsx;
        fsx_err = fsx - rfx_ptiles(subj_ind, j);
        fsx_bad = fsx_err(fsx_err > 0);
        rfc_errs(subj_ind, j) = sum(fsx_bad) * 0.01 / (length(f_sens_mag) * 0.01);
        rfc_err_times(subj_ind, j) = length(fsx_bad) * 0.01 / (length(f_sens_mag) * 0.01);
        
        rp_err{subj_ind}(j, 1:length(pos_err_x)) = pos_err_x;
    end
    subj_ind = subj_ind + 1;
end
