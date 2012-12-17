
for subj_ind=[1:length(fcs_arr)]
    for wipe_ind=[1:size(fcs_arr{subj_ind},2)]
        wipe = fcs_arr{subj_ind}(:,wipe_ind)';
        wipe = wipe(~isnan(wipe));
        x = linspace(0,1,length(wipe));
        best_resnorm = 999;
        for delta=linspace(0.05, 0.4, 20)
            [params,resnorm,residual] = lsqcurvefit(@trapezoid,[delta,1-delta,1],x,wipe,[0,0,0],[1,1,100000]);
            if resnorm < best_resnorm
                best_params = params;
                best_resnorm = resnorm;
                best_residual = residual;
            end
        end
        params = best_params; resnorm = best_resnorm; residual = best_residual;
        rise_params(subj_ind,wipe_ind) = params(1);
        fall_params(subj_ind,wipe_ind) = params(2);
        height_params(subj_ind,wipe_ind) = params(3);
        mean_sqr_err(subj_ind,wipe_ind) = resnorm/length(wipe);
        mean_resid(subj_ind,wipe_ind) = sum(abs(residual))/length(wipe);
        mean_sqr_err_mean(subj_ind,wipe_ind) = sum((mean(wipe) - wipe).^2)/length(wipe);
        mean_resid_mean(subj_ind,wipe_ind) = sum(abs(mean(wipe) - wipe))/length(wipe);
        rise_rates(subj_ind,wipe_ind) = (params(3)+0.5)/(params(1)*length(wipe)/100);
        fall_rates(subj_ind,wipe_ind) = (params(3)+0.5)/((1-params(2))*length(wipe)/100);
        if params(1)*length(wipe) < 3
            rise_rates(subj_ind,wipe_ind) = nan;
        end
        if (1-params(2))*length(wipe) < 3
            fall_rates(subj_ind,wipe_ind) = nan;
        end
    end
end
rise_params(height_params==0) = nan;
fall_params(height_params==0) = nan;
height_params(height_params==0) = nan;
mean_sqr_err(mean_sqr_err==0) = nan;
mean_resid(mean_resid==0) = nan;
mean_sqr_err_mean(mean_sqr_err_mean==0) = nan;
mean_resid_mean(mean_resid_mean==0) = nan;
rise_rates(rise_rates==0) = nan;

multipliers = (1./nanmean(height_params+0.5,2))'
