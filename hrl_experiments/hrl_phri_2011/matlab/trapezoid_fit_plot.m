
colors = ['m', 'c', 'r', 'g', 'b', 'k'];

subj_ind = 1;
for wipe_ind=[1:size(fcs_arr{subj_ind},2)]
    wipe_ind
    wipe = fcs_arr{subj_ind}(:,wipe_ind)';
    wipe = regression_filter(wipe, 10, 2, 0.35);
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
    single_mean_sqr_err(wipe_ind) = resnorm/length(wipe);
    single_mean_resid(wipe_ind) = sum(abs(residual))/length(wipe);
    single_mean_sqr_err_mean(wipe_ind) = sum((mean(wipe) - wipe).^2)/length(wipe);
    single_mean_resid_mean(wipe_ind) = sum(abs(mean(wipe) - wipe))/length(wipe);
    subplot(1, 2, 1);
    plot(linspace(0,length(wipe)/100,length(wipe)),wipe, colors(mod(wipe_ind-1, length(colors))+1));
    hold on;
    subplot(1, 2, 2);
    plot(linspace(0,length(wipe)/100,length(wipe)),trapezoid(params,x),  colors(mod(wipe_ind-1, length(colors))+1));
    hold on;
end
