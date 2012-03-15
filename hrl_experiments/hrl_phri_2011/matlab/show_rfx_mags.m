hist_size = 15;
end_mult = 2.8;
total_hist = zeros(4, hist_size);
above_ptiles = zeros(5, 4);
for subj = 1:num_subs
    figure(subj)
    for treat = 1:4
        mags = rfx_mags{subj}(treat,:);
        ptile = rfx_ptiles(subj,treat);
        ptile75 = rfx_ptiles(subj,3);
        %mags_norm = mags./ptile;
        mags_norm = mags./ptile75;
        cur_hist = histc(mags_norm, linspace(0, end_mult, hist_size));
        total_hist(treat,:) = total_hist(treat,:) + cur_hist/sum(cur_hist);
        for pt_ind = 1:4
            ptile2 = rfx_ptiles(subj,pt_ind);
            pt_above = sum(mags>ptile2) / (sum(mags==mags)*num_subs);
            above_ptiles(treat,pt_ind) = above_ptiles(treat,pt_ind) + pt_above;
        end

        plt = subplot(4, 1, treat);
        hist(mags');
        hold on;
        plot([ptile,ptile], [0,1000])
        set(plt, 'Xlim', [0, max(rfx_mags{subj}(:))]);

    end
end

ptile75_avg = mean(rfx_ptiles(:,3));
figure(10)
xbar_max = end_mult * mean(rfx_ptiles(:,4));
xbar_adj = 0.5*end_mult/(hist_size-1);
for treat = 1:4
    ptile_avg = mean(rfx_ptiles(:,treat));
    %xbar = linspace(0, end_mult, hist_size) * ptile_avg;
    xbar = linspace(0, end_mult, hist_size) * ptile75_avg;
    plt = subplot(5, 1, treat);
    bar(xbar+xbar_adj, total_hist(treat,:)/num_subs);
    hold on;
    plot([ptile_avg,ptile_avg], [0,10000])
    set(plt, 'Xlim', [0, xbar_max]);
    %set(plt, 'Ylim', [0, 1.3 * max(total_hist(treat,:)/num_subs)]);
    set(plt, 'Ylim', [0, 0.28]);
end
train_total_hist = zeros(1, hist_size);
for subj = 1:num_subs
    ptile75 = rfx_ptiles(subj,3);
    mags = rfx_mags_train{subj};
    mags_norm = rfx_mags_train{subj} / ptile75;
    cur_hist = histc(mags_norm, linspace(0, end_mult, hist_size));
    train_total_hist = train_total_hist + cur_hist/sum(cur_hist);

    for pt_ind = 1:4
        ptile2 = mean(rfx_ptiles(:,pt_ind));
        pt_above = sum(mags>ptile2) / (sum(mags==mags)*num_subs);
        above_ptiles(5,pt_ind) = above_ptiles(5,pt_ind) + pt_above;
    end
end
plt = subplot(5, 1, 5);
xbar = linspace(0, end_mult, hist_size) * ptile75_avg;
bar(xbar+xbar_adj, train_total_hist/num_subs);
set(plt, 'Xlim', [0, xbar_max]);
%set(plt, 'Ylim', [0, 1.3 * max(train_total_hist/num_subs)]);
set(plt, 'Ylim', [0, 0.28]);
