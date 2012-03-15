
hist_size = 15;

for subj = 1:num_subs
    figure(subj+40)
    for treat = 1:5
        plt = subplot(5, 1, treat);
        hist(rp_err{subj}(treat,:)');
        set(plt, 'Xlim', [min(rp_err{subj}(:)), 0]);
    end
end
