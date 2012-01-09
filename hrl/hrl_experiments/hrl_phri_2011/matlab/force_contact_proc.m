%subj_num = 1;


n_contact = 0;
force_contacts = NaN * zeros([1000, length(force_mag)]);
force_contacts_all = NaN * zeros(1, length(force_mag));
j = -1;
fca_ind = 1;
for i=1:length(force_mag)
    if force_mag(i) > force_thresh && j == -1
        j = i;
        n_contact = n_contact + 1;
    end
    if j ~= -1
        if force_mag(i) <= force_thresh
            if i-j+1 < time_thresh
                n_contact = n_contact - 1;
                fca_ind = fca_ind - (i-j);
            end
            j = -1;
        else
            force_contacts(n_contact, i-j+1) = force_mag(i);
            force_contacts_all(fca_ind) = force_mag(i);
            fca_ind = fca_ind + 1;
        end
    end
end
force_contacts = force_contacts(1:n_contact, :)';
force_contacts_all = force_contacts_all(1:fca_ind-1)';

clear normize_fcs;
n_fcs = 1;
for i=1:n_contact
   data = force_contacts(:, i);
   data = data(~isnan(data));
   if length(data) > norm_time_thresh
       normize_fcs(:, n_fcs) = resample_to_n(data, normal_len);
       n_fcs = n_fcs + 1;
   end
end

fcs_arr{subj_num} = force_contacts;
fcsa_arr{subj_num} = force_contacts_all;
nfcs_arr{subj_num} = normize_fcs;

sf_ptiles(subj_num, :) = prctile(force_contacts_all, [25, 50, 75, 95]);
sf_ptiles_dense(subj_num, :) = prctile(force_contacts_all, [1:1:100]);

