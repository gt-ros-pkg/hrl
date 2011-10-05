n_contact = 0;
force_contacts = NaN * zeros([length(force_mag), length(force_mag)]);
force_contacts_all = NaN * zeros(length(force_mag));
j = -1;
fca_ind = 1;
force_thresh = 0.5;
time_thresh = 20;
for i=1:length(force_mag)
    if force_mag(i) > 0.5 && j == -1
        j = i;
        n_contact = n_contact + 1;
    end
    if j ~= -1
        if force_mag(i) <= 0.5
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
normal_len = 100;
n_fcs = 1;
for i=1:n_contact
   data = force_contacts(:, i);
   data = data(~isnan(data));
   if length(data) > 30
       normize_fcs(:, n_fcs) = resample_to_n(data, 100);
       n_fcs = n_fcs + 1;
   end
end