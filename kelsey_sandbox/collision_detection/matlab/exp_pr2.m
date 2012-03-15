clc;
clear all;
max_r = 10;

fnames = { '../rosbag/circle.csv', ...
           '../rosbag/figure8.csv', ...
           '../rosbag/reaching.csv', ...
           '../rosbag/sinoid.csv'};

joint_names = {'shoulder_pan_joint', ...
               'shoulder_lift_joint', ...
               'upper_arm_roll_joint', ...
               'elbow_flex_joint', ...
               'forearm_roll_joint', ...
               'wrist_flex_joint', ...
               'wrist_roll_joint'};

for fname = fnames
           
fprintf('fname: %s\n', fname{:});
for joint = 1:length(joint_names)
    fprintf('joint: %s\n', joint_names{joint} );
    NMSETR = zeros(max_r,1);
    NMSETEST = zeros(max_r,1);
    for r = 1:max_r
        [nMSEtest nMSEtr] = go_pr2(joint, fname{:}); 
        fprintf('\t%d. nMSE (test): %6.4f \tnMSE (training): %6.4f\n', r, nMSEtest, nMSEtr);
        NMSETEST(r) = nMSEtest;
        NMSETR(r) = nMSEtr;
    end
    fprintf('\t--------------------------------------------------\n');
    fprintf('\ttest\t\tmean: %6.4f +- %6.4f, min: %6.4f, max: %6.4f\n', ...
        mean(NMSETEST), var(NMSETEST), min(NMSETEST), max(NMSETEST));
    fprintf('\ttraining\tmean: %6.4f +- %6.4f, min: %6.4f, max: %6.4f\n\n', ...
        mean(NMSETR), var(NMSETR), min(NMSETR), max(NMSETR));
end
end