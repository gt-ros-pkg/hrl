function [nMSEtest nMSEtr] = go_pr2( joint, fname )
data = load(fname);

X = data(:,2:15);
T = data(:,16:end);
N = size(X, 1);

% wrap continious joints
X(:,3) = mod( X(:,3) + pi, 2 * pi) - pi; 
X(:,5) = mod( X(:,5) + pi, 2 * pi) - pi; 
X(:,7) = mod( X(:,7) + pi, 2 * pi) - pi; 

% mean center data
%X = X - repmat(mean(X), N, 1);
%T = T - repmat(mean(T), N, 1);

% filter 
% windowSize = 10;
% X = filtfilt(ones(1,windowSize)/windowSize,1,X);
% T = filtfilt(ones(1,windowSize)/windowSize,1,T);
Y = T(:,joint);

dt = 0.01;
acc = (X(:,8:14) - [X(1,8:14); X(1:end-1,8:14)]) ./ dt;
%acc = (X(:,2) - [X(1,2); X(1:end-1,2)]) ./ dt;
%acc = filtfilt(ones(1,windowSize)/windowSize,1,acc);
X = [X, acc];

Ts = 0.01;
wvp=2*pi*10;
Fv_num=[1];
Fv_den=[1/wvp^2 2/wvp 1];
Fv_c=tf(Fv_num,Fv_den);
Fv_d=c2d(Fv_c,Ts,'tustin');
[B,A]=tfdata(Fv_d,'v');

X = filtfilt(B, A, X);


%%

% split into training and test set
idx = randperm(N);
split = round(N * 2/3);
Xtr = X(idx(1:split),:);
Ytr = Y(idx(1:split),:);
Xtest = X(idx(split+1:end), :);
Ytest = Y(idx(split+1:end), :);

% free memory
clear data X T Y;

% random downsampling of training data to <= 10000
N = size(Xtr, 1);
n = min(10000, N);
idx = randperm(N);
idx = idx(1:n);
Xtr = Xtr(idx, :);
Ytr = Ytr(idx, :);

% specify parameter
meanfunc = @meanConst;
covfunc = @covSEard;
likfunc = @likGauss;
hyp.cov = ones(1, 22); 
hyp.lik = log(0.1);
hyp.mean = 0;

% run minimization
hyp = minimize(hyp, @gp, -100, @infExact, meanfunc, covfunc, likfunc, downsample(Xtr,10), downsample(Ytr,10));

% prediction error on test
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, Xtr, Ytr, Xtest);

% compute nMSE
N = size(m, 1);
mse = (m - Ytest) .^ 2;
nMSEtest = sum(mse)/N/var(Ytest, 1);

% prediction error on training
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, Xtr, Ytr, Xtr);

% compute nMSE
N = size(m, 1);
mse = (m - Ytr) .^ 2;
nMSEtr = sum(mse)/N/var(Ytr, 1);

% % plot part of results
% clf;
% NTest = size(Xtest, 1);
% s = floor(rand() * (NTest - 2000));
% e = s + 200;
% 
% z = s:e; z = z'; 
% f = [m(s:e)+2*sqrt(s2(s:e)); flipdim(m(s:e)-2*sqrt(s2(s:e)),1)];
% fill([z; flipdim(z,1)], f, [7 7 7]/8, 'EdgeColor', 'w');
% hold on; plot(z, m(s:e), 'LineWidth', 2); plot(z, Ytest(s:e), 'r');

end