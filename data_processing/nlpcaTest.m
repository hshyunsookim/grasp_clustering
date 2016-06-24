clear, clc, close all

load('M.mat')
labels = M(:,end);
N = M(M(:,end)==0, 1:end-1);
M = M(M(:,end)==1, 1:end-1);

X = [M];
% save('X','X')
% [X, W, mu] = prewhiten(X);

data = X';

% % generate circular data
% t=linspace(-pi , +pi , 100);  % angular value t=-pi,...,+pi
% data = [sin(t);cos(t)];       % circle
% data = data + 0.2*randn(size(data));    % add noise

% nonlinear PCA (circular PCA, inverse network architecture)
[pc,net,network]=nlpca(data, 3);
              
% plot components
nlpca_plot(net)