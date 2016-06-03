clear, clc, close all

load('M.mat')
labels = M(:,end);
N = M(M(:,end)==0, 1:end-1);
M = M(M(:,end)==1, 1:end-1);

X = [M];
save('X','X')
% [X, W, mu] = prewhiten(X);

figure,
subplot(2,1,1), scatter3(X(:,1), X(:,2), X(:,3)); title('Original dataset')
subplot(2,1,2), scatter3(X(:,5), X(:,6), X(:,7));  drawnow

no_dims = round(intrinsic_dim(X, 'MLE'));
no_dims = 3
disp(['MLE estimate of intrinsic dimensionality: ' num2str(no_dims)]);

% [mappedX, mapping1] = compute_mapping(X, 'Isomap', no_dims);	
% mapping1.val
% figure, scatter3(mappedX(:,1), mappedX(:,2), mappedX(:,3)); title('Result of Isomap');

[mappedX1, mapping1] = compute_mapping(X, 'Autoencoder', no_dims);	
figure, scatter3(mappedX1(:,1), mappedX1(:,2), mappedX1(:,3)); title('Result of Autoencoder');
% figure, scatter(mappedX1(:,1), mappedX1(:,2)); title('Result of Autoencoder');

[mappedX2, mapping2] = compute_mapping(X, 'Laplacian', no_dims, 50);	
mapping2.val
figure, scatter3(mappedX2(:,1), mappedX2(:,2), mappedX2(:,3), 5, labels(mapping2.conn_comp)); title('Result of Laplacian Eigenmaps'); drawnow
% figure, scatter(mappedX2(:,1), mappedX2(:,2),  5, labels(mapping2.conn_comp)); title('Result of Laplacian Eigenmaps'); drawnow


%%
% Factor analysis was ok (linear plot when reduced to 2d)
% LPP with high k is ok too
% MCML is ok

%% 
recX = recon_data_from_autoenc(mapping1.network, mappedX1);
negSample = [N];
reducedNegSample = out_of_sample(negSample,mapping1);
recNegSample = recon_data_from_autoenc(mapping1.network, reducedNegSample);

%%
figure(4),clf
title('Original dataset')
subplot(2,2,1)
scatter3(X(:,1), X(:,2), X(:,3), '.', 'markeredgecolor', 'b'); hold on
scatter3(N(:,1), N(:,2), N(:,3), '.', 'markeredgecolor', 'r'), hold on

scatter3(recX(:,1), recX(:,2), recX(:,3), 'x', 'markeredgecolor', 'g'); hold on
scatter3(recNegSample(:,1), recNegSample(:,2), recNegSample(:,3), 'x', 'markeredgecolor', 'r');



subplot(2,2,2)
scatter3(X(:,5), X(:,6), X(:,7), '.', 'markeredgecolor', 'b'); hold on
scatter3(N(:,5), N(:,6), N(:,7), '.', 'markeredgecolor', 'r'), hold on
scatter3(recX(:,5), recX(:,6), recX(:,7), 'x', 'markeredgecolor', 'g'); hold on
scatter3(recNegSample(:,5), recNegSample(:,6), recNegSample(:,7), 'x', 'markeredgecolor', 'r');


subplot(2,2,3)
title('Reconstructed dataset')
scatter3(recX(:,1), recX(:,2), recX(:,3), 'x', 'markeredgecolor', 'g'); hold on
scatter3(recNegSample(:,1), recNegSample(:,2), recNegSample(:,3), 'x', 'markeredgecolor', 'r');

subplot(2,2,4)
scatter3(recX(:,5), recX(:,6), recX(:,7), 'x', 'markeredgecolor', 'g'); hold on
scatter3(recNegSample(:,5), recNegSample(:,6), recNegSample(:,7), 'x', 'markeredgecolor', 'r');

%%
figure(5), clf
for i = 1:7
subplot(3,3,i)
plot(mappedX1(:,1), X(:,i),'k.')
axis square
end

figure(6), clf
for i = 1:7
subplot(3,3,i)
plot(mappedX1(:,2), X(:,i),'k.')
axis square
end


%% 
% scatter(X,A(:,1),S,C,'Marker','s')


figure(7), clf
for i = 1:7
% normalize vector to go from zero to 1
% normValue = (X(:,i)-min(X(:,i)))./(max(X(:,i))-min(X(:,i)));

% this will do blue to red. play around with it to get the color scheme you want
% C = [normValue zeros(size(normValue)) 1-normValue];

subplot(3,3,i)
% scatter3(mappedX1(:,1), mappedX1(:,2), mappedX1(:,3), 5, C, 'fill', 'marker','s')
scatter3(mappedX1(:,1), mappedX1(:,2), X(:,i),'k.')
axis square
end
