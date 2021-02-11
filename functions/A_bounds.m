function [lambda_max, lambda_min] = A_bounds(robot)
%A_bounds
% For revolute joints, mi_1 and mi_2 for the lower and the upper
% bounds of the inertia matrix are constant
% [Slide 365] these scalar quantities can be defined as the minimal and maximal
% eigenvalues (lambda_min, lambda_max) of A(q), for all q in [q_min,q_max]
% Performing a sweep on the allowed values of q between q_max and q_min

NJ = robot.n_joints;
% Size of the mesh of the sweep
n_interval = 5;
q_space = zeros(NJ,n_interval);

for j =1:NJ
    q_space(j,:) = linspace(robot.qmin(j),robot.qmax(j),n_interval);
end
 
% Getting all the combination of indexes N_interval 6 to 6, with repetition
index_permutation = permn(1:n_interval,NJ); 

% Variables to store the maximum and minimum values found during the loop
eig_max = 0;
eig_min = inf;

for i = 1:size(index_permutation,1) % For each combination
    indexes = index_permutation(i,:);
    q_sweep = zeros(NJ,1);
    for j = 1:NJ % for each joint
        q_sweep(j) = q_space(j,indexes(j));
    end

    %Computing the functions we want to analyze
    A_sweep = ComputeMatInert(robot,q_sweep);
    eig_A = eig(A_sweep); % calculating the eigen values of A

    % updating eig_max, eig_min
    if max(eig_A) > eig_max
        lambda_max = max(eig_A);
    end
    if min(eig_A) < eig_min
        lambda_min = min(eig_A);
    end
end

end