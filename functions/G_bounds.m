function gb = G_bounds(robot)
%G_bounds 
% Computes the maximal value of the eingenvalues of G(q),
% for all q in [q_min,q_max], performing a sweep on the 
% allowed values of q between q_max and q_min

    NJ = robot.n_joints;
    % Size of the mesh of the sweep
    n_interval = 4;

    q_space = zeros(NJ,n_interval);

    for j =1:NJ
        q_space(j,:) = linspace(robot.qmin(j),robot.qmax(j),n_interval);
    end

    % Getting all the combination of indexes N_interval 6 to 6, with repetition
    index_permutation = permn(1:n_interval,NJ); 

    % Variables to store the maximum values found during the loop
    gb = 0;

    for i = 1:size(index_permutation,1) % For each combination
        indexes = index_permutation(i,:);
        q_sweep = zeros(NJ,1);
        for j = 1:NJ % for each joint
            q_sweep(j) = q_space(j,indexes(j));
        end

        G_sweep = ComputeGravTorque(robot, q_sweep);
        G_norm1 = norm(G_sweep,1);
        
        %  g_max
        if G_norm1 > gb
            gb = G_norm1;
        end
    end


end