clear
close all

% start and goal
theta_start = [0;0];
theta_goal = [1;1];

% initial trajectory
n = % to implement
k = % to implement
xi_0 = zeros(n, k);
xi_0_vec = % to implement

% start and goal equality constraints
A = % to implement
B = % to implement

% nonlinear optimization
options = optimoptions('fmincon','Display','iter',...
    'Algorithm','sqp','MaxFunctionEvaluations',1e5);
xi_star_vec = fmincon(@(xi) cost(xi), xi_0_vec, ...
    [], [], A, B, [], [], [], options);
xi_star = % to implement

% plot result
figure
grid on
hold on
axis equal
plot(xi_star(1,:), xi_star(2,:), 'o-',...
    'Color', [1, 0.5, 0], 'LineWidth', 3);

% cost function to minimize
function C = cost(xi)
    % hint: check the dimensions of the input xi
    % you may need to reshape xi...
end
