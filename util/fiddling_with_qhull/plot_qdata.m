%% Load data

input_data;
output_data; 

%% Prepare figure and plot input points

clf;
plot(N(1,:), N(2,:), 'or', 'MarkerSize', 5, 'LineWidth', 2);
axis equal;

%% Plot Voronoi graph vertices

hold on;
plot(X(1,:), X(2,:), 'ob');
hold off;

%% Plot Voronoi graph facets
hold on;
for i = 1:length(L)
    Xs = L{i};
    if (any(Xs(2,:) == -10.101))
        %continue;
    end
    line(Xs(1,:), Xs(2,:));
end
hold off;