%% FalconEye Recon Packet Loss Simulation - Failure Mode
% Simulates communication reliability with updated drone types

%% Drone Network Parameters
numDrones = 31; 
simTime = 3600; % Simulation time (seconds)
updateInterval = 1; % Status updates per secondfa
commsRange = 1800; % Slightly larger range for simulation (1.8 km)

% Drone positions (Skydio clustered near bridges, DJI scattered)
skydioPos = randn(16, 2) * 300 + [1600 1800]; % Bridge zone
djiPos = rand(15, 2) * 3650; % Full area
dronePositions = [skydioPos; djiPos];

% Drone types (1=Skydio 2DX, 2=DJI Matrice 350 RTK)
droneTypes = [ones(16,1); 2*ones(15,1)];

%% Packet Loss Model Parameters
% Base packet loss percentages
baseLoss = [1.5; % Skydio 2DX
            2.0]; % DJI Matrice 350 RTK

% Increased distance-based loss to push failure
distanceLossFactor = 0.0004;

% Interference zones (heavy bridge + new elevated zone)
interferenceZones = [1600 1800 600; % Bridge area
                     2600 2900 400]; % Elevated ridge

%% Simulation
packetLoss = zeros(numDrones, numDrones);
commsHistory = cell(numDrones, numDrones);

for t = 1:simTime
    % Random walk to simulate drift
    dronePositions = dronePositions + randn(numDrones, 2)*4;
    dronePositions = max(min(dronePositions, 3650), 0);
    
    distances = pdist2(dronePositions, dronePositions);
    
    % Check interference
    inInterference = false(numDrones,1);
    for z = 1:size(interferenceZones,1)
        zoneDist = sqrt((dronePositions(:,1)-interferenceZones(z,1)).^2 + ...
                        (dronePositions(:,2)-interferenceZones(z,2)).^2);
        inInterference = inInterference | (zoneDist < interferenceZones(z,3));
    end
    
    for i = 1:numDrones
        for j = i+1:numDrones
            if distances(i,j) <= commsRange
                % Packet loss formula
                loss = baseLoss(droneTypes(i)) + baseLoss(droneTypes(j)) + ...
                       distances(i,j)*distanceLossFactor;
                
                if inInterference(i) || inInterference(j)
                    loss = loss * 1.5;  % Stronger penalty
                end
                
                loss = loss * (0.9 + 0.2*rand()); % Add 10–30% fluctuation
                loss = min(loss, 10.0); % Cap for realism
                
                % Store
                packetLoss(i,j) = loss;
                if isempty(commsHistory{i,j})
                    commsHistory{i,j} = [t loss];
                else
                    commsHistory{i,j} = [commsHistory{i,j}; t loss];
                end
            end
        end
    end

    if mod(t, 600) == 0
        fprintf('Time: %d/%d sec | Active Links: %d\n', ...
                t, simTime, sum(distances(:)<=commsRange)/2);
    end
end

%% Visualization
figure;

% Drone types and interference zones
subplot(2,1,1);
hold on;
scatter(dronePositions(:,1), dronePositions(:,2), 50, droneTypes, 'filled');
title('Drone Network (Skydio=1, DJI=2)');
xlabel('X Position (m)'); ylabel('Y Position (m)');
colorbar('Ticks',[1 2],'TickLabels',{'Skydio 2DX','DJI Matrice 350 RTK'});

for z = 1:size(interferenceZones,1)
    rectangle('Position',[interferenceZones(z,1:2)-interferenceZones(z,3), ...
              2*interferenceZones(z,3)*[1 1]], ...
              'Curvature',[1 1], 'EdgeColor','r', 'LineWidth',1.5);
end
axis equal;

% Packet loss histogram
subplot(2,1,2);
activeLinks = packetLoss(packetLoss > 0);
histogram(activeLinks, 20, 'FaceColor', [0 0.4470 0.7410]);
title(sprintf('Packet Loss Distribution (Mean: %.2f%%)', mean(activeLinks)));
xlabel('Packet Loss Percentage'); ylabel('Number of Links');
grid on;

%% Helper Function
function str = droneTypeToString(type)
    switch type
        case 1
            str = 'Skydio 2DX';
        case 2
            str = 'DJI Matrice 350 RTK';
        otherwise
            str = 'Unknown';
    end
end

%% Final Report
activeLinks = packetLoss(packetLoss > 0);

[counts, bins] = histcounts(activeLinks, 20);
[~, idx] = max(counts);
modeRange = sprintf('%.1f%%-%.1f%%', bins(idx), bins(idx+1));

meanPass = mean(activeLinks) < 4;
medianPass = median(activeLinks) < 4;
designStatus = 'FAIL';

recommendations = {};
if baseLoss(2) > baseLoss(1)
    recommendations{end+1} = 'Consider using more Skydio 2DX drones in core communication links';
end
if any(activeLinks > 4)
    recommendations{end+1} = 'Reduce distances between critical DJI drones or deploy repeaters';
end

[bestLoss, bestIdx] = min(activeLinks);
[worstLoss, worstIdx] = max(activeLinks);
[i_best, j_best] = ind2sub(size(packetLoss), find(packetLoss == bestLoss, 1));
[i_worst, j_worst] = ind2sub(size(packetLoss), find(packetLoss == worstLoss, 1));

best_pos1 = dronePositions(i_best,:); best_pos2 = dronePositions(j_best,:);
worst_pos1 = dronePositions(i_worst,:); worst_pos2 = dronePositions(j_worst,:);

best_distance = norm(best_pos1 - best_pos2);
worst_distance = norm(worst_pos1 - worst_pos2);
best_in_interference = inInterference(i_best) || inInterference(j_best);
worst_in_interference = inInterference(i_worst) || inInterference(j_worst);

% --- Print Report ---
fprintf('\n=== FALCONEYE RELIABILITY REPORT ===\n');
fprintf('Total Active Links: %d\n', numel(activeLinks));
fprintf('\n--- Network Performance ---\n');
fprintf('Mean Packet Loss: %.2f%%\n', mean(activeLinks));
fprintf('Median Packet Loss: %.2f%%\n', median(activeLinks));
fprintf('Most Common Loss Range: %s\n', modeRange);
fprintf('\n--- Extremes ---\n');
fprintf('Best Link: %.2f%% loss\n', bestLoss);
fprintf('   Between Drone %d (%s) and Drone %d (%s)\n', ...
        i_best, droneTypeToString(droneTypes(i_best)), ...
        j_best, droneTypeToString(droneTypes(j_best)));
fprintf('   Distance: %.1fm | Interference: %s\n', best_distance, string(best_in_interference));
fprintf('Worst Link: %.2f%% loss\n', worstLoss);
fprintf('   Between Drone %d (%s) and Drone %d (%s)\n', ...
        i_worst, droneTypeToString(droneTypes(i_worst)), ...
        j_worst, droneTypeToString(droneTypes(j_worst)));
fprintf('   Distance: %.1fm | Interference: %s\n', worst_distance, string(worst_in_interference));
fprintf('\n--- Compliance ---\n');
fprintf('Links <4%% Loss: %d (%.1f%%)\n', sum(activeLinks < 4), 100*sum(activeLinks < 4)/numel(activeLinks));
fprintf('Links ≥4%% Loss: %d (%.1f%%)\n', sum(activeLinks >= 4), 100*sum(activeLinks >= 4)/numel(activeLinks));
fprintf('Design Status: %s \n', designStatus);
fprintf('\n--- Recommendations ---\n');
for i = 1:length(recommendations)
    fprintf('• %s\n', recommendations{i});
end
fprintf('\n==============================\n');
