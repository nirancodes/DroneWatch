%% FalconEye Recon Packet Loss Simulation
% Simulates communication reliability between DJI Matrice & Parrot Anafi drones
% No toolboxes required - pure MATLAB implementation

%% Drone Network Parameters
numDrones = 13; 
simTime = 3600; % Simulation time (seconds)
updateInterval = 1; % Status updates per second
commsRange = 1500; % Communication range in meters

% Drone types (1=DJI Matrice, 2=Parrot Anafi)
droneTypes = [ones(7,1); 2*ones(6,1)];

% Predefined positions (DJI at borders, Parrot near river)
dronePositions = [
    100 1800; 3500 1800; 1800 100; 1800 3500; 300 300; 3300 3300; 1800 1800;  % DJI
    800 2600; 850 2550; 900 2500; 950 2450; 1000 2400; 1050 2350              % Parrot
];
dronePositions = dronePositions + randn(size(dronePositions))*20;  % Add jitter

%% Packet Loss Model Parameters
baseLoss = [3.0;  % DJI Matrice 300 RTK
            5.0]; % Parrot Anafi USA

distanceLossFactor = 0.0004;  % More aggressive than before

interferenceZones = [1600 1800 500;  % Bridge
                     800 2500 400];  % River

%% Simulation Initialization
packetLoss = zeros(numDrones, numDrones);
commsHistory = cell(numDrones, numDrones);

for t = 1:simTime
    % Random walk for dynamic positioning
    dronePositions = dronePositions + randn(numDrones, 2)*5;
    dronePositions = max(min(dronePositions, 3650), 0);  % Keep within area

    distances = pdist2(dronePositions, dronePositions);
    
    % Interference check
    inInterference = false(numDrones,1);
    for z = 1:size(interferenceZones,1)
        zoneDist = sqrt((dronePositions(:,1)-interferenceZones(z,1)).^2 + ...
                        (dronePositions(:,2)-interferenceZones(z,2)).^2);
        inInterference = inInterference | (zoneDist < interferenceZones(z,3));
    end

    % Packet loss calculations
    for i = 1:numDrones
        for j = i+1:numDrones
            if distances(i,j) <= commsRange
                loss = baseLoss(droneTypes(i)) + baseLoss(droneTypes(j)) + ...
                       distances(i,j)*distanceLossFactor;
                
                if inInterference(i) || inInterference(j)
                    loss = loss * 1.2;
                end
                
                loss = loss * (0.95 + 0.1*rand());  % Add small variation
                loss = min(loss, 10.0);  % Cap at 10% (not enforcing 4% threshold)
                
                packetLoss(i,j) = loss;
                if isempty(commsHistory{i,j})
                    commsHistory{i,j} = [t loss];
                else
                    commsHistory{i,j} = [commsHistory{i,j}; t loss];
                end
            end
        end
    end

    % Progress update
    if mod(t, 300) == 0
        fprintf('Time: %d/%d sec | Active Links: %d\n', ...
                t, simTime, sum(distances(:)<=commsRange)/2);
    end
end

%% Visualization
figure;

% 1. Drone Positions
subplot(2,1,1); hold on;
scatter(dronePositions(:,1), dronePositions(:,2), 50, droneTypes, 'filled');
title('Drone Positions (DJI=1, Parrot=2)');
xlabel('X Position (m)'); ylabel('Y Position (m)');
colorbar('Ticks',[1 2],'TickLabels',{'DJI Matrice','Parrot Anafi'});

% Interference zones
for z = 1:size(interferenceZones,1)
    rectangle('Position',[interferenceZones(z,1:2)-interferenceZones(z,3), ...
                         2*interferenceZones(z,3)*[1 1]], ...
              'Curvature',[1 1], 'EdgeColor','r', 'LineWidth',1.5);
end
axis equal;

% 2. Packet Loss Histogram
subplot(2,1,2);
activeLinks = packetLoss(packetLoss > 0);
histogram(activeLinks, 20, 'FaceColor', [0.2 0.6 0.9]);
title(sprintf('Packet Loss Distribution (Mean: %.2f%%)', mean(activeLinks)));
xlabel('Packet Loss Percentage'); ylabel('Number of Links');
grid on;

%% Drone Type Conversion Helper
function str = droneTypeToString(type)
    switch type
        case 1
            str = 'DJI Matrice';
        case 2
            str = 'Parrot Anafi';
        otherwise
            str = 'Unknown';
    end
end

%% Reliability Report
activeLinks = packetLoss(packetLoss > 0);

[counts, bins] = histcounts(activeLinks, 20);
[~, idx] = max(counts);
modeRange = sprintf('%.1f%%-%.1f%%', bins(idx), bins(idx+1));

meanPass = mean(activeLinks) < 4;
medianPass = median(activeLinks) < 4;
if meanPass && medianPass
    designStatus = 'PASS';
else
    designStatus = 'FAIL';
end

recommendations = {};
if meanPass && medianPass
    recommendations{end+1} = 'Design meets requirements - maintain current configuration';
else
    if baseLoss(2) > baseLoss(1)
        recommendations{end+1} = 'Upgrade Parrot drone antennas to reduce base packet loss';
    end
    if any(activeLinks > 4)
        recommendations{end+1} = 'Reposition worst-performing drones to reduce link distances';
    end
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
fprintf(['   Between Drone %d (%s at [%.1f, %.1f]m) and Drone %d ' ...
         '(%s at [%.1f, %.1f]m)\n'], ...
        i_best, droneTypeToString(droneTypes(i_best)), best_pos1(1), best_pos1(2), ...
        j_best, droneTypeToString(droneTypes(j_best)), best_pos2(1), best_pos2(2));
fprintf('   Distance: %.1fm | Interference: %s\n', best_distance, string(best_in_interference));

fprintf('Worst Link: %.2f%% loss\n', worstLoss);
fprintf(['   Between Drone %d (%s at [%.1f, %.1f]m) and Drone %d ' ...
         '(%s at [%.1f, %.1f]m)\n'], ...
        i_worst, droneTypeToString(droneTypes(i_worst)), worst_pos1(1), worst_pos1(2), ...
        j_worst, droneTypeToString(droneTypes(j_worst)), worst_pos2(1), worst_pos2(2));
fprintf('   Distance: %.1fm | Interference: %s\n', worst_distance, string(worst_in_interference));

fprintf('\n--- Compliance ---\n');
fprintf('Links <4%% Loss: %d (%.1f%%)\n', sum(activeLinks < 4), 100*sum(activeLinks < 4)/numel(activeLinks));
fprintf('Links ≥4%% Loss: %d (%.1f%%)\n', sum(activeLinks >= 4), 100*sum(activeLinks >= 4)/numel(activeLinks));
fprintf('Design Status: %s (Both mean & median <4%%? %s)\n', designStatus, string(meanPass && medianPass));

fprintf('\n--- Recommendations ---\n');
for i = 1:length(recommendations)
    fprintf('• %s\n', recommendations{i});
end
fprintf('\n==============================\n');