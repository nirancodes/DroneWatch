%% FalconEye Recon - ACO Path Planning for Border Surveillance
% University of Toronto - APS111 Project
% Team 44 - Drone Watch
% Customized for Lewiston-Queenston Bridge Monitoring
%% ===================== PROJECT PARAMETERS =====================
% Surveillance area metrics
surveillanceArea = 365000;      % Total area in m² (Appendix B4)
perimeterLength = 5900;         % Perimeter in meters (5.9km)
maxDistanceFromCC = 1800;       % 1800m from furthest point
minDistanceFromCC = 650;        % 650m from closest point
% Grid parameters (1 unit ≈ 36.5m)
meterPerUnit = 36.5;           
gridSize = 100;                 % 100x100 grid
startPos = [25, 20];            % Control Center (Queenston Boat Ramp)
endPos = [65, 80];              % LQ Bridge Center Point
% Surveillance Hotspots (Canadian-side only)
hotspots = [60, 70;   % Canadian bridge approach
           40, 50];  % Riverbank area (assumed Canadian side)
% FalconEye Recon Fleet Parameters
numAnts = 40;                   % 15 DroneA, 15 DroneB, 10 DroneC
maxIter = 200;                  % Convergence iterations
alpha = 1.6;                    % Pheromone weight (enhanced for hotspots)
beta = 2.5;                     % Heuristic weight (target attraction)
evaporationRate = 0.12;         % Slow evaporation (persistent surveillance)
Q = 4;                          % Strong pheromone deposit
tau0 = 0.15;                    % Initial trail strength
%% ===================== ENVIRONMENT MODELING =====================
% Initialize grid: 1=land, 2=water, 3=road
grid = ones(gridSize);
waterAreas = zeros(gridSize);
% Expanded Niagara River - reaches full horizontal span
waterAreas(35:65, 1:end) = 1;  % Horizontal band from col 1 to end
waterAreas(40:60, 20:30) = 1;  % Add a bend for realism (optional)
grid(waterAreas == 1) = 2;
% Roads and infrastructure (Highway 405 + bridge)
grid(20:25, 15:85) = 3;         % Main highway
%grid(70:75, 60:85) = 3;         % Bridge approach
% Ensure key points are accessible
grid(startPos(1), startPos(2)) = 1;
grid(endPos(1), endPos(2)) = 1;
%% ===================== DRONE FLEET SPECS =====================
% Autel Dragonfish Pro (DroneA - Long Range)
droneA.speed = 25;              % m/s
droneA.coverage = 2000;         % 2km radius
droneA.ants = 1:15;             % First 15 ants simulate DroneA
% Jouav CW-25E (DroneB - Water Surveillance)
droneB.speed = 22;
droneB.coverage = 3000;
droneB.ants = 16:30;
% DJI Mavic 3 (DroneC - Agile Monitoring)
droneC.speed = 30;
droneC.coverage = 1000;
droneC.ants = 31:40;
%% ===================== ACO INITIALIZATION =====================
% Heuristic matrix (distance + priority zones)
[rows, cols] = ndgrid(1:gridSize, 1:gridSize);
heuristic = 1./(sqrt((rows-endPos(1)).^2 + (cols-endPos(2)).^2 + eps));
% Priority boosts
heuristic(grid == 2) = heuristic(grid == 2) * 3;  % Water areas
heuristic(grid == 3) = heuristic(grid == 3) * 2;  % Roads
% Hotspot pheromone pre-seeding
pheromone = tau0 * ones(gridSize);
for i = 1:size(hotspots,1)
   pheromone(hotspots(i,1), hotspots(i,2)) = 5 * tau0;
end
%% ===================== MAIN ACO ALGORITHM =====================
bestPath = [];
bestLength = Inf;
% Preallocate path storage
allPaths = cell(numAnts, 1);
allLengths = Inf(numAnts, 1);
for iter = 1:maxIter
   % Reset paths for each iteration
   [allPaths{:}] = deal([]);
   allLengths(:) = Inf;
  
   % Drone fleet simulation
   for ant = 1:numAnts
       currentPos = startPos;
       path = currentPos;
       visited = false(gridSize);
       visited(currentPos(1), currentPos(2)) = true;
       pathHistory = zeros(gridSize^2, 2); % Preallocated path storage
       pathCount = 1;
       pathHistory(pathCount,:) = currentPos;
      
       while ~isequal(currentPos, endPos)
           % Get terrain-aware neighbors
           [neighbors, valid] = getNeighbors(currentPos, grid, visited);
           if ~valid, break; end
          
           % Drone-specific behavior
           if ismember(ant, droneA.ants)
               % DroneA: Prefers roads and open areas
               prob = (pheromone(sub2ind(size(pheromone), neighbors(:,1), neighbors(:,2))).^alpha .* ...
                      heuristic(sub2ind(size(heuristic), neighbors(:,1), neighbors(:,2))).^(beta*0.9));
           elseif ismember(ant, droneB.ants)
               % DroneB: Strong water preference
               prob = (pheromone(sub2ind(size(pheromone), neighbors(:,1), neighbors(:,2))).^(alpha*0.7) .* ...
                      heuristic(sub2ind(size(heuristic), neighbors(:,1), neighbors(:,2))).^(beta*1.8));
           else
               % DroneC: Agile hotspot response
               prob = (pheromone(sub2ind(size(pheromone), neighbors(:,1), neighbors(:,2))).^(alpha*1.5) .* ...
                      heuristic(sub2ind(size(heuristic), neighbors(:,1), neighbors(:,2))).^(beta*0.7));
           end
          
           % Move selection
           prob = prob / sum(prob);
           nextIdx = randsample(1:size(neighbors,1), 1, true, prob);
           currentPos = neighbors(nextIdx, :);
           pathCount = pathCount + 1;
           pathHistory(pathCount,:) = currentPos;
           visited(currentPos(1), currentPos(2)) = true;
       end
      
       % Store valid paths
       if isequal(currentPos, endPos)
           allPaths{ant} = pathHistory(1:pathCount,:);
           allLengths(ant) = pathCount;
          
           % Update pheromones (Dynamic Path Planning)
           deltaTau = Q / allLengths(ant);
           for i = 1:pathCount
               r = pathHistory(i,1);
               c = pathHistory(i,2);
               pheromone(r,c) = pheromone(r,c) + deltaTau;
              
               % Extra reinforcement for hotspots
               if any(ismember([r,c], hotspots, 'rows'))
                   pheromone(r,c) = pheromone(r,c) + deltaTau * 2;
               end
           end
       end
   end
  
   % Pheromone evaporation
   pheromone = (1 - evaporationRate) * pheromone;
  
   % Elite path reinforcement
   [minLength, idx] = min(allLengths);
   if minLength < bestLength
       bestPath = allPaths{idx};
       bestLength = minLength;
       fprintf('Iter %d: Best path = %.2f km\n', iter, bestLength*0.0365);
      
       % FalconEye's ACO enhancement
       eliteBoost = Q / bestLength * 3;
       for i = 1:bestLength
           pheromone(bestPath(i,1), bestPath(i,2)) = ...
               pheromone(bestPath(i,1), bestPath(i,2)) + eliteBoost;
       end
   end
end
%% ===================== VISUALIZATION =====================
figure;
hold on;
% Environment layers with darker green (RGB: 0.2 0.6 0.2)
land = imagesc(grid);
colormap([0.2 0.6 0.2;    % Dark green = land
         0 0.5 1;         % Blue = water
         0.3 0.3 0.3]);   % Dark gray = road
% Create dummy objects for legend with updated green color
land_patch = patch(NaN, NaN, [0.2 0.6 0.2]);  % Dark green
water_patch = patch(NaN, NaN, [0 0.5 1]);
road_patch = patch(NaN, NaN, [0.3 0.3 0.3]);
% Key locations
cc = plot(startPos(2), startPos(1), 'gd', 'MarkerSize', 12, 'LineWidth', 2);
bridge = plot(endPos(2), endPos(1), 'rh', 'MarkerSize', 12, 'LineWidth', 2);
hotspotPlots = plot(hotspots(:,2), hotspots(:,1), 'yx', 'MarkerSize', 12, 'LineWidth', 2);
% Add compass for 43°09'04"N 79°02'35"W perspective with custom orientation
compassPos = [90, 15]; % Position in top-right corner
compassRadius = 8;
% Compass circle (white for contrast)
rectangle('Position',[compassPos(2)-compassRadius, compassPos(1)-compassRadius,...
                   2*compassRadius, 2*compassRadius],...
                   'Curvature',[1 1],'EdgeColor','w','LineWidth',1.5);
% Compass directions (white text)
text(compassPos(2)-compassRadius-1.5, compassPos(1), 'N',...  % Left (North)
   'HorizontalAlignment','center','FontWeight','bold','Color','r','FontSize',12);
text(compassPos(2)+compassRadius+1.5, compassPos(1), 'S',...  % Right (South)
   'HorizontalAlignment','center','FontWeight','bold','Color','w','FontSize',10);
text(compassPos(2), compassPos(1)-compassRadius-1.5, 'W',...  % Top (West)
   'HorizontalAlignment','center','FontWeight','bold','Color','w','FontSize',10);
text(compassPos(2), compassPos(1)+compassRadius+1.5, 'E',...  % Bottom (East)
   'HorizontalAlignment','center','FontWeight','bold','Color','w','FontSize',10);
% Compass needles (white with red north)
line([compassPos(2) compassPos(2)-compassRadius*0.7],...  % North needle (left - red)
    [compassPos(1) compassPos(1)],...
    'Color','r','LineWidth',2);
line([compassPos(2) compassPos(2)+compassRadius*0.5],...  % South needle (right - white)
    [compassPos(1) compassPos(1)],...
    'Color','w','LineWidth',1.5);
line([compassPos(2) compassPos(2)],...                    % West needle (top - white)
    [compassPos(1) compassPos(1)-compassRadius*0.5],...
    'Color','w','LineWidth',1.5);
line([compassPos(2) compassPos(2)],...                    % East needle (bottom - white)
    [compassPos(1) compassPos(1)+compassRadius*0.5],...
    'Color','w','LineWidth',1.5);
% Add small direction indicators (white)
text(compassPos(2)-compassRadius*0.7, compassPos(1)-compassRadius*0.7, 'NW',...
   'HorizontalAlignment','center','FontSize',8,'Color','w');
text(compassPos(2)+compassRadius*0.7, compassPos(1)-compassRadius*0.7, 'NE',...
   'HorizontalAlignment','center','FontSize',8,'Color','w');
text(compassPos(2)-compassRadius*0.7, compassPos(1)+compassRadius*0.7, 'SW',...
   'HorizontalAlignment','center','FontSize',8,'Color','w');
text(compassPos(2)+compassRadius*0.7, compassPos(1)+compassRadius*0.7, 'SE',...
   'HorizontalAlignment','center','FontSize',8,'Color','w');
% Optimal path
if ~isempty(bestPath)
   pathPlot = plot(bestPath(:,2), bestPath(:,1), 'm-', 'LineWidth', 2);
   legend([land_patch, water_patch, road_patch, cc, bridge, hotspotPlots(1), pathPlot], ...
          {'Land', 'Water', 'Roads', 'Control Center', 'LQ Bridge Endpoint (Canadian Boundary)', 'Hotspots', 'Surveillance Path'});
  
   % Command window output
   pathLengthMeters = bestLength * meterPerUnit;
   fprintf('\n=== SURVEILLANCE AREA METRICS ===\n');
   fprintf('Total surveillance area: %.1f km²\n', surveillanceArea/1e6);
   fprintf('Perimeter length: %.1f km\n', perimeterLength/1000);
   fprintf('\n=== CONTROL CENTER DISTANCES ===\n');
   fprintf('Furthest point: %.1f km\n', maxDistanceFromCC/1000);
   fprintf('Closest point: %.1f km\n', minDistanceFromCC/1000);
   fprintf('\n=== OPTIMAL PATH RESULTS ===\n');
   fprintf('Path length: %.1f meters (%.1f km)\n', pathLengthMeters, pathLengthMeters/1000);
   fprintf('Percentage of perimeter covered: %.1f%%\n', (pathLengthMeters/perimeterLength)*100);
else
   warning('No valid path found!');
   legend([land_patch, water_patch, road_patch, cc, bridge, hotspotPlots(1)], ...
          {'Land', 'Water', 'Roads', 'Control Center', 'LQ Bridge Endpoint (Canadian Boundary)', 'Hotspots'});
end
% Formatting
title({'FalconEye Recon - Optimized Surveillance Path', ...
      'Lewiston-Queenston Bridge Canadian Side'});
xlabel('North-South Distance (1 unit ≈ 36.5m)');
ylabel('East-West Distance (1 unit ≈ 36.5m)');
% Optional zoomed-in view (adjust as per your hotspot/start/end layout)
xlim([10 90]);   % East-West window
ylim([10 90]);   % North-South window
axis equal tight;
set(gca, 'YDir', 'normal');
%% ===================== HELPER FUNCTION =====================
function [neighbors, valid] = getNeighbors(pos, grid, visited)
   % 8-direction movement with terrain awareness
   moves = [-1,-1; -1,0; -1,1;
             0,-1;       0,1;
             1,-1; 1,0; 1,1];
  
   neighbors = zeros(size(moves,1), 2); % Preallocate
   validCount = 0;
  
   for m = 1:size(moves,1)
       newPos = pos + moves(m,:);
       if newPos(1)>=1 && newPos(1)<=size(grid,1) && ...
          newPos(2)>=1 && newPos(2)<=size(grid,2) && ...
          grid(newPos(1), newPos(2)) ~= 0 && ~visited(newPos(1), newPos(2))
           validCount = validCount + 1;
           neighbors(validCount,:) = newPos;
       end
   end
  
   neighbors = neighbors(1:validCount,:);
   valid = validCount > 0;
end

