# DroneWatch

## 🌟 Overview
Drone Watch displays an **optimized drone surveillance system** leveraging **Ant Colony Optimization (ACO)** algorithms to autonomously patrol the Lewiston-Queenston Bridge area. This system addresses critical **border security challenges** with:

- **Real-time path optimization** for different drone types in each design 
- **Packet loss simulation** ensuring <4% communication loss threshold
- **Terrain-aware routing** prioritizing water/road surveillance
- **Hotspot reinforcement** for high-risk zones

This is a simulation that my team later used to select the best design (one that had high communication reliability) based on remaining under a 4% threshold for packet loss (the metric used to determine how much communication data is lost between drones and from drones to their control station). This project essentially points to system reliability engineering through packet loss modeling, clean architecture with configurable parameters, and direct application to international border security. 
![image](https://github.com/user-attachments/assets/cebbd6ea-8a86-4e59-a5e8-9e0bcc5c042d)


## 🚀 Key Features

### 1. Intelligent Path Planning
% ACO Parameters with drone-specific behaviors
alpha = 1.6;  % Pheromone weight 
beta = 2.5;   % Heuristic weight
evaporationRate = 0.12; % Persistent surveillance

- **Drone-specific routing**: For example, the selected solution has: 
  - Autel: Prefers roads/open areas (α=1.6, β=2.25)
  - Jouav: Water-optimized (α=1.12, β=4.5) 
  - DJI: Agile hotspot response (α=2.4, β=1.75)

### 2. Communication Reliability Engine
Used for operators to determine whether the design maintains communication reliability, where the weakest performing drones are, and more - so that the system can be improved for future iterations. 
![image](https://github.com/user-attachments/assets/bfbddb0e-cb94-4429-93f3-51f8d580941b)
![image](https://github.com/user-attachments/assets/9cade35e-df11-490d-802a-8f77eb5f1d33)
![image](https://github.com/user-attachments/assets/56289f1d-ca4f-449c-b649-3a528043d37f)

- **3 simulation variants** testing different drone fleets
- **Dynamic interference modeling** for bridge/river zones
- **Distance-aware loss calculations** with hardware specs

### 3. Production-Grade Visualization
![image](https://github.com/user-attachments/assets/565a954c-6338-4ba0-acc2-0bb7ba95823a)
- **Terrain layers**: Land (dark green), water (blue), roads (gray)
- **Interactive compass**: 43°09'04"N 79°02'35"W orientation
- **Hotspot markers**: Priority surveillance zones

## 🔧 Setup & Usage

1. **Requirements**:
   - MATLAB R2021a+
   - 4GB RAM minimum

2. **Run Simulations**:
```matlab
% Run path planning
>> ACOAlgoDroneWatch

% Test comms reliability
>> packetLossSimulator_FalconEye
```

3. **Customize Parameters**:
```matlab
% In ACOAlgoDroneWatch.m:
surveillanceArea = 365000;  % Modify for new regions
hotspots = [x1,y1; x2,y2]; % Update surveillance zones
```
