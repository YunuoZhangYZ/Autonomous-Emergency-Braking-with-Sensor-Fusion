function [scenario, egoVehicle] = scenario_05_AEB_CCRb_2_initialGap_40m()
% scenario_05_AEB_CCRb_2_initialGap_40m Returns the drivingScenario defined
% in the Designer

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.4 (R2022a).

% Copyright 2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 10, 'SampleTime', 0.05);

% Add all road segments
roadCenters = [0 0 0;
    1000 0 0];
roadWidth = 10;
road(scenario, roadCenters, roadWidth, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [20 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Vehicle UnderTest');
waypoints = [20 0 0;
    160 0 0];
speed = 13.89;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
globalVehicleTarget = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [64.7 0 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Global Vehicle Target');
waypoints = [64.7 0 0;
    90 0 0;
    138.23 0 0];
speed = [13.89;13.89;0];
trajectory(globalVehicleTarget, waypoints, speed);

