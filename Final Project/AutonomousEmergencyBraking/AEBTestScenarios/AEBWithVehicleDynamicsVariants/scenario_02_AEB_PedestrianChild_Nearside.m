function [scenario, egoVehicle] = scenario_02_AEB_PedestrianChild_Nearside()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.12 (R2022a) and Automated Driving Toolbox 3.4 (R2022a).
% Generated on: 22-Oct-2021 00:16:50

% Copyright 2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('SampleTime', 0.05);

% Add all road segments
roadCenters = [-21.1 0 0;
    1000 0 0];
marking = [laneMarking('Unmarked')
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(3, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the actors
euroncappedestriantarget = actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.24, ...
    'Width', 0.298, ...
    'Height', 1.154, ...
    'Position', [32 -5 0], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Euro NCAP Pedestrian Target');
waypoints = [32 -5 0;
    32 -3 0;
    32 4 0];
speed = [0;1.15;1.15];
waittime = [2.75;0;0];
trajectory(euroncappedestriantarget, waypoints, speed, waittime);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-19.152012441838 0 0], ...
    'FrontOverhang', 0.9, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Vehicle Under Test');
waypoints = [-19.152012441838 0 0;
    -16.6 0 0;
    40 0 0];
speed = [0.2;6;6];
waittime = [0;0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 4.4, ...
    'Position', [27.4 -3.5 0], ...
    'FrontOverhang', 0.600000000000001, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Small Obstruction Vehicle');

vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [21.8 -3.5 0], ...
    'FrontOverhang', 0.9, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Large Obstruction Vehicle');

