function helperPlotAEBVariantResults(testIterationResult, egoNewSpeeds, newCollisionPoints)
% helperPlotAEBVariantResults plots the grid results with specific colors
% for each cells indicating the pass or fail status of closed loop
% simulation of AEBTestBench.slx with generated scenario variants. This
% function assumes that the ego_velocity and collision signals were logged
% at the time of simulation of "AEBTestBench.slx"
%
% This function requires the Simulink test iteration results, ego vehicle
% new speeds, and new collision points that are used to generate
% scenario variants.
%
% Inputs:
% testIterationResult -  Output of getIterationResults().
% egoNewSpeeds    -  Array of ego vehicle speeds used for scenario
%                        variant generation.
% newCollisionPoints  -  Array of collision points used for scenario
%                        variation.
%
% Example of calling the function:
%
%  tf = sltest.testmanager.TestFile...
%     ('AEBScenarioVariationTest');
% resultset = run(tf);
% tfr = getTestFileResults(resultset);
% tsr = getTestSuiteResults(tfr);
% tcr = getTestCaseResults(tsr);
% tir = getIterationResults(tcr);
%
% egoNewSpeeds = [50 45 40 35 30 25 20 15 10]/3.6; % 1 m/s = 3.6 km/hr
% newCollisionPoints = [0.2 0.3 0.5 0.7];
%
% helperPlotAEBVariantResults(tir, egoNewSpeeds, newCollisionPoints)
%
% This is a helper function for example purposes and may be modified in
% the future.

% Copyright 2022 The MathWorks, Inc.

% Define the speed reduction table.
% Vehicle under Test Speeds
vutSpeeds = [50;45;40;35;30;25;20;15;10];

% Impact speeds where green color ends. This means that, for the vehicle
% under test, its speed at the end of collision if present (or
% simulation) is less than this speed, then the color of that cell in the
% output grid would be green. The number elements should be same as that of
% vut speeds array.
greenEndSpeeds = [5;5;5;5;5;5;1;1;1];

% Impact speeds where yellow color ends. This means that, for the vehicle
% under test, its speed at the end of collision if present (or
% simulation) is less than this speed, then the color of that cell in the
% output grid would be yellow. The number elements should be same as that of
% vutSpeeds array.
yellowEndSpeeds = [15;15;15;15;15;NaN;NaN;NaN;NaN];

% Impact speeds where orange color ends. This means that, for the vehicle
% under test, its speed at the end of collision if present (or
% simulation) is less than this speed, then the color of that cell in the
% output grid would be orange. The number elements should be same as that of
% vutSpeeds array.
orangeEndSpeeds = [30;25;25;25;25;15;NaN;NaN;NaN];

% Impact speeds where brown color ends. This means that, for the vehicle
% under test, its speed at the end of collision if present (or
% simulation) is less than this speed, then the color of that cell in the
% output grid would be brown. The number elements should be same as that of
% vutSpeeds array.
brownEndSpeeds = [40;35;35;NaN;NaN;NaN;NaN;NaN;NaN];

% Impact speeds where red color ends. This means that, for the vehicle
% under test, its speed at the end of collision if present (or
% simulation) is less than this speed, then the color of that cell in the
% output grid would be red. The number elements should be same as that of
% vutSpeeds array.
redEndSpeeds = [50;45;40;35;30;25;20;15;10];

% Create the speed reduction look-up table.
speedReductionTable = table(vutSpeeds, greenEndSpeeds, yellowEndSpeeds, orangeEndSpeeds, brownEndSpeeds, redEndSpeeds);

% Convert egoNewSpeeds to km/hr.
speeds = round(egoNewSpeeds * 3.6); % km/hr

% Define test iteration result index to extract Simulink test Iteration
% results.
tirIndex=1;

% Get the size of speed reduction table
[~, cols] = size(speedReductionTable);

% Initialize the color array with zeros.
color = zeros(size(egoNewSpeeds,2), size(newCollisionPoints,2));

% Initialize the endSpeed array with zeros.
endSpeed = zeros(size(egoNewSpeeds,2), size(newCollisionPoints,2));

% Iterate over the number of ego new speeds
for i = 1 : length(egoNewSpeeds) % speed
    % Iterate over the number of collision point
    for j= 1 : length(newCollisionPoints) % collision point
        % Get the run array for the current simulation iteration.
        runArray = getOutputRuns(testIterationResult(tirIndex));
        % Get the collision status from the run array.
        collision_status = getSignalsByName(runArray,'collision');
        % export the collision status data
        tsCollision = export(collision_status);
        % check if there is any collision in the current selected run.
        collisionPointIndex = find(tsCollision.Data>0, 1, 'first');
        % Get the ego_velocity signal data from the current selected
        % simulation.
        egoSpeed = getSignalsByName(runArray,'ego_velocity');
        % export the ego speed data.
        ts_egoSpeed = export(egoSpeed);
        % Get the end speed of  the ego vehicle in the current
        % simulation
        endEgoSpeed = ts_egoSpeed.Data(end);
        % Convert the end speed to km/hr
        endSpeed(i,j) = endEgoSpeed*3.6;
        % Neglect the residual end speed (< 0.2778 m/s) of ego vehicle
        % only when there is no collision in the scenario.
        if(isempty(collisionPointIndex) && endSpeed(i,j) < 1)
            endSpeed(i,j) = 0;
        end
        % Initialize the color array with the the column number if the end
        % speed of the ego vehilce in the current run matches with the
        % column in the speed reduction look up table starting from second
        % column.
        for k = 2:cols
            if( endSpeed(i,j) <= table2array(speedReductionTable(i, k)) )
                switch k
                    case 2
                        color(i, j) = 1;
                    case 3
                        color(i, j) = 2;
                    case 4
                        color(i, j) = 3;
                    case 5
                        color(i, j) = 4;
                    case 6
                        color(i, j) = 5;
                end
                break;
            else
                k=k+1;
            end
        end
        %Increment the index to access next iteration result.
        tirIndex=tirIndex+1;
    end
end
% Open ui figure.
fig = uifigure('Position',[300 300 760 360], 'Name', 'AEB CPNC Scenario Variation Results');
fig.Resize='off';

% Initialize uitable with the fig handle.
uit = uitable(fig);
uit.Position = [70 100 400 190];
% Store the endSpeed array to get populated in the final result.
uit.Data = endSpeed;
uit.RowName = 'numbered';

% Based on the column number in the color array initialize the colr value.
for i = 1:size(color,1)
    for j=1:size(color,2)
        switch color(i,j)
            case 1 % Green
                colorValue = [0 0.6 0.3];
            case 2 % Yellow
                colorValue = [1 1 0.4];
            case 3 %Orange
                colorValue = [1 0.6 0.2];
            case 4 % Brown
                colorValue = [0.6  0.1 0.1];
            case 5 % Red
                colorValue = [1 0 0];
        end
        % Update the color in the grid result.
        s = uistyle('BackgroundColor',colorValue);
        addStyle(uit,s,'cell',[i, j]);
    end
end
uit.RowName = cellfun(@num2str, num2cell(round(speeds)),'UniformOutput',false);
uit.ColumnName  = cellfun(@num2str, num2cell(newCollisionPoints),'UniformOutput',false);

% Plot the look up table in the ui figure.
ax=uiaxes(fig, Position=[500 100 275 200]);
imagedata= imread('TestSpeedVsImpactSpeed.PNG');
image(imagedata,  "Parent", ax);
ax.Visible='off';

% Add title to the figure
str = {'AEB - Car to Pedestrian Nearside Child', '              Scenario Variants','           Simulation Grid Results'};
lbl = uilabel(fig);
lbl.Text = str;
lbl.Position = [270 230 300 200];

% Add x label
lbl1 = uilabel(fig);
lbl1.Text = 'Collision Point';
lbl1.Position = [200 200 300 200];

% Add y-label
lbl2 = uilabel(fig);
str = {'     VUT', 'Test Speed','     km/hr'};
lbl2.Text =str;
lbl2.Position = [8 100 300 200];
end