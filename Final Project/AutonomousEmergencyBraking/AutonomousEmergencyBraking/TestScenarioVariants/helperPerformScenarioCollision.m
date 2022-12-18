function VariantScenarioDescriptor = helperPerformScenarioCollision(egoID, targetID, VariantScenarioDescriptor, options)
% helperPerformScenarioCollision This function creates a collision in the 
% seed scenario, if possible.
%
% This is a helper for example purposes and may be modified in the 
% future.

% Copyright 2022 The MathWorks, Inc.

% Validation checks on input arguments
arguments
    egoID (1,1) double {mustBeNumeric, mustBePositive}
    targetID (1,1) double {mustBeNumeric, mustBePositive}
    VariantScenarioDescriptor variantgenerator.internal.ScenarioDescriptor
    options.method (1, 1) string {mustBeMember( options.method,["WaitTime","EntryTime"])} = "WaitTime"
end

if egoID == targetID
    error("Ego and Target IDs must be different.")
end
scenarioData = VariantScenarioDescriptor.getScenarioData();

% validateattributes(scenario, {'drivingScenario'}, {'nonempty'});
numActors = size(scenarioData.entities.actors, 1);
if numActors < 2
    error(message('Less than two actors in the input scenario'));
end
% add validation to check struct egoData and targetData has all the mandatory fields
% validateattributes(egoData, {'struct'}, {'nonempty'});
% validateattributes(targetData, {'struct'}, {'nonempty'});

IDArray = [egoID, targetID];

for i = 1:2
    actorData = scenarioData.entities.actors{IDArray(i)};
    if ~isfield(actorData.route,'Waypoints') || isempty(actorData.route.Waypoints)
        error(actorData.actorProfile.Name + ": Waypoints not set in scenario.")
    elseif ~isfield(actorData.route,'Speed') || isempty(actorData.route.Speed)
        error(actorData.actorProfile.Name + ": Speed not set in scenario.")
    end

    isActorDataModified = false;
    if any(actorData.route.Speed < 0)
        error(actorData.actorProfile.Name + ": Negative speed for actor is not supported.")
    end
    if all(size(actorData.route.Speed) == [1, 1])
        actorData.route.Speed = ones(size(actorData.route.Waypoints, 1), 1)*actorData.route.Speed;
        isActorDataModified = true;
        if actorData.actorProfile.ActorID == targetID
            actorData.route.Speed(1) = 0; %target needs to have a rampUp
        end
    end

    % check size of speed and Waypoints are consistent
    if size(actorData.route.Speed,1) ~= size(actorData.route.Waypoints,1)
        Speed = actorData.route.Speed';
        if(size(Speed,1) == size(actorData.route.Waypoints,1))
            actorData.route.Speed = Speed;
        else
            errorMsg = actorData.actorProfile.Name + ": Size of speed and waypoints is not equal";
            error(errorMsg);
        end
    end
    if isfield(actorData.route,'Yaw')
        % check size of waypoints and Yaw are consistent
        if size(actorData.route.Waypoints,1) ~= size(actorData.route.Yaw,1)
            Yaw = actorData.route.Yaw';
            if(size(Yaw,1) == size(actorData.route.Waypoints,1))
                actorData.route.Yaw = Yaw;
            else
                errorMsg = actorData.actorProfile.Name + ": Size of yaw and waypoints is not equal";
                error(errorMsg);
            end
        end
    end

    actorWaypoints = actorData.route.Waypoints;
    actorSpeed = actorData.route.Speed;

    if size(actorWaypoints,1) < 2
        error('Less number of waypoints for actor');
    end
    if all(actorSpeed == 0)
        error('Actor has speed zero for all waypoints');
    end

    if size(actorWaypoints,1) < 3
        % if Waypoints are less than 3 for actor
        mid = (actorWaypoints(1,:) + actorWaypoints(2,:))./2;
        actorWaypoints = [actorWaypoints(1,:);mid;actorWaypoints(2,:)];
        value = actorSpeed(end);
        actorSpeed = ones(3,1)*value;
        if actorData.actorProfile.ActorID == targetID %target has ramp up
            actorSpeed(1) = 0;
        end
        if isfield(actorData.route,'Yaw')
            actorData.route = rmfield(actorData.route,'Yaw');
        end
        isActorDataModified = true;
        warning(actorData.actorProfile.Name + ": Added a waypoint in the actor trajectory");
    end
    if actorData.actorProfile.ActorID == targetID
        if (~all(actorSpeed(2) == actorSpeed(3:end))) || any(actorSpeed(2:end) == 0)
            Speed = 10;
            % Convert Speed value to m/s
            Speed = (Speed*1000)/3600;
            actorSpeed(2:end) = Speed;
            isActorDataModified = true;
            warning(actorData.actorProfile.Name + ": Invalid actor speed. Changing speed to default paramaters.");
        end
    else
        if (~all(actorSpeed(1) == actorSpeed(2:end))) || any(actorSpeed(1:end) == 0)
            Speed = 60;
            % Convert Speed value to m/s
            Speed = (Speed*1000)/3600;
            actorSpeed(1:end) = Speed;
            isActorDataModified = true;
            warning(actorData.actorProfile.Name + ": Invalid actor speed. Changing speed to default paramaters.");
        end
    end

    if actorData.actorProfile.ActorID == egoID && any(actorSpeed(1:end) > 42)
        Speed = 60;
        Speed = Speed*1000/3600;
        actorSpeed(1:end) = Speed;
        isActorDataModified = true;
        warning(actorData.actorProfile.Name + ": Speed values greater than 42 metres per second(151 kmph) are not supported. Changing speed to default paramaters.");
    end

    if isActorDataModified
        actorData.route.Speed = actorSpeed;
        actorData.route.Waypoints = actorWaypoints;
        actorData.route = rmfield(actorData.route,'WaitTime');
        scenarioData.entities.actors{actorData.actorProfile.ActorID} = actorData;

        scenarioData = HelperCollision.reRunScenario(scenarioData, VariantScenarioDescriptor);
    end
end

VariantScenarioDescriptor = VariantScenarioDescriptor.updateScenarioData(scenarioData);

actorsOfInterest.CauseActorID = egoID;
actorsOfInterest.AffectActorID = targetID;
CollisionObj = HelperCollision(scenarioData, actorsOfInterest, VariantScenarioDescriptor);

%check if the scenario has a collision.
if CollisionObj.Time == -1
    if isequal(options.method, "WaitTime")
        warning('EgoActor and targetActor are not colliding in seed scenario. Modifying WaitTime to create collision.');
    elseif isequal(options.method, "EntryTime")
        warning('EgoActor and targetActor are not colliding in seed scenario. Modifying EntryTime to create collision.');
    end
    scenarioData =  HelperCollision.createCollision(scenarioData, actorsOfInterest, VariantScenarioDescriptor,  options.method);
    VariantScenarioDescriptor = VariantScenarioDescriptor.updateScenarioData(scenarioData);

    %visualize the updated scenario
    scenario = getScenario(VariantScenarioDescriptor,Simulator="DrivingScenario");

    restart(scenario);

    figScene = figure;
    set(figScene,Position=[50 50 500 500]);
    hPanel1 = uipanel(figScene,Position=[0 0 1 1]);
    hPlot1 = axes(hPanel1);
    plot(scenario,Waypoints="on",Parent=hPlot1)
    title("Modified Seed Scenario to create Collision")

    figSceneChasePlot = figure;
    set(figSceneChasePlot,Position=[50 50 500 500]);
    hPanel2 = uipanel(figSceneChasePlot,Position=[0 0 1 1]);
    hPlot2 = axes(hPanel2);
    chasePlot(scenario.Actors(egoID), Parent = hPlot2)

    while advance(scenario)
        pause(0.01)
    end
end


end
