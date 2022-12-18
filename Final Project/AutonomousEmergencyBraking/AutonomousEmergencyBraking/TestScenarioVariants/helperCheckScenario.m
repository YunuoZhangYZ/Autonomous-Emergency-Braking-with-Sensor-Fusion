function checkStatus = helperCheckScenario(egoID, targetID, VariantScenarioDescriptor)
% helperCheckScenario This function performs validation checks on the input 
% arguments for the Scenario Variant Example.
% 
% This is a helper for example purposes and may be modified in the future.

% Copyright 2022 The MathWorks, Inc.

% Validation checks on input arguments
arguments
    egoID (1,1) double {mustBeNumeric, mustBePositive}
    targetID (1,1) double {mustBeNumeric, mustBePositive}
    VariantScenarioDescriptor variantgenerator.internal.ScenarioDescriptor
end

checkStatus = 1; %setting the default status to be passed

if egoID == targetID
    checkStatus = 0;
    return;
end
scenarioData = VariantScenarioDescriptor.getScenarioData();

% validateattributes(scenario, {'drivingScenario'}, {'nonempty'});
numActors = size(scenarioData.entities.actors, 1);
if numActors < 2
    checkStatus = 0;
    return;
end
% add validation to check struct egoData and targetData has all the mandatory fields
% validateattributes(egoData, {'struct'}, {'nonempty'});
% validateattributes(targetData, {'struct'}, {'nonempty'});

IDArray = [egoID, targetID];

for i = 1:2
    actorData = scenarioData.entities.actors{IDArray(i)};
    if ~isfield(actorData.route,'Waypoints') || isempty(actorData.route.Waypoints)
        checkStatus = 0; %test failed
        return;
    elseif ~isfield(actorData.route,'Speed') || isempty(actorData.route.Speed)
        checkStatus = 0;
        return;
    end

    if any(actorData.route.Speed < 0)
        checkStatus = 0;
        return;
    end
    if all(size(actorData.route.Speed) == [1, 1])
        checkStatus = 0;
        return;
    end

    % check size of speed and Waypoints are consistent
    if size(actorData.route.Speed,1) ~= size(actorData.route.Waypoints,1)
        checkStatus = 0;
        return;
    end
    if isfield(actorData.route,'Yaw')
        % check size of waypoints and Yaw are consistent
        if size(actorData.route.Waypoints,1) ~= size(actorData.route.Yaw,1)
            checkStatus = 0;
            return;
        end
    end

    actorWaypoints = actorData.route.Waypoints;
    actorSpeed = actorData.route.Speed;

    if size(actorWaypoints,1) < 2
        checkStatus = 0;
        return;
    end
    if all(actorSpeed == 0)
        checkStatus = 0;
        return;
    end

    if size(actorWaypoints,1) < 3
        % if Waypoints are less than 3 for actor
        checkStatus = 0;
        return;
    end
    if actorData.actorProfile.ActorID == targetID
        if (~all(actorSpeed(2) == actorSpeed(3:end))) || any(actorSpeed(2:end) == 0)
            checkStatus = 0;
            return;
        end
    else
        if (~all(actorSpeed(1) == actorSpeed(2:end))) || any(actorSpeed(1:end) == 0)
            checkStatus = 0;
            return;
        end
    end

    if actorData.actorProfile.ActorID == egoID && any(actorSpeed(1:end) > 42)
        checkStatus = 0;
        return;
    end
end

actorsOfInterest.CauseActorID = egoID;
actorsOfInterest.AffectActorID = targetID;
CollisionObj = HelperCollision(scenarioData, actorsOfInterest, VariantScenarioDescriptor);

%check if the scenario has a collision.
if CollisionObj.Time == -1
    checkStatus = 0;
    return;
end

end


