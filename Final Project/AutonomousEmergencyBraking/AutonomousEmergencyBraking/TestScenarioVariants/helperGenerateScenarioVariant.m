function scenarioVariant = helperGenerateScenarioVariant(scenarioDescriptor, egoID, targetID, egoNewSpeed, newCollisionPoint)
% helperGenerateScenarioVariant Get the seed scenario object and ego
% vehicle ID and target vehicle ID along with the required new ego speed
% and collision point to generate the new variant.
%
% This is a helper function for example purposes and may be removed or
% modified in the future.

% Copyright 2022 The MathWorks, Inc.

% Create a scenario variant object using the seed scenario descriptor.
svObj = HelperScenarioVariant(scenarioDescriptor);

% Perform actor variation in speed with the ego new speed data.
avObj = HelperActorVariation();
avObj.addSpeedVariation({egoID}, {egoNewSpeed});
svObj.ActorVariations = avObj;

% Perform the event variation in collision point with the new collision
% point data
evObj = HelperEventVariation();
evObj.addCollision(svObj, egoID, targetID, CollisionPoint=newCollisionPoint);
svObj.EventVariations = evObj;

% Generate the variants
svObj.generateVariants();

% Construct a scenario descriptor object that has the new scenario object
% with new speed and collision data
dsObj = scenarioDescriptor.updateScenarioData(svObj.VariantGenerator.scenarioData);

% Get the scenario object from the scenario descriptor object
scenarioVariant = getScenario(dsObj,Simulator="DrivingScenario");
end