classdef HelperSpeedVariation < HelperActorVariation
    % HELPERSPEEDVARIATION Class to hold speed variation parameters for 
    % actors in the scenario.
    % 
    % This is a Class for the Scenario Variant Generator for Automated 
    % Driving Toolbox.
    % This is a helper for example purposes and may be modified in the 
    % future.

    % Copyright 2022 The MathWorks, Inc.
    
    properties
        ActorID
        Speed
    end
    
    methods
        function obj = HelperSpeedVariation(ActorID,Speed)
            %SPEEDVARIATION Construct an instance of this class
            %   Detailed explanation goes here
            obj.ActorID = ActorID;
            obj.Speed = Speed;
            totalActors = size(obj.ActorID,2);
            for i = 1:totalActors
                if iscell(obj.Speed{i})
                     actorsSpeed = cell2mat(obj.Speed{i});
                 else
                     actorsSpeed = obj.Speed{i};
                 end
                if any(actorsSpeed <= 0)
                    error("Only positive speed values are supported.")
                end
                if any(actorsSpeed > 42)
                    error("Speed values greater than 42 metres per second(151 kmph) are not supported.")
                end
            end
        end

         function scenarioDataOutput = applyVariation(obj, scenarioData)
             % Apply Variation and set input to the obj and particular scenario;

             %access seed scenario
             % copy these Waypoints to scenario's ActorID
             % return the scenario.
            
             totalActors = size(obj.ActorID,2);
             scenarioDataOutput = scenarioData;

             for i = 1:totalActors
                 currentActorID = obj.ActorID{i};
                 actorLog = scenarioData.entities.actors{currentActorID};
                 if iscell(obj.Speed{i})
                     actorsSpeed = cell2mat(obj.Speed{i});
                 else
                     actorsSpeed = obj.Speed{i};
                 end
                 % If empty keep original values
                 if ~isempty(actorsSpeed)
                     if length(actorsSpeed) == 1
                         actorLog.route.Speed(1:end) = actorsSpeed(1);
                     else
                         actorLog.route.Speed = actorsSpeed;
                     end
%                      actorLog.route.Speed(1:end) = actorsSpeed(1);
                     %                  actorLog.route.Speed(2:end) = actorsSpeed(1); %Bug
                     scenarioDataOutput.entities.actors{currentActorID} = actorLog;
                 end
             end

        end
    end
end


