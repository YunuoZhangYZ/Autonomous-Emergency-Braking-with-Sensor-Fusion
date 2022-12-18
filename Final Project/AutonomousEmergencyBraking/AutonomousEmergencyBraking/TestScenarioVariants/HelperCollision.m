classdef HelperCollision < HelperEventVariation
    % HELPERCOLLISION Class to store collision related Details.
    % 
    % This is a Class for the Scenario Variant Generator for Automated 
    % Driving Toolbox.
    % This is a helper for example purposes and may be modified in the 
    % future.

    % Copyright 2022 The MathWorks, Inc.
    properties

        Time % time instant of collision in seconds
        PointOfCollision %Waypoint of Cause and Affect Actor at time of collision

        CauseActor %actor that causes the event
        AffectActor %actor that is affected by the event
        CauseActorCollisionSurface %Collision surface name for the cause actor 
        CauseActorCollisionFraction %Fraction of the cause actor collision surface, at which the collision should take place
        AffectActorCollisionSurface %Collision surface name for the affect actor
        AffectActorCollisionFraction %Fraction of the cause actor collision surface, at which the collision should take place
        OccurenceTime %Time after the start simulation time, when collision should occur
        OccurenceDistance %Distance from the origin, where collision should occur
        Method %Method (WaitTime or EntryTime) that should be modified to create variations
    
        extractedScenarioCollisionInformation %structure that contains the collision details of the passed ScenarioData for CauseActor and AffectActor
        %{      
            Fields inside extractedScenarioCollisionInformation for cacd
            (Cause Actor Collision Details) and aacd (Affect Actor
            Colllision Details)
    
            collisionFraction: impact percentage of collision
    
            Waypoints: position of Actor at the time of collision
            
            CollisionSurfaceLength: length of the side of CauseActor, that
            collided with AffectActor (only for cacd)
     
            LeftEdge
            RightEdge
            BackEdge
            FrontEdge
           
            collidingSurface
            collidingSurfaceName
        %}
    end

    properties(Hidden)
        CauseActorCornerPoints
        AffectActorCornerPoints
        CauseActorWaypoints
        AffectActorWaypoints
        VariantScenarioDescriptor
    end

    methods
        function obj = HelperCollision(scenarioData, actorsOfInterest, VariantScenarioDescriptor, options)

            arguments
                scenarioData struct {mustBeNonempty}
                actorsOfInterest  struct  {mustBeNonempty}
                VariantScenarioDescriptor variantgenerator.internal.ScenarioDescriptor
                options.OccuranceTime (1,1) double {mustBeNumeric, mustBeNonnegative} = 0
                options.OccuranceDistance (1,1) double {mustBeNumeric, mustBeNonnegative} = 0
                options.CauseActorCollisionFraction (1,1) double {mustBeNumeric, mustBeNonnegative}
                options.AffectActorCollisionFraction (1,1) double {mustBeNumeric, mustBeNonnegative}
                options.CauseActorCollisionSurface (1, 1) string
                options.AffectActorCollisionSurface (1, 1) string
                options.Method (1, 1) string = "WaitTime"
             end


            narginchk(2,5);
            nargoutchk(1,1);   

            if ~any(lower(options.Method) == ["waittime", "entrytime"])
                error("Invalid Method. Only WaitTime and EntryTime methods are supported.")
            end
            obj.Method = options.Method;
            obj.VariantScenarioDescriptor = VariantScenarioDescriptor;

            obj = obj.SetandComputeCollisionData(scenarioData, actorsOfInterest);

            %checking if user provides CauseActorCollisionFraction as input, set it to be the subsequent collisions'
            %CauseActorCollisionFraction. 
            %Otherwise use CauseActorCollisionFraction from the extracted seed scenario collision details.
            if ~isfield(options, 'CauseActorCollisionFraction')
                if isfield(obj.extractedScenarioCollisionInformation, "CauseActorCollisionDetails")
                    obj.CauseActorCollisionFraction = obj.extractedScenarioCollisionInformation.CauseActorCollisionDetails.collisionFraction/100;
                else
                    obj.CauseActorCollisionFraction = -1;
                end
            else
                obj.CauseActorCollisionFraction = options.CauseActorCollisionFraction;
            end

        end

    end


    methods

        function scenarioData = applyVariation(obj, scenarioData)
            %Variations are applied on the scenarioData using the
            %parameters stored in the Collision object.

            if obj.Time == -1 %if there is no collision in the scenario, return.
                return;
            end
           
            objTemp = struct();

            for fn = fieldnames(obj)'
                    objTemp.(fn{1}) = obj.(fn{1});
            end

            scenarioData = obj.applySpeedVariation(scenarioData);
            scenarioData = obj.applyDimensionVariation(scenarioData);

            scenarioData = HelperCollision.reRunScenario(scenarioData, obj.VariantScenarioDescriptor);
            actorsOfInterest.CauseActorID = obj.CauseActor.actorProfile.ActorID;
            actorsOfInterest.AffectActorID = obj.AffectActor.actorProfile.ActorID;
            obj = obj.SetandComputeCollisionData(scenarioData, actorsOfInterest); %recomputing collision data for more accurate collisionPoint consistency

            scenarioData = obj.applyCollisionPointVariation(scenarioData);

            for fn = fieldnames(obj)'
                    obj.(fn{1}) = objTemp.(fn{1}); %resetting the collision objects state back for next set of variations
            end

        end

        function scenarioData = applyCollisionPointVariation(obj, scenarioData)
            % This function generates new scenario by changing collision
            % point. It uses CauseActorCollisionFraction to generate the
            % new collision point.

            cacd = obj.extractedScenarioCollisionInformation.CauseActorCollisionDetails; %CauseActorCollisionDetails
            aacd = obj.extractedScenarioCollisionInformation.CauseActorCollisionDetails; %AffectActorCollisionDetails

            causeActorData = scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID};
            affectActorData = scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID};

            newCollisionPoint = obj.CauseActorCollisionFraction*100;

            if abs(newCollisionPoint - cacd.collisionFraction) < 10e-5 || ~isequal(cacd.collidingSurfaceName, "Front")
                return
            end

            % Calculate the WaitTime to be assigned
            WaitTime = (abs(newCollisionPoint -  abs(cacd.collisionFraction))/100 * ...
                cacd.CollisionSurfaceLength)/abs(affectActorData.route.Speed(2));
            egowait = false;
            targetwait = false;

            % Compute who has to wait, ego or target
            if newCollisionPoint > cacd.collisionFraction
                targetwait = true;
            elseif newCollisionPoint < cacd.collisionFraction
                egowait = true;
            end

            if strcmp(obj.Method,'WaitTime')
                if targetwait
                    if isfield(affectActorData.route,'WaitTime') && ~isempty(affectActorData.route.WaitTime)
                        affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) + WaitTime;
                    else
                        affectActorData.route.WaitTime = zeros(size(affectActorData.route.Waypoints, 1), 1);
                        affectActorData.route.WaitTime(1) = WaitTime;
                    end
                elseif egowait
                    if isfield(affectActorData.route,'WaitTime') && ~isempty(affectActorData.route.WaitTime) && affectActorData.route.WaitTime(1) > WaitTime
                        affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) - WaitTime;
                    else
                        warning("Cannot add waitTimes to Ego. Collision Point Variant generated may not be consistent.");
                    end
    %                 if isfield(causeActorData.route,'WaitTime') && ~isempty(causeActorData.route.WaitTime)
    %                     causeActorData.route.WaitTime(1) = causeActorData.route.WaitTime(1) + WaitTime;
    %                 else
    %                     causeActorData.route.WaitTime = zeros(size(causeActorData.route.Waypoints, 1), 1);
    %                     causeActorData.route.WaitTime(1) = WaitTime;
    %                 end
                end
            elseif strcmp(obj.Method,'EntryTime')
                if targetwait
                    affectActorData.route.EntryTime = affectActorData.route.EntryTime + WaitTime;
                elseif egowait
                    causeActorData.route.EntryTime = causeActorData.route.EntryTime + WaitTime;
                end
            end

            scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID} = causeActorData;
            scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID} = affectActorData;

        end



        function scenarioData = applySpeedVariation(obj, scenarioData, ~, ~)

            %below step computes the modifications required to be made in
            % the variant for changing to new ego Speed

            causeActorData = scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID};
            affectActorData = scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID};

            [causeActorData, affectActorData] = obj.helperComputeCollisionTimeDiff(causeActorData, affectActorData);
            scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID} = causeActorData;
            scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID} = affectActorData;

        end

        function [causeActorData,affectActorData] = helperComputeCollisionTimeDiff(obj, causeActorData, affectActorData)
            %   This script computes modifications depending upon the Method
            %   WaitTime, EntryTime. It is used for applying speed
            %   modifications.

            %   This is a helper script for example purposes and be modified in the
            %   future
            %   Copyright 2021-2022 The MathWorks, Inc.

            
            if abs(norm(causeActorData.simulatedTrajectory.Velocity(1, :)) - causeActorData.route.Speed(1)) < 10e-4
                return;
            end


            % Store the simulation time instants and actor poses from scenario
            simulationTime = causeActorData.simulatedTrajectory.Timestamps;

            %store ego pose data
            egoPoseData = causeActorData.simulatedTrajectory;
            % Store ego positions
            egoPositions = egoPoseData.Waypoints;
            % Check for stationary ego
            if size(unique(egoPositions,'rows'),1) == 1
                error('Ego is stationary');
            end
            errorLimit = 0.005;
            % Variable 'egoArrivalTimes' stores the simulation time instant at ego's
            %Waypoints
            egoArrivalTimes = zeros(size(causeActorData.route.Waypoints,1),1);
            % Variable 'egoCumulativeDistance' stores the total distance travelled by
            % ego for reaching Waypoints
            egoCumulativeDistance = zeros(size(causeActorData.route.Waypoints,1),1);
            % Below loop finds the time instants when ego reaches its Waypoints and the
            % total distance travelled by the ego to reach that waypoint
            [~,map] = ismembertol(causeActorData.route.Waypoints,egoPositions,errorLimit,'ByRows',true);
            % 'map' is mapping between positions and Waypoints
            if ~any(map ~= 0)
                % if no waypoint matched with position
                error('Decrease error threshold');
            end
            if size(unique(map(map ~= 0)),1) < size(map(map ~= 0),1)
                % if multiple waypoint matched to same position because of low
                % threshold
                error('Increase error threshold');
            end
            index = 2;
            distance = 0;
            for i = 2:size(egoPositions,1)
                distance = distance + norm(egoPositions(i,:)-egoPositions(i-1,:));
                if index > size(causeActorData.route.Waypoints,1)
                    break;
                end
                if map(index) == i
                    egoCumulativeDistance(index) = distance; %store the cumulative distance
                    egoArrivalTimes(index) = simulationTime(i);
                    index = index+1;
                end
            end
            %if collision occured before ego second waypoint
            if index == 2
                error('Design Scenario as per the mentioned requirements: Actors not colliding with constant Speed or decrease error threshold');
            end

%             Condition for Ramp up
%             if egoArrivalTimes(2) >= obj.Time 
%                 error('Design Scenario as per the mentioned requirements: Actors not colliding with constant Speed');
%             end

            % Store target pose data
            targetPoseData = affectActorData.simulatedTrajectory;
            % Store target positions
            targetPositions = targetPoseData.Waypoints;
            % Variable 'targetArrivalTimes' stores the simulation time instant at target's Waypoints
            targetArrivalTimes = zeros(size(affectActorData.route.Waypoints,1),1);
            % Variable 'targetCumulativeDistance' stores the total distance travelled by
            % target for reaching Waypoints
            targetCumulativeDistance = zeros(size(affectActorData.route.Waypoints,1),1);
            % Below loop finds the time instant when the target reaches its Waypoints
            %and the distance travlled by the target to reach it's Waypoints
            index = 2;
            distance = 0;
            [~,map] = ismembertol(affectActorData.route.Waypoints,targetPositions,errorLimit,'ByRows',true);
            % 'map' is mapping between positions and Waypoints
            if ~any(map ~= 0)
                % if no waypoint matched with position
                error('Increase error threshold');
            end
            if size(unique(map(map ~= 0)),1) < size(map(map ~= 0),1)
                % if multiple waypoint matched to same position because of low
                % threshold
                error('Increase error threshold');
            end
            for i = 2:size(targetPositions,1)
                if index > size(targetCumulativeDistance,1)
                    break;
                end
                distance = distance + norm(targetPositions(i,:)-targetPositions(i-1,:));
                if index > size(affectActorData.route.Waypoints,1)
                    break;
                end
                if map(index) == i
                    targetCumulativeDistance(index) = distance; %store the cumulative distance
                    targetArrivalTimes(index) = simulationTime(i);
                    index = index+1;
                end
            end
            %if collision occured before target second waypoint
            if index == 2
                error('Design Scenario as per the mentioned requirements: Actors not colliding with constant Speed or decrease error threshold');
            end
%             Condition for Ramp up
            if targetArrivalTimes(2) >= obj.Time
                error('Design Scenario as per the mentioned requirements: Actors not colliding with constant Speed');
            end
            if size(unique(map(1:index-1)),1) < size(map(1:index-1),1)
                error('Increase error threshold');
            end
            % Struct variable 'modifications' store the modifications that need to be made in the scenario object
            modifications = struct('egoCollisionTimeDiff',0,'targetCollisionTimeDiff',0,'Waypoints',[]);
            egoNewSpeed = abs(causeActorData.route.Speed(1));
            % Below steps calls different functions depending upon the type of Method
            try
                if strcmp(obj.Method,'WaitTime')
                    [modifications.egoCollisionTimeDiff,modifications.targetCollisionTimeDiff]...
                        = HelperCollision.helperComputeArrivalTimeDiffWaittime(egoNewSpeed, obj.Time,causeActorData);
                    if modifications.egoCollisionTimeDiff ~= 0
                        if isfield(affectActorData.route,'WaitTime') && ~isempty(affectActorData.route.WaitTime) && affectActorData.route.WaitTime(1) > modifications.egoCollisionTimeDiff
                            affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) - modifications.egoCollisionTimeDiff;
                        else
                            warning("Cannot add waitTimes to Ego. Ego Speed Change Variant generated may not be consistent.");
                        end
%                         causeActorData.route.Speed(1) = 0;
%                         if isfield(causeActorData.route,'WaitTime') && ~isempty(causeActorData.route.WaitTime)
%                             causeActorData.route.WaitTime(1) = causeActorData.route.WaitTime(1) + modifications.egoCollisionTimeDiff;
%                         else
%                             causeActorData.route.WaitTime = zeros(size(causeActorData.route.Waypoints, 1), 1);
%                             causeActorData.route.WaitTime(1) = modifications.egoCollisionTimeDiff;
%                         end
                    end
                    if modifications.targetCollisionTimeDiff ~=0
                        affectActorData.route.Speed(1) = 0;
                        if isfield(affectActorData.route,'WaitTime') && ~isempty(affectActorData.route.WaitTime)
                            affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) + modifications.targetCollisionTimeDiff;
                        else
                            affectActorData.route.WaitTime = zeros(size(affectActorData.route.Waypoints, 1), 1);
                            affectActorData.route.WaitTime(1) = modifications.targetCollisionTimeDiff;
                        end
                    end

                elseif strcmp(obj.Method,'EntryTime')
                    [modifications.egoCollisionTimeDiff,modifications.targetCollisionTimeDiff]...
                        = HelperCollision.helperComputeArrivalTimeDiffWaittime(egoNewSpeed, obj.Time,causeActorData);
                    causeActorData.route.EntryTime = causeActorData.route.EntryTime + modifications.egoCollisionTimeDiff;
                    affectActorData.route.EntryTime = affectActorData.route.EntryTime + modifications.targetCollisionTimeDiff;

                end
            catch ME
                error(ME.message);
            end

        end


        function scenarioData = applyDimensionVariation(obj, scenarioData)
            %function to keep collision consistency after dimension
            %variations.

            causeActorData = scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID};
            affectActorData = scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID};
            
            cacd = obj.extractedScenarioCollisionInformation.CauseActorCollisionDetails;
            aacd = obj.extractedScenarioCollisionInformation.AffectActorCollisionDetails;

            egoPositions = causeActorData.simulatedTrajectory.Waypoints;
            count = length(egoPositions);
            % Compute front corners of the cuboid around ego using new ego
            % dimensions
            [egoFrontLeftCorner,egoFrontRightCorner] = HelperCollision.computeactorBoundingBox(causeActorData, causeActorData.simulatedTrajectory.Waypoints, ...
                causeActorData.simulatedTrajectory.Yaw,causeActorData.simulatedTrajectory.Pitch,causeActorData.simulatedTrajectory.Roll);
            %Compute the time when ego front with new dimensions
            %reaches the collision point
            minDistance = Inf;
            minIndex = -1;
            minTheta = 300;
            for i =1:count
                u = (egoFrontLeftCorner(i,1:2)-egoFrontRightCorner(i,1:2))./...
                    norm(egoFrontLeftCorner(i,1:2)-egoFrontRightCorner(i,1:2));
                v = (cacd.FrontEdge(1,1:2) - cacd.FrontEdge(2,1:2))./...
                    norm(cacd.FrontEdge(1,1:2) - cacd.FrontEdge(2,1:2));
                theta = acosd(dot(u,v));
                minTheta = min(min(theta, minTheta), abs(180 - theta));
                if min(abs(theta), abs(theta - 180)) < 10e-3
                    distance = HelperCollision.distanceBetweenParallelLines([egoFrontLeftCorner(i,1:2);egoFrontRightCorner(i,1:2)], ...
                        cacd.FrontEdge(:,1:2));
                    if distance < minDistance
                        minDistance = distance;
                        minIndex = i;
                    end
                end
            end
            if minIndex == -1
                error('Error in the creation of variants using new actor dimensions');
            end
            egoNewTime = obj.CauseActor.simulatedTrajectory.Timestamps(minIndex);
            
            % Compute front corners of the cuboid target using new target
            % dimensions
            [targetFrontLeftCorner,targetFrontRightCorner] = HelperCollision.computeactorBoundingBox(affectActorData, affectActorData.simulatedTrajectory.Waypoints, ...
                affectActorData.simulatedTrajectory.Yaw,affectActorData.simulatedTrajectory.Pitch,affectActorData.simulatedTrajectory.Roll);
            %Compute the time when target front with new dimensions
            %reaches the collision point
            minDistance = Inf;
            minIndex = -1;
            for i =1:count
                u = (targetFrontLeftCorner(i,1:2)- targetFrontRightCorner(i,1:2))./...
                    norm(targetFrontLeftCorner(i,1:2)- targetFrontRightCorner(i,1:2));
                v = (aacd.FrontEdge(1,1:2) - aacd.FrontEdge(2,1:2))/...
                    norm(aacd.FrontEdge(1,1:2) - aacd.FrontEdge(2,1:2));
                theta = acosd(dot(u,v));
                if min(abs(theta), abs(theta - 180)) < 10e-3
                    distance = HelperCollision.distanceBetweenParallelLines([targetFrontLeftCorner(i,1:2);targetFrontRightCorner(i,1:2)], ...
                        aacd.FrontEdge(:,1:2));
                    if distance < minDistance
                        minDistance =  distance;
                        minIndex = i;
                    end
                end
            end
            if minIndex == -1
                error('Error in the creation of variants using new actor dimensions');
            end
            
            targetNewTime = obj.AffectActor.simulatedTrajectory.Timestamps(minIndex);
            if abs(targetNewTime - egoNewTime) < 10e-5
                return
            end

            if strcmp(obj.Method,'WaitTime')

                if egoNewTime < targetNewTime
                    if isfield(affectActorData.route,'WaitTime') && ~isempty(affectActorData.route.WaitTime) && affectActorData.route.WaitTime(1) > targetNewTime - egoNewTime
                        affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) + (egoNewTime - targetNewTime);
                    else
                        warning("Cannot add waitTimes to Ego. Dimension Variant generated may not be consistent.");
                    end
                    
    %                 if ~isfield(causeActorData.route,'WaitTime') || isempty(causeActorData.route.WaitTime)
    %                     causeActorData.route.WaitTime = zeros(size(causeActorData.route.Waypoints, 1), 1);                
    %                 end
    % 
    %                 %assign WaitTime to ego
    %                 causeActorData.route.WaitTime(1) = causeActorData.route.WaitTime(1) + (targetNewTime - egoNewTime);
    %                 scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID} = causeActorData;
    
                elseif egoNewTime > targetNewTime
                    if ~isfield(affectActorData.route,'WaitTime') || isempty(affectActorData.route.WaitTime)
                        affectActorData.route.WaitTime = zeros(size(affectActorData.route.Waypoints, 1), 1);    
                         affectActorData.route.Speed(1) = 0;
                    end
    
                    %assign WaitTime to ego
                    affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) + (egoNewTime - targetNewTime);
                    scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID} = affectActorData;
                end
            elseif strcmp(obj.Method,'EntryTime')
                if egoNewTime < targetNewTime
                    causeActorData.route.EntryTime = affectActorData.route.EntryTime + targetNewTime - egoNewTime;
                    scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID} = causeActorData;
                else
                    affectActorData.route.EntryTime = affectActorData.route.EntryTime - targetNewTime + egoNewTime;
                    scenarioData.entities.actors{obj.AffectActor.actorProfile.ActorID} = affectActorData;
                end
            end
        end
            

        function obj = SetandComputeCollisionData(obj, scenarioData, actorsOfInterest)
            %This function is used to extract collision related details
            %from the scenario. Futhermore the extracted collision details
            %are filled in the appropriate fields.
            
            obj.CauseActor = scenarioData.entities.actors{actorsOfInterest.CauseActorID};
            CauseActorDetails = scenarioData.entities.actors{actorsOfInterest.CauseActorID};
            cs = CauseActorDetails.simulatedTrajectory;
            
            % Store CauseActor positions
            obj.CauseActorWaypoints = cs.Waypoints;

            [CauseActorFLCorner,CauseActorFRCorner,CauseActorBLCorner,CauseActorBRCorner] = HelperCollision.computeactorBoundingBox(CauseActorDetails, cs.Waypoints, ...
                cs.Yaw,cs.Pitch,cs.Roll);
            obj.CauseActorCornerPoints = cell(4, 1);
            obj.CauseActorCornerPoints{1} = CauseActorFLCorner; obj.CauseActorCornerPoints{2} = CauseActorFRCorner;
            obj.CauseActorCornerPoints{3} = CauseActorBLCorner; obj.CauseActorCornerPoints{4} = CauseActorBRCorner;

            obj.Time = -1;

            obj.AffectActor = scenarioData.entities.actors{actorsOfInterest.AffectActorID};
            AffectActorActorDetails = scenarioData.entities.actors{actorsOfInterest.AffectActorID};

            % Store AffectActor positions
            es = AffectActorActorDetails.simulatedTrajectory;
            obj.AffectActorWaypoints = es.Waypoints;
            [AffectActorFLCorner,AffectActorFRCorner,AffectActorBLCorner,AffectActorBRCorner] = HelperCollision.computeactorBoundingBox(AffectActorActorDetails, ...
                es.Waypoints, es.Yaw, es.Pitch, es.Roll);
            obj.AffectActorCornerPoints = cell(4, 1);
            obj.AffectActorCornerPoints{1} = AffectActorFLCorner; obj.AffectActorCornerPoints{2} = AffectActorFRCorner;
            obj.AffectActorCornerPoints{3} = AffectActorBLCorner; obj.AffectActorCornerPoints{4} = AffectActorBRCorner;

            obj = obj.computeCollisionData();
        end

          

        function obj = computeCollisionData(obj)
            % This function is used to identify the collision in the
            % scenario and save its details.

            CauseActorFLCorner = obj.CauseActorCornerPoints{1}; CauseActorFRCorner = obj.CauseActorCornerPoints{2};
            CauseActorBLCorner = obj.CauseActorCornerPoints{3}; CauseActorBRCorner = obj.CauseActorCornerPoints{4};

            AffectActorFLCorner = obj.AffectActorCornerPoints{1}; AffectActorFRCorner = obj.AffectActorCornerPoints{2};
            AffectActorBLCorner = obj.AffectActorCornerPoints{3}; AffectActorBRCorner = obj.AffectActorCornerPoints{4};

            % switching off warning for avoiding warnings related to 'polyshape' in below loop
            warning('off','MATLAB:polyshape:repairedBySimplify');
            count = length(obj.CauseActorWaypoints);
            for i = 1:count
                %Compute the cuboid/polygon around CauseActor and AffectActor, to check if they are
                %intersecting
                isColliding = 0;
                collidingVertices = [];

                AffectActorPolygon = polyshape([AffectActorFLCorner(i,1),AffectActorFRCorner(i,1),AffectActorBRCorner(i,1),AffectActorBLCorner(i,1)], ...
                    [AffectActorFLCorner(i,2),AffectActorFRCorner(i,2),AffectActorBRCorner(i,2),AffectActorBLCorner(i,2)]);
                CauseActorPolygon = polyshape([CauseActorFLCorner(i,1),CauseActorFRCorner(i,1),CauseActorBRCorner(i,1),CauseActorBLCorner(i,1)], ...
                    [CauseActorFLCorner(i,2),CauseActorFRCorner(i,2),CauseActorBRCorner(i,2),CauseActorBLCorner(i,2)]);
                commonArea = intersect(AffectActorPolygon,CauseActorPolygon);
                if ~isempty(commonArea.Vertices)
                    isColliding = 1;
                    collidingVertices = commonArea.Vertices;
                end
                
                if  isColliding
                    %when cuboid around AffectActor and CauseActor intersect, it is taken as collision
                    
                    cacd = struct(); %CauseActorCollisionDetails
                    aacd = struct(); %AffectActorCollisionDetails

                    %time instant of collision
                    obj.Time = obj.AffectActor.simulatedTrajectory.Timestamps(i);
                    %store the position of front edge of the cuboid around CauseActor at the time of collision
                    cacd.FrontEdge = [CauseActorFLCorner(i,:);CauseActorFRCorner(i,:)];
                    %store the position of front edge of the cuboid around AffectActor at the time of collision
                    aacd.FrontEdge = [AffectActorFLCorner(i,:);AffectActorFRCorner(i,:)];
                    %store the position of back edge of the cuboid around CauseActor and AffectActor at the time of collision
                    cacd.BackEdge = [CauseActorBLCorner(i,:);CauseActorBRCorner(i,:)];
                    aacd.BackEdge = [AffectActorBLCorner(i,:);AffectActorBRCorner(i,:)];
                    cacd.LeftEdge = [CauseActorFLCorner(i,:);CauseActorBLCorner(i,:)];
                    cacd.RightEdge = [CauseActorFRCorner(i,:);CauseActorBRCorner(i,:)];
                    aacd.LeftEdge = [AffectActorFLCorner(i,:);AffectActorBLCorner(i,:)];
                    aacd.RightEdge = [AffectActorFRCorner(i,:);AffectActorBRCorner(i,:)];

                    %check for 90 degree collision
                    theta = acosd(dot(cacd.FrontEdge(1,:)- cacd.FrontEdge(2,:),aacd.FrontEdge(1,:) - aacd.FrontEdge(2,:)));
                    if  abs(theta - 90) > 5
                        error("Only 90 degree collisions are supported.")
                    end

                    %store CauseActor position
                    cacd.Waypoints = obj.CauseActorWaypoints(i,:);
                    %store taget position
                    aacd.Waypoints = obj.AffectActorWaypoints(i,:);

                    %calculate collision surfaces for cause and affect
                    %actor.
                    [cacd, aacd] = HelperCollision.calculateCollisionSurface(collidingVertices, cacd, aacd);
                    
                    pointofCollision = HelperCollision.calculatePointofCollision(collidingVertices, cacd); %point of collision is calculated on causeActor
                    obj.PointOfCollision = pointofCollision;

                    %calculate impact percentage of collision
                    cacd.collisionFraction = HelperCollision.calculatecollisionFraction(pointofCollision, cacd.collidingSurface);
                    aacd.collisionFraction = HelperCollision.calculatecollisionFraction(pointofCollision, aacd.collidingSurface);

                    if ~isnan(cacd.collisionFraction)
                        cacd.CollisionSurfaceLength = sqrt((cacd.collidingSurface(1, 1) - cacd.collidingSurface(2, 1))^2 + (cacd.collidingSurface(1, 2) - cacd.collidingSurface(2, 2))^2);
                    else
                        cacd.CollisionSurfaceLength = -1;
                    end

                    obj.extractedScenarioCollisionInformation.CauseActorCollisionDetails = cacd; %CauseActorCollisionDetails
                    obj.extractedScenarioCollisionInformation.AffectActorCollisionDetails = aacd; %AffectActorCollisionDetails
                    
                    break;
                end
            end

            warning('on','MATLAB:polyshape:repairedBySimplify');
            
        end     
                
    end

    methods(Static)

        function scenarioData = reRunScenario(scenarioData, VariantScenarioObj)
            %this method is used to first create a scenario from the
            %descriptor object and then reRun it, to get accurate
            %simulatedTrajectory.
            VariantScenarioObj = VariantScenarioObj.updateScenarioData(scenarioData);   
            scenario = getScenario(VariantScenarioObj,Simulator="DrivingScenario");

            VariantScenarioObj = getScenarioDescriptor(scenario,Simulator="DrivingScenario");
            scenarioData = VariantScenarioObj.getScenarioData();
        end

        function scenarioData = createCollision(scenarioData, actorsOfInterest, VariantScenarioDescriptor, methodForVariations)
            %this method is used to create a collision in the scenario. The
            %collision would only be created if the ego and target
            %trajectories intersect and there isnt a need to add waitTimes
            %to ego vehicle.

            causeActorData = scenarioData.entities.actors{actorsOfInterest.CauseActorID};
            cs = causeActorData.simulatedTrajectory;
            [CauseActorFLCorner,CauseActorFRCorner,CauseActorBLCorner,CauseActorBRCorner] = HelperCollision.computeactorBoundingBox(causeActorData, cs.Waypoints, ...
                cs.Yaw,cs.Pitch,cs.Roll);
            egoMidFront = (CauseActorFLCorner + CauseActorFRCorner)./2;
            egoMidBack = (CauseActorBLCorner + CauseActorBRCorner)./2;

            affectActorData = scenarioData.entities.actors{actorsOfInterest.AffectActorID};
            es = affectActorData.simulatedTrajectory;
            [AffectActorFLCorner,AffectActorFRCorner,AffectActorBLCorner,AffectActorBRCorner] = HelperCollision.computeactorBoundingBox(affectActorData, ...
                es.Waypoints, es.Yaw, es.Pitch, es.Roll);
            targetMid = (AffectActorFLCorner + AffectActorFRCorner)./2;

            egoNewTime = -1;
            targetNewTime = -1;
            found = 0;
            errorLimit = 0.3;

            %iterate through both actors Waypoints to find potential point
            %of intersection
            for i = 1:length(CauseActorFLCorner)
                for j = 1:length(AffectActorFLCorner)
                    if abs(targetMid(j,1)- egoMidFront(i,1)) < errorLimit && abs(targetMid(j,2) - egoMidFront(i,2)) < errorLimit
                        egoNewTime = cs.Timestamps(i);
                        targetNewTime = es.Timestamps(j);
                        found = 1;
                        break;
                    end

                end
                if found
                    break;
                end
            end

            if ~found %check collision with ego's back
                for i = 1:1:length(CauseActorFLCorner)
                    for j = 1:1:length(AffectActorFLCorner)
                        if abs(targetMid(j,1)- egoMidBack(i,1)) < errorLimit && abs(targetMid(j,2) - egoMidBack(i,2)) < errorLimit
                            egoNewTime = cs.Timestamps(i);
                            targetNewTime = es.Timestamps(j);
                            found = 1;
                            break;
                        end

                    end
                    if found
                        break;
                    end
                end
            end

            if ~found
                error("Ego and Target paths are not intersecting. Cannot create collision.");
            end

            if strcmp(methodForVariations, "WaitTime")
                if egoNewTime < targetNewTime
                    error("Cannot add waitTimes to Ego. Cannot generate collision for the given scenario.");
    %                     if ~isfield(causeActorData.route,'WaitTime') || isempty(causeActorData.route.WaitTime)
    %                         causeActorData.route.WaitTime = zeros(size(causeActorData.route.Waypoints, 1), 1);
    %                     end
    % 
    %                     %assign WaitTime to ego
    %                     causeActorData.route.WaitTime(1) = causeActorData.route.WaitTime(1) + (targetNewTime - egoNewTime);
    %                     scenarioData.entities.actors{obj.CauseActor.actorProfile.ActorID} = causeActorData;
    
                elseif egoNewTime > targetNewTime
                    if ~isfield(affectActorData.route,'WaitTime') || isempty(affectActorData.route.WaitTime)
                        affectActorData.route.WaitTime = zeros(size(affectActorData.route.Waypoints, 1), 1);
                    end
    
                    %assign WaitTime to ego
                    affectActorData.route.WaitTime(1) = affectActorData.route.WaitTime(1) + (egoNewTime - targetNewTime);
                    scenarioData.entities.actors{actorsOfInterest.AffectActorID} = affectActorData;
                end
            elseif strcmp(methodForVariations, "EntryTime")
                if egoNewTime < targetNewTime
                    causeActorData.route.EntryTime = causeActorData.route.EntryTime + (targetNewTime - egoNewTime);
                    scenarioData.entities.actors{actorsOfInterest.CauseActorID} = causeActorData;
                else
                    affectActorData.route.EntryTime = causeActorData.route.EntryTime + (egoNewTime - targetNewTime);
                    scenarioData.entities.actors{actorsOfInterest.AffectActorID} = affectActorData;
                end
            end

            scenarioData = HelperCollision.reRunScenario(scenarioData, VariantScenarioDescriptor);

            
        end

        function [egoTimeDiff,targetTimeDiff] = helperComputeArrivalTimeDiffWaittime(egoNewSpeed, time, causeActorData)
            %   This script computes modifications because of the ego actor's new Speed in terms of WaitTime

            %   This is a helper script for example purposes and may be modified in the future.
            %   Copyright 2021-2022 The MathWorks, Inc.
            egoTimeDiff = 0;
            targetTimeDiff = 0;
            % 'distanceTravelled' stores the distance travelled by ego till collision point
            distanceTravelled = 0;
            for i = 2:length(causeActorData.simulatedTrajectory.Waypoints)
                distanceTravelled = distanceTravelled + norm(causeActorData.simulatedTrajectory.Waypoints(i,:)- causeActorData.simulatedTrajectory.Waypoints(i-1,:));
                if causeActorData.simulatedTrajectory.Timestamps(i) >= time
                    break;
                end
            end

            egoNewTime = (distanceTravelled)/egoNewSpeed;

            if isfield(causeActorData.route,'WaitTime') && ~isempty(causeActorData.route.WaitTime)
                egoNewTime = egoNewTime + causeActorData.route.WaitTime(1);
            end

            if abs(time - egoNewTime) < 10e-4
                return
            end

            % If ego actor comes first at the collision point
            if time > egoNewTime
                egoTimeDiff = time - egoNewTime;
                % If the target comes first at the collision point
            elseif egoNewTime > time
                targetTimeDiff = egoNewTime - time;
            end
        end

        function [cacd, aacd] = calculateCollisionSurface(collidingPoints, cacd, aacd)
            %This function is used to find closest surfaces to vehicle, given a colliding point.

            surfaceNames = ["Front", "Right", "Back", "Left"];
            surfaceCauseActorCount = zeros([4, 1]);
            surfaceAffectActorCount = zeros([4, 1]);
            count = size(collidingPoints, 1);

            for i = 1:count
                vertice = collidingPoints(i, :);
                collisionSurfaceCauseActor = helperCalculateCollisionSurface(cacd.FrontEdge, cacd.RightEdge, ...
                    cacd.BackEdge, cacd.LeftEdge, vertice);
                collisionSurfaceAffectActor = helperCalculateCollisionSurface(aacd.FrontEdge, aacd.RightEdge, ...
                    aacd.BackEdge, aacd.LeftEdge, vertice);

                if any(collisionSurfaceCauseActor) && any(collisionSurfaceAffectActor)
                    surfaceCauseActorCount(collisionSurfaceCauseActor) = surfaceCauseActorCount(collisionSurfaceCauseActor) + 1;
                    surfaceAffectActorCount(collisionSurfaceAffectActor) = surfaceAffectActorCount(collisionSurfaceAffectActor) + 1;
                end
            end

            %if multiple possible surfaces, we take  the first one
            surfaces = [find(surfaceCauseActorCount == max(surfaceCauseActorCount), 1), find(surfaceAffectActorCount == max(surfaceAffectActorCount), 1)]; 
            CauseActorCollidingSurfaceName = surfaceNames(surfaces(1));
            AffectActorCollidingSurfaceName = surfaceNames(surfaces(2));


            CauseActorCollidingSurface = cacd.FrontEdge;
            if surfaces(1) == 2
                CauseActorCollidingSurface = cacd.RightEdge;
            elseif surfaces(1) == 3
                CauseActorCollidingSurface = cacd.BackEdge;
            elseif surfaces(1) == 4
                CauseActorCollidingSurface = cacd.LeftEdge;
            end

            AffectActorCollidingSurface = aacd.FrontEdge;
            if surfaces(2) == 2
                AffectActorCollidingSurface = aacd.RightEdge;
            elseif surfaces(2) == 3
                AffectActorCollidingSurface = aacd.BackEdge;
            elseif surfaces(2) == 4
                AffectActorCollidingSurface = aacd.LeftEdge;
            end

            cacd.collidingSurfaceName = CauseActorCollidingSurfaceName;
            aacd.collidingSurfaceName = AffectActorCollidingSurfaceName;
            cacd.collidingSurface = CauseActorCollidingSurface;
            aacd.collidingSurface = AffectActorCollidingSurface;

            
            function collisionSurface = helperCalculateCollisionSurface(leftSurface, rightSurface, frontSurface, backSurface, vertice)
                %this function is used to find which surface the
                %collisionPoint lies on.

                collision = zeros([4, 1]);
                collision(1) = checkOnLineSegment(leftSurface(1, :), leftSurface(2, :), vertice);
                collision(2) = checkOnLineSegment(rightSurface(1, :), rightSurface(2, :), vertice);
                collision(3) = checkOnLineSegment(frontSurface(1, :), frontSurface(2, :), vertice);
                collision(4) = checkOnLineSegment(backSurface(1, :), backSurface(2, :), vertice);
                collisionSurface = find(collision - min(collision) < 1e-1);
    
            end

            function error = checkOnLineSegment(point1, point2, vertice)
                x = vertice(1);y = vertice(2);
                x1 = point1(1);y1 = point1(2);
                x2 = point2(1);y2 = point2(2);
                AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
                AP = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
                PB = sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y));
                error =  abs(AB - AP - PB);
            end
        end

        function  collisionFraction = calculatecollisionFraction(pointofCollision, collidingSurface)
            %IMPACT PERCENTAGE : DEFINED AS THE DISTANCE OF THE POINT OF COLLISION WITH
            %ACTOR'S COLLIDING SURFACE'S FRONT (or LEFT) CORNER AT THE
            %TIME OF COLLISION, DIVIDED BY ACTOR COLLIDING SURFACE LENGTH 
            collisionFraction = NaN;
            x = collidingSurface(1, 1);y = collidingSurface(1, 2);
            x1 = pointofCollision(1); y1 = pointofCollision(2);
            collidingSurfaceLength = sqrt((collidingSurface(1, 1) - collidingSurface(2, 1))^2 + (collidingSurface(1, 2) - collidingSurface(2, 2))^2);
            if collidingSurfaceLength ~= 0
                collisionFraction = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))*100/collidingSurfaceLength;
            end

                           
        end

        function pointOfCollision = calculatePointofCollision(pointsOfCollision, cacd)
            %collisionPoint: Defined as the point, in the set of colliding vertices, closest to the front (or left) of
            %the CauseActor's colliding surface
            count = size(pointsOfCollision, 1);
            minDistance = 1e7;

            for i = 1:count
                vertice = pointsOfCollision(i, 1:2);
                distanceFromCorner = sqrt((cacd.collidingSurface(1, 1) - vertice(1))^2 + (cacd.collidingSurface(1, 2) - vertice(2))^2);
                if distanceFromCorner < minDistance
                    pointOfCollision = vertice;
                    minDistance = distanceFromCorner;
                end               
            end

        end

        function [frontLeft,frontRight,backLeft,backRight] = computeactorBoundingBox(actorData, positions, Yaw, Pitch, Roll)
            % This function computes the world co-ordinates of the corners of the cuboid around the
            % actors
            % It makes use of actor's dimensions and it's positions to
            % computes the co-ordinates
            count = length(positions);
            classID = actorData.actorProfile.ClassID;
            if classID == 1 || classID == 2 || classID == 3 %check for vehicle
                frontLeft = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([ actorData.actorProfile.Dimensions.Wheelbase + actorData.actorProfile.Dimensions.FrontOverhang,actorData.actorProfile.Dimensions.Width/2,0],count,1));
                frontRight = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([ actorData.actorProfile.Dimensions.Wheelbase + actorData.actorProfile.Dimensions.FrontOverhang,-1*actorData.actorProfile.Dimensions.Width/2,0],count,1));
                backLeft = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([ -1*(actorData.actorProfile.Dimensions.RearOverhang),actorData.actorProfile.Dimensions.Width/2,0],count,1));
                backRight = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([ -1*(actorData.actorProfile.Dimensions.RearOverhang),-1*actorData.actorProfile.Dimensions.Width/2,0],count,1));
            else
                frontLeft = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([actorData.actorProfile.Dimensions.Length/2,actorData.actorProfile.Dimensions.Width/2,0],count,1));
                frontRight = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([actorData.actorProfile.Dimensions.Length/2,-1*actorData.actorProfile.Dimensions.Width/2,0],count,1));
                backLeft = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([-1*actorData.actorProfile.Dimensions.Length/2,actorData.actorProfile.Dimensions.Width/2,0],count,1));
                backRight = HelperCollision.convertToWorld(positions,Yaw,Pitch,Roll, ...
                    repmat([-1*actorData.actorProfile.Dimensions.Length/2,-1*actorData.actorProfile.Dimensions.Width/2,0],count,1));
            end
        end

        function pointsInWorld = convertToWorld(positions,Yaw,Pitch,Roll,points)
            %This function converts points from actor's frame to World
            %frame using actor's pose and positions

            % Function to compute rotation around x-axis
            rotx = @(x) [1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];
            % Function to compute rotation around y-axis
            roty = @(y) [cos(y)  0 sin(y); 0 1 0; -sin(y) 0 cos(y)];
            % Function to compute rotation around z-axis
            rotz = @(z)[cos(z) -sin(z) 0;sin(z) cos(z) 0;0 0 1];
            count = size(points,1);
            pointsInWorld = zeros(size(points));
            for i= 1:count
                Rz = rotz(deg2rad(Yaw(i)));
                Ry = roty(deg2rad(Pitch(i)));
                Rx = rotx(deg2rad(Roll(i)));
                R = Rz*Ry*Rx;
                R = R';
                newPoint = points(i,:)*R + [positions(i,1:2),0];
                pointsInWorld(i,:) = newPoint;
            end
        end

        function distance = distanceBetweenParallelLines(line1,line2)
            %Function to compute distance between two parallel 2-D lines
            slope = HelperCollision.computeSlope(line1);
            if slope == Inf
                distance =  abs(line1(2,1) - line2(2,1));
                return;
            end
            if slope == 0
                distance = abs(line1(2,2) - line2(2,2));
                return;
            end
            intercept1 = HelperCollision.computeIntercept(line1,slope);
            intercept2 = HelperCollision.computeIntercept(line2,slope);
            distance = abs(intercept2 - intercept1)/sqrt(1 + slope*slope);
        end

        function intercept = computeIntercept(line,slope)
            %Function to compute intercept of a 2-D line
            if slope == Inf
                intercept = Inf;
                return;
            end
            intercept = line(1,2) - slope*line(1,1);
        end

        function slope = computeSlope(line)
            %Function to compute slope of a 2-D line
            if line(2,1) == line(1,1)
                slope = Inf;
                return;
            end
            slope = (line(2,2)-line(1,2)/(line(2,1) - line(1,1)));
        end
    end

end
