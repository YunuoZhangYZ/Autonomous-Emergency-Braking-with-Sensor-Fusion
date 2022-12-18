classdef HelperEventVariation < handle
    % HELPEREVENTVARIATION Class to hold variation parameters for events in 
    % the scenario. Use this class to add event related variations in the
    % scenario.
    % 
    % This is a Class for the Scenario Variant Generator for Automated 
    % Driving Toolbox.
    % This is a helper for example purposes and may be modified in the 
    % future.

    % Copyright 2022 The MathWorks, Inc.
    
    properties
         
        InteractionEventGraph; % Connections between edges.      
        
    end
    


    
    methods
        function obj = HelperEventVariation()
            
            
            obj.InteractionEventGraph = [];

           % obj.InteractionTimeGraph = buildInteractionTimeGraph(obj);
            
            

        end

        function obj = addCollision(obj, scenarioVariantObj, CauseActorID, AffectActorID, options)

            arguments
                obj 
                scenarioVariantObj HelperScenarioVariant {mustBeNonempty}
                CauseActorID  double  {mustBeNumeric, mustBePositive}
                AffectActorID double {mustBeNumeric, mustBePositive}
                options.CollisionPoint double {mustBeNumeric, mustBeGreaterThanOrEqual(options.CollisionPoint, 0), mustBeLessThanOrEqual(options.CollisionPoint, 1)}
            end

            scenarioData = scenarioVariantObj.ScenarioData;

            actorsOfInterest.CauseActorID = CauseActorID;
            actorsOfInterest.AffectActorID = AffectActorID;
            if ~isfield(options, 'CollisionPoint')
                pcObj = HelperCollision(scenarioData, actorsOfInterest, scenarioVariantObj.VariantScenarioDescriptor, Method = scenarioVariantObj.MethodForVariations);
            else
                pcObj = HelperCollision(scenarioData, actorsOfInterest, scenarioVariantObj.VariantScenarioDescriptor, Method=scenarioVariantObj.MethodForVariations, CauseActorCollisionFraction=options.CollisionPoint);
            end
            
            if size(obj.InteractionEventGraph, 1) == 0
                obj.InteractionEventGraph(1).CollisionObj = pcObj;
            else
                obj.InteractionEventGraph(end).CollisionObj = pcObj;
            end

        end

        function obj = addTrajectoryTurnVariation(obj, scenarioVariantObj, CauseActorID, AffectActorID)

            arguments
                obj 
                scenarioVariantObj HelperScenarioVariant {mustBeNonempty}
                CauseActorID  double  {mustBeNumeric, mustBePositive}
                AffectActorID double {mustBeNumeric, mustBePositive}
            end

            scenarioData = scenarioVariantObj.ScenarioData;

            actorsOfInterest.CauseActorID = CauseActorID;
            actorsOfInterest.AffectActorID = AffectActorID;
            ttObj = HelperTrajectoryatTurn(scenarioData, actorsOfInterest, scenarioVariantObj.VariantScenarioDescriptor, Method = scenarioVariantObj.MethodForVariations);

            if size(obj.InteractionEventGraph, 1) == 0
                obj.InteractionEventGraph(1).TrajectoryTurnObj = ttObj;
            else
                %                 obj.InteractionEventGraph(end+1).TrajectoryTurnObj = ttObj;
                obj.InteractionEventGraph(end).TrajectoryTurnObj = ttObj; %currently multi event variations are not supported
            end
        end

        function obj = addCutInCollision(obj, scenarioVariantObj, CauseActorID, AffectActorID,TTC)

            arguments

                obj

                scenarioVariantObj HelperScenarioVariant {mustBeNonempty}

                CauseActorID double {mustBeNumeric}

                AffectActorID double {mustBeNumeric}

                TTC double

            end

            scenarioData = scenarioVariantObj.ScenarioData;

            actorsOfInterest.CauseActorID = CauseActorID;

            actorsOfInterest.AffectActorID = AffectActorID;

            ccObj = HelperCutInCollision(scenarioData, actorsOfInterest,"TTC",TTC);

            % pcObj = PotentialCollision(CauseActor, AffectActor);

            if size(obj.InteractionEventGraph, 1) == 0

                obj.InteractionEventGraph(1).CutInCollisionObj = ccObj;

            else

                obj.InteractionEventGraph(end+1).CutInCollisionObj = ccObj;

            end

        end

        function obj = addHeadOnCollision(obj, scenarioVariantObj, CauseActorID, AffectActorID)

            arguments

                obj

                scenarioVariantObj HelperScenarioVariant {mustBeNonempty}

                CauseActorID double {mustBeNumeric}

                AffectActorID double {mustBeNumeric}

            end

            scenarioData = scenarioVariantObj.ScenarioData;

            actorsOfInterest.CauseActorID = CauseActorID;

            actorsOfInterest.AffectActorID = AffectActorID;

            hcObj = HelperHeadOnCollision(scenarioData, actorsOfInterest);

            % pcObj = PotentialCollision(CauseActor, AffectActor);

            if size(obj.InteractionEventGraph, 1) == 0

                obj.InteractionEventGraph(1).HeadOnCollisionObj = hcObj;

            else

                obj.InteractionEventGraph(end+1).HeadOnCollisionObj = hcObj;

            end

        end

        function obj = addMultiEventVariation(obj, CauseActor, AffectActor)

          
        end 


        %Fetches the top priority event
        function topPriorityEvent = getTopPriorityEvent(obj)
            topPriorityEvent = obj.InteractionEventGraph(1);
            for i = 1:size(obj.InteractionEventGraph)
                timestamp = obj.InteractionEventGraph(i).timestamp;
                if (timestamp < topPriorityEvent.timestamp)
                    topPriorityEvent = obj.InteractionEventGraph(i);
                end

            end


        end

        
        

        function graph = buildInteractionTimeGraph(obj)
            % we need actor of interest, time stamps, all pairs of actors. 
            % User will input us the pairs of actors interacting with each
            % other. 
              % Scenario reader. 
            % User will input us the pairs of actors interacting with each
            % other. 
            % Default input can be takne from seed scenario. 
            graph = ones(1,1);
        end
        
        function graph = buildInteractionEventGraph(obj)
            % we need actor of interest, Timestamps(o), interaction between
            % all pairs of actors.
            % Scenario reader. 
            % User will input us the pairs of actors interacting with each
            % other. 
            % Default input can be takne from seed scenario


            %This graph can be constructue by accessing PCObjects and
            %fetching underlying informaiton. Like cause, effect. 
 
            graph = ones(1,1);
        end
        
    end
end

