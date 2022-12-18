classdef HelperScenarioVariant < handle
    % HELPERSCENARIOVARIANT Root class, used to generate variants from a
    % scenario. This class holds all the variation parameters such as 
    % Scene, Actor and Event which are required to generate variants of 
    % a scenario.
    % 
    % This is a Class for the Scenario Variant Generator for Automated 
    % Driving Toolbox.
    % This is a helper for example purposes and may be modified in the 
    % future.

    % Copyright 2022 The MathWorks, Inc.

    properties
        ScenarioData
        VariantGenerator
        ActorVariations
        SceneVariations
        EventVariations
        MethodForVariations
    end

    properties(Hidden = true)
        VariantScenarioDescriptor;
    end

    methods
        function obj = HelperScenarioVariant(VariantScenarioDescriptor, options)
            %SCENARIOVARIANT Construct an instance of this class
            %   Detailed explanation goes here

            arguments
                VariantScenarioDescriptor variantgenerator.internal.ScenarioDescriptor
                options.method (1, 1) string {mustBeMember(options.method,["WaitTime","EntryTime"])} = "WaitTime"
            end
            obj.ScenarioData = VariantScenarioDescriptor.getScenarioData();
            obj.VariantScenarioDescriptor = VariantScenarioDescriptor;
            obj.MethodForVariations = options.method;

        end

        function descriptorArr = getScenarioVariantDescriptors(obj)
            sizeOfDatas = size(obj.VariantGenerator,2);
            descriptorArr = cell(sizeOfDatas, 1);
            for i = 1:sizeOfDatas
                dsObj = obj.VariantScenarioDescriptor;

                dsObj = dsObj.updateScenarioData(obj.VariantGenerator(i).scenarioData);

                descriptorArr{i} = dsObj;
            end
        end

        function obj = show(obj, titleString)
            allvariants = obj.VariantGenerator;
            sizeOfDatas = size(allvariants,2);

            for i = 1:sizeOfDatas
                dsObj = obj.VariantScenarioDescriptor;

                dsObj = dsObj.updateScenarioData(allvariants(i).scenarioData);

                scenario = getScenario(dsObj,Simulator="DrivingScenario");

                restart(scenario);

                figScene = figure;
                set(figScene,Position=[50 50 500 500]);
                hPanel1 = uipanel(figScene,Position=[0 0 1 1]);
                hPlot1 = axes(hPanel1);
                plot(scenario,Waypoints="on",Parent=hPlot1)
                title("Scenario Variant: " + titleString)
                while advance(scenario)
                    pause(0.01)
                end

            end

        end

        function descriptorArr = generateVariants(obj)
            %This function is used to generate Variants using the variation
            %parameters specified. Actor variations are first generated,
            %followed by event variations. Scene variations are not
            %supported as of now. 

            %Use the actorVariation object to iterate through all the actor
            %variation params and generate variations.
            obj.VariantGenerator = obj.ActorVariations.ActorVariationGraph;
            for i = 1:size(obj.ActorVariations.ActorVariationGraph, 2)
                scenarioData = obj.ScenarioData;

                for fn = fieldnames(obj.ActorVariations.ActorVariationGraph(i))'
                    %                     disp(fn)
                    varObj = obj.ActorVariations.ActorVariationGraph(i).(fn{1});
                    if size(varObj, 2) <= 1
                        if ~isempty(varObj)
                            scenarioData = varObj.applyVariation(scenarioData);
                        end
                    else
                        for i2 = 1:size(varObj, 2)
                            scenarioData = varObj(i2).applyVariation(scenarioData);
                        end
                    end
                end
                obj.VariantGenerator(i).scenarioData = scenarioData;
            end

            %Use the EventVariation object to iterate through all the event
            %variation params and generate variations.
            if ~isempty(obj.EventVariations)
                for i = 1:size(obj.ActorVariations.ActorVariationGraph, 2)
                    scenarioData = obj.VariantGenerator(i).scenarioData;
                    for i2 = 1:size(obj.EventVariations.InteractionEventGraph, 2)
                        for fn = fieldnames(obj.EventVariations.InteractionEventGraph(i2))'
                            if fn{1} == "TrajectoryTurnObj"
                                varObj = obj.EventVariations.InteractionEventGraph(i2).(fn{1});
                                [scenarioData, updatedDSFlag] = varObj.applyVariation(scenarioData);
                                if isfield(obj.EventVariations.InteractionEventGraph(i2), 'CollisionObj')
                                    collisionObj = obj.EventVariations.InteractionEventGraph(i2).CollisionObj;
                                    actorsOfInterest.CauseActorID = collisionObj.CauseActor.actorProfile.ActorID;
                                    actorsOfInterest.AffectActorID = collisionObj.AffectActor.actorProfile.ActorID;
                                    if ~updatedDSFlag
                                        obj.EventVariations.InteractionEventGraph(i2).CollisionObj = ...
                                            HelperCollision(obj.ScenarioData, actorsOfInterest, obj.VariantScenarioDescriptor, "Method", collisionObj.Method, ...
                                            "CauseActorCollisionFraction", collisionObj.CauseActorCollisionFraction);
                                    else
                                        obj.EventVariations.InteractionEventGraph(i2).CollisionObj = ...
                                            HelperCollision(scenarioData, actorsOfInterest,  obj.VariantScenarioDescriptor, "Method", collisionObj.Method, ...
                                            "CauseActorCollisionFraction", collisionObj.CauseActorCollisionFraction);
                                    end
                                end
                            else
                                varObj = obj.EventVariations.InteractionEventGraph(i2).(fn{1});
                                scenarioData = varObj.applyVariation(scenarioData);
                            end
                        end
                    end
                    obj.VariantGenerator(i).scenarioData = scenarioData;
                end
            end

            descriptorArr = obj.getScenarioVariantDescriptors;
        end
    end
end

