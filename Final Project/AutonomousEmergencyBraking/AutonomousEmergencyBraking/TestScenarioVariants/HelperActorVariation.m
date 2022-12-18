classdef HelperActorVariation < handle
    % HELPERACTORVARIATION Class to hold variation parameters for actors 
    % in the scenario. Use this class to add actor related variations in 
    % the scenario.
    % 
    % This is a Class for the Scenario Variant Generator for Automated 
    % Driving Toolbox.
    % This is a helper for example purposes and may be modified in the 
    % future.

    % Copyright 2022 The MathWorks, Inc.

    properties
        ActorVariationGraph;
    end

    methods
        function obj = HelperActorVariation()
            obj.ActorVariationGraph = [];
        end

        function obj = addDimensionVariation(obj,ActorID, Dimension)

            dimVarObj = HelperDimensionVariation(ActorID,Dimension);
            obj.ActorVariationGraph(end+1).DimensionObj = dimVarObj;


        end

        function obj = addSpeedVariation(obj,ActorID,Speed)
            SpeedVarObj = HelperSpeedVariation( ActorID,Speed);
            obj.ActorVariationGraph(end+1).SpeedObj = SpeedVarObj;

        end

        function obj = addWaypointVariation(obj,ActorID, Waypoints)
            waypointVarObj = HelperWaypointsVariation(ActorID, Waypoints);

            obj.ActorVariationGraph(end+1).WaypointsObj = waypointVarObj;

        end

        function obj = addYawVariation(obj,ActorID, Yaw)
            yawVarObj = HelperYawVariation(ActorID, Yaw);
            obj.ActorVariationGraph(end+1).YawObj = yawVarObj;
        end

        function obj = addStartPositionVariation(obj,ActorID,StartPosition)
            startPosObj = HelperStartPositionVariation(ActorID,StartPosition);
            obj.ActorVariationGraph(end+1).StartPositionObj = startPosObj;
        end

        function obj = addMultiVariation(obj,ActorID,options)
            % Variations can be made to multiple actors or multiple variations
            % can be combined for a particular actor using the 'addMultiVariation'.

            arguments
                obj
                ActorID cell
                options.Dimension cell
                options.Waypoints  cell
                options.Speed  cell
                options.StartPosition  cell
                options.Yaw cell
            end
            % FYI : output of this function
            %    DimensionObj: [1×1 HelperDimensionVariation]
            %         WaypointsObj: [1×1 HelperWaypointsVariation]
            %             SpeedObj: [1×1 HelperSpeedVariation]
            %     StartPositionObj: [1×1 HelperStartPositionVariation]
            %               YawObj: [1×1 HelperYawVariation]

            indexEnd = size(obj.ActorVariationGraph,2);

            if isfield(options,"Dimension")
                dimVarObj = HelperDimensionVariation(ActorID, options.Dimension);
                obj.ActorVariationGraph(indexEnd+1).DimensionObj = dimVarObj;
            end
            if isfield(options,"Waypoints")
                waypointVarObj=HelperWaypointsVariation(ActorID, options.Waypoints);
                obj.ActorVariationGraph(indexEnd+1).WaypointsObj = waypointVarObj;
            end
            if isfield(options,"Speed")
                SpeedVarObj = HelperSpeedVariation(ActorID, options.Speed);
                obj.ActorVariationGraph(indexEnd+1).SpeedObj = SpeedVarObj;
            end
            if isfield(options,"StartPosition")
                startPosObj = HelperStartPositionVariation(ActorID, options.StartPosition);
                obj.ActorVariationGraph(indexEnd+1).StartPositionObj = startPosObj;
            end
            if isfield(options,"Yaw")
                yawVarObj = HelperYawVariation(ActorID, options.Yaw);
                obj.ActorVariationGraph(indexEnd+1).YawObj = yawVarObj;
            end
        end
    end
end
