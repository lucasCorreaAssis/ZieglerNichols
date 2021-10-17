classdef PIDParameters
    properties
        Kp
        Ti
        Td
    end
    
    methods
        function obj = PIDParameters(Kp, Ti, Td)
            obj.Kp = Kp;
            obj.Ti = Ti;
            obj.Td = Td;
        end
    end
end
