classdef ZieglerNichols

    properties
        Process % Must be a tf
        time    {mustBeNumeric}
    end
    
    % Private Properties
    properties (Access = private)
        % Open loop output
        output
        
        % Tangent Line of the Inflexion Point
        tangentEq

        theta   {mustBeNumeric}
        tau     {mustBeNumeric}
        k       {mustBeNumeric}
    end
    
    methods
        % Constructor
        function obj = ZieglerNichols(process, time)
            obj.Process = process;
            obj.time = time;
            obj = obj.estimateProperties();
        end
        
        % Here we'll estimate theta, tau and k automatically
        function obj = estimateProperties(obj) 
            [obj.output, obj.time] = obj.getOpenLoopOutput();
            [M, I] = obj.getGreatestAngularCoefficient();
            obj.tangentEq = obj.getTangentEquation(M, I);
            [obj.k, obj.theta, obj.tau] = obj.getParametersFromTangent();
        end
        
        function [output, time] = getOpenLoopOutput(obj)
            [output, time] = step(obj.Process, obj.time);
        end
        
        function [M, I] = getGreatestAngularCoefficient(obj)
            doutput = diff(obj.output)./diff(obj.time);
            [M, I] = max(doutput);
        end
        
        function tangentEq = getTangentEquation(obj, M, I)
            b = obj.output(I) - obj.time(I)*M;
            tangentEq = M*obj.time + b;
        end
        
        function [k, theta, tau] = getParametersFromTangent(obj)
            k = obj.estimateK();
            theta = obj.estimateTheta();
            tau = obj.estimateTau();
        end
        
        function k = estimateK(obj)
            maxOutput = obj.output(length(obj.output));
            k = maxOutput / 1; % step
        end
        
        function theta = estimateTheta(obj)
            indice_theta = find(obj.tangentEq > 0, 1, 'first');
            theta = obj.time(indice_theta);
        end
        
        function tau = estimateTau(obj)
            maxOutput = obj.output(length(obj.output));
            indice_tau = find(obj.tangentEq > maxOutput, 1, 'first');
            tau = obj.time(indice_tau);
        end
        
        % Here we'll output PID parameters
        function outputArg = getPIDParameters(obj)
            Kp = obj.calculateKp();
            Ti = obj.calculateTi();
            Td = obj.calculateTd();
            outputArg = PIDParameters(Kp, Ti, Td);
        end
     
        function Kp = calculateKp(obj)
            Kp = (1.2 * obj.tau) / (obj.k * obj.theta);
        end
        
        function Ti = calculateTi(obj)
            Ti = 2 * obj.theta;
        end
        
        function Td = calculateTd(obj)
            Td = 0.5 * obj.theta;
        end
        
        function plotProcessAndTangent(obj)
            plot(obj.time, obj.output);
            hold on
            plot(obj.time, obj.tangentEq);
            hold off
            maxOutput = obj.output(length(obj.output));
            ylim([0 maxOutput]);
            hold on
            s = tf('s');
            process_approx = obj.k / (obj.tau * s + 1) * exp(-s * obj.theta);
            output_approx = step(process_approx, obj.time);
            plot(obj.time, output_approx);
            hold off
        end
 
    end

end