function [t, v, x0] = disturbances_old(pathData,flagPlot, debug)
%     pathData = 'Data/';
    if nargin < 2 
        flagPlot = false;
        debug = false;
    end
    
    fileName = [pathData '/disturbances/preComputed.mat'];
    varNames = {'sim.winBusOut[1].AbsQFlow[1]', ... 
'sim.winBusOut[1].AbsQFlow[2]', ... 
'sim.winBusOut[1].AbsQFlow[3]', ... 
'sim.winBusOut[1].iSolDir', ... 
'sim.winBusOut[1].iSolDif', ... 
'sim.winBusOut[2].AbsQFlow[1]', ... 
'sim.winBusOut[2].AbsQFlow[2]', ... 
'sim.winBusOut[2].AbsQFlow[3]', ... 
'sim.winBusOut[2].iSolDir', ... 
'sim.winBusOut[2].iSolDif', ... 
'sim.winBusOut[3].AbsQFlow[1]', ... 
'sim.winBusOut[3].AbsQFlow[2]', ... 
'sim.winBusOut[3].AbsQFlow[3]', ... 
'sim.winBusOut[3].iSolDir', ... 
'sim.winBusOut[3].iSolDif', ... 
'sim.winBusOut[4].AbsQFlow[1]', ... 
'sim.winBusOut[4].AbsQFlow[2]', ... 
'sim.winBusOut[4].AbsQFlow[3]', ... 
'sim.winBusOut[4].iSolDir', ... 
'sim.winBusOut[4].iSolDif', ... 
'sim.winBusOut[5].AbsQFlow[1]', ... 
'sim.winBusOut[5].AbsQFlow[2]', ... 
'sim.winBusOut[5].AbsQFlow[3]', ... 
'sim.winBusOut[5].iSolDir', ... 
'sim.winBusOut[5].iSolDif', ... 
'weaBus.solBus[1].iSolDir', ... 
'weaBus.solBus[1].iSolDif', ... 
'weaBus.solBus[1].Tenv', ... 
'weaBus.solBus[2].iSolDir', ... 
'weaBus.solBus[2].iSolDif', ... 
'weaBus.solBus[2].Tenv', ... 
'weaBus.solBus[3].iSolDir', ... 
'weaBus.solBus[3].iSolDif', ... 
'weaBus.solBus[3].Tenv', ... 
'weaBus.solBus[4].iSolDir', ... 
'weaBus.solBus[4].iSolDif', ... 
'weaBus.solBus[4].Tenv', ... 
'weaBus.solBus[5].iSolDir', ... 
'weaBus.solBus[5].iSolDif', ... 
'weaBus.solBus[5].Tenv', ... 
'weaBus.Te', ... 
'weaBus.hConExt', ... 
'weaBus.dummy', ... 
'weaBus.TGroundDes'
    };

    t = findOutput(fileName, 'Time' )';
    v = zeros(length(t),length(varNames));
    for i=1:length(varNames)
        if debug
            temp1 = strfind(varNames(i),'Tenv');
            temp2 = strfind(varNames(i),'Te');
            temp3 = strfind(varNames(i),'TGroundDes');
            temp4 = strfind(varNames(i),'dummy');
            if ~isempty(temp1{1}) || ~isempty(temp2{1}) || ~isempty(temp3{1})
                v(:,i) = ones(length(t),1).*293.15;
            elseif ~isempty(temp4{1})
                v(:,i) = ones(length(t),1);
            else
                v(:,i) = zeros(length(t),1);
            end
        else
            v(:,i) = findOutput(fileName,varNames(i))';
        end
    end
    %Remove last point because not equidistant
    t = t(1:end-1);
    v = v(1:end-1,:); 
          
%     initial values
    x0 = 293.15;
    
    if flagPlot
        indISolDir = 26:3:38;
        indISolDif = 27:3:39;
        figure()
        plot( t*ones(1,length(indISolDir)) , v(:,indISolDir) ), hold on
        plot(t,v(:,indISolDif)), hold off
        legend([varNames(indISolDir) varNames(indISolDif)])
        xlabel('t [s]')
        ylabel('Direct and diffuse [W/m2]')
        
        indTEnv = [28:3:40];
        indTe = 41;
        figure()
        plot(t,v(:,indTEnv)), hold on
        plot(t,v(:,indTe),'k','linewidth',2), hold off
        legend([varNames(indTEnv) varNames(indTe)])
        xlabel('t [s]')
        ylabel('Environment and ambient temperature [K]')
        
        
        indQAbsAndISolWin = 1:25;
        figure()
        plot(t,v(:,indQAbsAndISolWin)), hold on
        legend([varNames(indQAbsAndISolWin)])
        xlabel('t [s]')
        ylabel('Absorbed, direct and diffuse heat through window [W]')
    end
end