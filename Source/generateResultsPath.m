function resultsPath = generateResultsPath(sim)
%GENERATERESULTSDIRCETORY Function checks results folder existis and 
% creates path later used to save plot

resultsPath = ['.' filesep 'Results'];

if ~exist(resultsPath, 'dir')
   mkdir(resultsPath)
end

formatOut = 'yy_mm_dd_HH_MM_SS';
date_time = datestr(now,formatOut);

resultsPath = [resultsPath filesep date_time];

if ~exist(resultsPath, 'dir')
   mkdir(resultsPath)
end

fileName = '';
fileName = [fileName 'ZCBF'];


if(sim.considerMinimumBreakingDistance)
    fileName = [fileName '_MBD_ON'];
else
    fileName = [fileName '_MBD_OFF'];
end

resultsPath = [resultsPath filesep fileName];

end
