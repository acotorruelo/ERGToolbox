% Automatic installer for the ERG toolbox. This installer copies all needed
% files for the execution of the ERGT to a folder of the user's choosing.
% The directory is then added to MATLAB's path.

installpath=uigetdir(pwd,'Choose where ERGT is going to be installed');
disp('Copying files...')
[status,msg]=copyfile('@ERGmultiregion',[installpath '\@ERGmultiregion']);
if ~status
    error(['Error copying files: ' msg])
end
[status,msg]=copyfile('@ERGregion',[installpath '\@ERGregion']);
if ~status
    error(['Error copying files: ' msg])
end
[status,msg]=copyfile('@ERGsys',[installpath '\@ERGsys']);
if ~status
    error(['Error copying files: ' msg])
end
[status,msg]=copyfile('simulink',[installpath '\simulink']);
if ~status
    error(['Error copying files: ' msg])
end
disp('DONE')
disp('Adding ERGT to path...')
addpath(genpath(installpath),'-end');
disp('DONE')
disp('Saving changes...')
savepath
disp('DONE')