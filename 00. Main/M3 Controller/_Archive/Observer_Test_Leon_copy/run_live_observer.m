
%% run_live_observer.m - Start the Observer live simulation

%% Project paths - portable across team members

% Folder containing this script
scriptFolder = fileparts(mfilename('fullpath'));

% Automatically detect Observer_Test folder
if isfolder(fullfile(scriptFolder, 'functions')) && ...
        isfolder(fullfile(scriptFolder, 'models'))

    % Script is directly inside Observer_Test
    observerRoot = scriptFolder;

elseif isfolder(fullfile(scriptFolder, 'Observer_Test', 'functions')) && ...
        isfolder(fullfile(scriptFolder, 'Observer_Test', 'models'))

    % Script is directly inside M3 Controller
    observerRoot = fullfile(scriptFolder, 'Observer_Test');

else
    error('Could not locate Observer_Test with its functions and models folders.');
end

functionsFolder = fullfile(observerRoot, 'functions');
modelsFolder    = fullfile(observerRoot, 'models');

% Automatically find project root (00. Main)
projectRoot = scriptFolder;

while true

    if isfolder(fullfile(projectRoot, 'M1 Model')) && ...
            isfolder(fullfile(projectRoot, 'M4 Competition'))
        break
    end

    parentFolder = fileparts(projectRoot);

    if isempty(parentFolder) || strcmp(parentFolder, projectRoot)
        error('Could not find project root containing M1 Model and M4 Competition.');
    end

    projectRoot = parentFolder;

end

% Temporarily use project root as current folder
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(projectRoot);

% Add Observer_Test folders at the beginning of the MATLAB path
addpath(functionsFolder, '-begin');
addpath(modelsFolder, '-begin');

% Clear cached functions
clear live_test_observer
clear LGQI_run_Observer
rehash;

% Verify selected live_test_observer.m
expectedFile = fullfile(functionsFolder, 'live_test_observer.m');
selectedFile = which('live_test_observer');

fprintf('\nProject root:\n%s\n', projectRoot);
fprintf('\nObserver root:\n%s\n', observerRoot);
fprintf('\nSelected live_test_observer:\n%s\n', selectedFile);

if isempty(selectedFile) || ~strcmpi(selectedFile, expectedFile)
    error('Wrong live_test_observer.m selected! Expected: %s', expectedFile);
end

%% Simulation settings

ctrlFile = fullfile(observerRoot,'controller1_Observer.mat');

if ~isfile(ctrlFile)
    error('Controller file not found: %s',ctrlFile);
end

P_on     = 1200;
rand_on  = [4 12];
rand_off = [4 12];
du_min   = -30;
window   = 40;

%% Start live simulation

live_test_observer(ctrlFile, ...
    P_on=P_on, ...
    rand_on=rand_on, ...
    rand_off=rand_off, ...
    du_min=du_min, ...
    window=window);
