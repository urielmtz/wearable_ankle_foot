%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% University of Leeds
% School of Mechanical Engineering
% Institute of Design, Robotics and Optimisation (iDRO)
%
% author: Uriel Martinez-Hernandez
% program name: run_classifier.m
% date: February 2018
% version: 1.0
%
% This program is part of the project 'Wearable soft robotics for
% independent living' funded by EPSRC.
%
% Brief description:
% Recognition of heel-strike and toe-off
% during walking using one IMU sensor.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all

%% start UDP communication
% Define computer-specific variables
matlabProcess = 'localhost';   portA = 41000;   % Modify these values to be those of your first computer.
labviewProcess = 'localhost';  portB = 42000;  % Modify these values to be those of your second computer.

% Create UDP Object
udpA = udp(ipB,portB,'LocalPort',portA);

% Connect to UDP Object
disp('Opening UDP port');
fopen(udpA);



%% definitions

comPort = '6';
captureDuration = 50;

shimmer = ShimmerHandleClass(comPort);                                     % Define shimmer as a ShimmerHandle Class instance with comPort1
SensorMacros = SetEnabledSensorsMacrosClass;                               % assign user friendly macros for setenabledsensors

DELAY_PERIOD = 0.01;                                                        % A delay period of time in seconds between data read operations
numSamples = 0;
isDataReady = 0;


histogram_value = 50;
analysis_mode = {'sensor_data'};

release_assistance = 0;
apply_assistance = 1;

%% Loading training data from walking activites (level-ground and ramp ascent)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

folder_mode = analysis_mode{1};
path = '.';
path = [path filesep folder_mode filesep];

% load state from expt
disp([path filesep 'multiple_expt.mat']);
load([path filesep 'multiple_expt.mat'], 'expt')
expt.rootpath = '';
expt.path = path;

% some variables
ncs = length(expt.trainingClasses);
nxydws = length(expt.trainingXs)*length(expt.trainingYs)*length(expt.trainingDs)*length(expt.trainingWs);
nwhisks = expt.trainingNwhisks;

% only test with last Nwhisks
Nwhisks = expt.testingNwhisks;

% parameters
state.cond = expt;
state.logth = nan;

% figure and text output
state.nofig = true;
state.notext = true;

disp(['Loading testing data...']);

% collect data together
for ic = 1:ncs    
    % filename    
    fname = ['multiple_data_' expt.testingClasses{ic} '_test'];

    % is data stored?
    expt.store = true;

    % load data
    if ~expt.store
    else
        % load data from store
        load([expt.path fname '_store.mat']);
        disp([expt.path fname '_store.mat']);

        % extract data
        for ixydw = 1:nxydws
            for iwhisk = 1:nwhisks
                ind = sub2ind([nxydws, nwhisks], ixydw, iwhisk);                    
                disp(['indexes: ixydw: ' num2str(ixydw) ', iwhisk: ' num2str(iwhisk) '; ic: ' num2str(ic) ', nxydws: ' num2str(nxydws) ', nwhisks: ' num2str(nwhisks) '; IND: ' num2str(ind)]);
                data{ic}(:, :, ind) = store{ind+1}{1};
            end                
        end            
    end
end

disp(['Loading training data...']);

% load classifier from pclass
run_classifier(expt, Nwhisks, '_train',histogram_value);
load([expt.path 'multiple_pclass_train.mat'], 'p', 'd');
state.classifier.p = p;
state.classifier.d = d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Seetings for classifier

output_all = {};
logths = log( [0.9] );
nths = length(logths);
ith = 1;
noiseRatio = 100;

activity_class = 1;

state.logth = logths(ith);

% initialize outputs
output.ic = [];
output.ixydw = [];
output.e_c = cell(ncs, nxydws);
output.e_xydw = cell(ncs, nxydws);
output.confusion_mat_class = zeros(ncs, ncs);
output.confusion_mat_subclass = zeros(nxydws, nxydws);
output.confusion_mat_subclass_no_transit = zeros(nxydws, nxydws);
output.confusion_mat_transition = cell(1, nxydws);
output.eventDetection = zeros(2,1); % row 1: timestep, row 2: event (1:HC, 2:TO)

for trans=1:nxydws
    output.confusion_mat_transition{1,trans} = zeros(ncs, ncs);
end


try
    figure(1);
    subplot(2,1,1);
    est_plot_data = zeros(1,ncs);
    hclass_data = bar(est_plot_data);
    axis([0.5 ncs+0.5 0 1]);
    title('State recognition');
    set(gca, 'xticklabel', {'release','assistance'});

    subplot(2,1,2);
    est_plot_beliefs = ones(1,ncs);
    est_plot_beliefs = est_plot_beliefs/sum(est_plot_beliefs);
    hclass_beliefs = bar(est_plot_beliefs);
    axis([0.5 ncs+0.5 0 1]);
    title('State recognition');
    set(gca, 'xticklabel', {'release','assistance'});


    %%
    if (shimmer.connect)                                                       % TRUE if the shimmer connects
        disp('Connect OK');

        % Define settings for shimmer
        %shimmer.setsamplingrate(100);                                         % Set the shimmer sampling rate to 51.2Hz
        shimmer.setsamplingrate(2500);                                         % Set the shimmer sampling rate to 51.2Hz
        shimmer.setinternalboard('9DOF');                                      % Set the shimmer internal daughter board to '9DOF'
        shimmer.disableallsensors;                                             % disable all sensors
        shimmer.setenabledsensors(SensorMacros.ACCEL,1,SensorMacros.MAG,0,...  % Enable the shimmer accelerometer, magnetometer, gyroscope and battery voltage monitor
        SensorMacros.GYRO,1,SensorMacros.BATT,0);    
        shimmer.setaccelrange(0);                                              % Set the accelerometer range to 0 (+/- 1.5g) for Shimmer2r
        shimmer.setbattlimitwarning(3.4);                                      % This will cause the Shimmer LED to turn yellow when the battery voltage drops below 3.4, note that it is only triggered when the battery voltage is being monitored, and after the getdata command is executed to retrieve the battery data 

        if (shimmer.start)                                                     % TRUE if the shimmer starts streaming
            disp('Start OK');

            tempPlotData = [];
            configureClassification = 1;

            elapsedTime = 0;                                                   % Reset to 0    
            tic;

            countEventDetection = 0;

            while ( elapsedTime < captureDuration )            
                pause(DELAY_PERIOD);                                           % Pause for this period of time on each iteration to allow data to arrive in the buffer

                newData = [];
                [newData,signalNameArray,signalFormatArray,signalUnitArray] = shimmer.getdata('c');   % Read the latest data from shimmer data buffer, signalFormatArray defines the format of the data and signalUnitArray the unit                        

                if ~isempty(newData)                                           % TRUE if new data has arrived                
                    if( configureClassification == 1 )
                        ic = activity_class;    % in this program we test one activity at the time
                        ixydw = randi(nxydws);  % for activity classification subclasses = 1;

                        % set state
                        state.ic = ic;
                        state.ixydw_init = ixydw;

                        % initialize position history
                        ixydw_history = [];

                        % start up machine
                        machine = controller(state);
                        result.ixydw = state.ixydw_init;
                        result.continue = 1;

                        % whisk range (last Nwhisks)
                        rwhisks = (nwhisks-Nwhisks+1) : nwhisks;

                        configureClassification = 0;                                
                    end

%                    tempPlotData =  [tempPlotData; newData];

                    if( result.continue == 1 )
                        
                        tempPlotData =  [tempPlotData; newData];    % save sensor data only when a decision has not been made

                        % available whisk?
                        if isempty(rwhisks)
                            null = 1;
                            break;
                        end

                        % choose whisk
                        iwhisk = rwhisks( randi(length(rwhisks)) );
                        ind = iwhisk + (result.ixydw-1)*nwhisks;

                        % position history
                        ixydw_history(end+1) = result.ixydw;

                        r_line = newData(:,7);    % r_line(X, Y) -> r_line(1, 2)

                        result = step(machine, awgn(r_line, noiseRatio, 'measured'),histogram_value);

                        norm_ic_beliefs = exp(result.ic_beliefs)/sum(exp(result.ic_beliefs));
                        set(hclass_beliefs,'YData',norm_ic_beliefs);

                        drawnow

                        countEventDetection = countEventDetection+length(r_line);
                    else
                        % terminate machine
                        machine.terminate();

                        est_plot_data = zeros(1,ncs);
                        est_plot_data(result.ic_est) = 1;                                        
                        set(hclass_data,'YData',est_plot_data);

                        %% Sent decision to LabviewProcess
                        if( result.ic_est == 1 )
                            fprintf(udpA,'%d', release_assistance);
                        elseif( result.ic_est == 2 )
                            fprintf(udpA,'%d', apply_assistance);
                        else
                            fprintf(udpA,'%d', release_assistance);
                        end
                        
                        drawnow

                        if( ic == 1 )
                            real_label = 'heel-contact';
                        elseif( ic == 2 )
                            real_label = 'toe-off';
                        end

                        output.eventDetection(1,end+1) = countEventDetection;
                        output.eventDetection(2,end) = result.ic_est;

                        if( result.ic_est == 1 )
                            est_label = 'heel-contact';
                        elseif( result.ic_est == 2 )
                            est_label = 'toe-off';
                        end

                        % process outputs
                        output.ic(end+1) = state.ic;
                        output.ixydw{end+1} = ixydw_history;
                        output.e_c{ic, ixydw}(end+1) = ic - result.ic_est;

                        output.e_xydw{ic, ixydw}(end+1) = result.ixydw - result.ixydw_est;
                        output.confusion_mat_subclass(ixydw, result.ixydw_est) = output.confusion_mat_subclass(ixydw, result.ixydw_est) + 1;

                        output.confusion_mat_class(ic, result.ic_est) = output.confusion_mat_class(ic, result.ic_est) + 1;
                        output.confusion_mat_transition{1,ixydw}(ic, result.ic_est) = output.confusion_mat_transition{1,ixydw}(ic, result.ic_est) + 1;

                        configureClassification = 1;
                    end
                end

                elapsedTime = elapsedTime + toc;                               % Stop timer and add to elapsed time
                tic;                                                           % Start timer                       
            end

            output.gait_data = tempPlotData;
            output_all{ith} = output;
            save(['output_all_plot_trial_2'],'output_all');

            isDataReady = 1;

            elapsedTime = elapsedTime + toc;                                   % Stop timer

            fclose(udpA);
            delete(udpA);
            clear ipA portA ipB portB udpA
            
            shimmer.stop;
            disp('Stop OK');        
        end

        shimmer.disconnect;
        disp('Disconnect OK');
    end
catch
    shimmer.stop;
    shimmer.disconnect;

    fclose(udpA);
    delete(udpA);
    clear ipA portA ipB portB udpA
end