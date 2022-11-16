%% Autre Algorithm simplified
clear all;
close all;

% File that contains every alarm detected
filename_a_S='alarms_strategy.dat';
fid_S = fopen(filename_a_S,'w');

% File that contains every alarm of the Identification
filename_a_I='alarms_Identification.dat';
fid_I=fopen(filename_a_I,'w');

% File that contains every alarm of the Verification
filename_a_V='alarms_Verification.dat';
fid_V=fopen(filename_a_V,'w');

% Earth polar radius
Rn=6356752;
% Earth equatorial radius
Re=6378137;
% Kalman parameters
sigma_v_gps=5;
% latitude of the receiver that received the messages
y_re=48.282935;
%conversion m to degre
degtodist_x=((Re*pi/180)*cos(y_re*pi/180));
degtodist_y=(Rn*pi/180);
% longitude parameters
a_max=1; % acceleration maximal is 1nd.s^-2
sigma_w_x=sigma_v_gps/((Re*pi/180)*cos(y_re*pi/180));
%maximum reporting interval for
RI_max=10;
% maximum change during Delta_t_max with a_max=1kt.s^-1
v_max=10;
% Conversion from kt to m.s^-1
conv=0.51444;
q_x=(RI_max*conv/degtodist_x)^2/v_max;
% latitude parameters
sigma_w_y=sigma_v_gps/(Rn*pi/180);
q_y=(RI_max*conv/degtodist_y)^2/v_max;
%cfo parameters
Delta_t=1000;
Delta_cfo=20;
q_cfo=(Delta_cfo^2)*3/Delta_t^3;
sigma_w_cfo=7;
%forgetting factors
alpha_v=0.95;
alpha_w=0.55;

% transceiver structure
Transceiver=struct('mmsi',0,'nb_r',0,'nb_error_x',0,'nb_error_y',0,'nb_error_cfo',0,'toa_last',0,'x_last',0,'y_last',0,'cfo_last',0,'X_y_est',zeros(2,1),'X_y_pred',zeros(2,1),'X_x_est',zeros(2,1),'X_x_pred',zeros(2,1),'X_cfo_est',zeros(2,1),'X_cfo_pred',zeros(2,1),'R_cfo',sigma_w_cfo^2,'q_cfo',q_cfo,'residu_cfo',0,'P_y_pred',zeros(2,2),'P_y_est',zeros(2,2),'P_x_pred',zeros(2,2),'P_x_est',zeros(2,2),'P_cfo_pred',zeros(2,2),'P_cfo_est',zeros(2,2),'list_inno_y',[],'list_inno_x',[],'list_inno_cfo',[],'list_S_y',[],'list_S_x',[],'list_S_cfo',[],'list_toa_mes',zeros(1,1));
% Structure that contain every transceiver controlled by the final algorithm
Struct_list_transceiver=struct('list_mmsi',[],'list_transceiver',[Transceiver],'nb_transceiver',0,'idx_new_transceiver',0);
% Structure that contains the fix parameters of the Kalman filter
Kalman=struct('R_x',sigma_w_x^2,'R_y',sigma_w_y^2,'R_cfo',sigma_w_cfo^2,'sigma_w_x',sigma_w_x,'sigma_w_y',sigma_w_y,'sigma_w_cfo',sigma_w_cfo,'q_x',q_x,'q_y',q_y,'q_cfo',q_cfo,'alpha_v',alpha_v,'alpha_w',alpha_w,'Rn',Rn,'Re',Re);
% Structure that contains every data received on a message
Data=struct('toa',[],'mmsi',[],'x',[],'y',[],'cfo',[],'msgType',[]);
% Structure of the algorithm
Algorithm=struct('Struct_list_transceiver',Struct_list_transceiver,'Kalman',Kalman,'idx_transceiver',0,'Data',Data,'signal',[],'list_signals',[],'list_samp_nb',[],'list_index',[],'nb_signals',0,'i',0);


% Creation of Algorithm_AIS
Algorithm_AIS=Algorithm;

% index of the first transceiver checks by the final algorithm
Algorithm_AIS.Struct_list_transceiver.idx_new_transceiver=1;


% first recording
filename='2022-06-29/';
% second recording
% filename='2022-07-14/';

% list_signals.csv contained only signal with AIS messages, the extraction 
% from noise was done considering the power. This is done to reduce the size
% of the file containing signals.
Algorithm_AIS.list_signals=readmatrix(filename+"list_signals.csv",'Delimiter','\n');
%list_indices.csv serves to compute the time of arrival (toa) of the messages
Algorithm_AIS.list_samp_nb=readmatrix(filename+"list_samp_nb.csv",'Delimiter','\n');

% We extract the number of signals potentially containing an AIS message 
% and the index of the start and end of each of these signals. 
Algorithm_AIS=preprocessing(Algorithm_AIS);

% Loop on every message
for i=1:Algorithm_AIS.nb_signals
    
    %read data in signal
    Algorithm_AIS=cfo_data_extraction(Algorithm_AIS,i);
    
    % update algorithms        
    Algorithm_AIS=update_tracked_transceiver(Algorithm_AIS);
    
    % starting Rooter
    % Condition to know if the message comes from a new transceiver
    if sum(find(Algorithm_AIS.Struct_list_transceiver.list_mmsi==Algorithm_AIS.Data.mmsi(i)))>0
        % Find the index of the transceiver in the list of the transceivers recorded
        Algorithm_AIS.idx_transceiver=find(Algorithm_AIS.Struct_list_transceiver.list_mmsi==Algorithm_AIS.Data.mmsi(i));
        % Ending Rooter
        
        % STEADY STATE
        % Algorithm 1 is applied
        Algorithm_AIS=Verification(Algorithm_AIS,fid_V,fid_S);
        
    else
        %INITIALISATION
        
        % Booking process is applied
        Algorithm_AIS=Identification(Algorithm_AIS,fid_I,fid_S);
        
        %A new transceiver is added to the transceiver controlled by the final algorithm
        Algorithm_AIS=recording_new_transceiver(Algorithm_AIS,fid_S);
    end
end
fclose(fid_I);
fclose(fid_V);
fclose(fid_S);
