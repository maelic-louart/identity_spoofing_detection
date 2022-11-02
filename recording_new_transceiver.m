function Algorithm_out=recording_new_transceiver(Algorithm_in,fid3)
% extraction of the data contained in the message
i=Algorithm_in.i;
toa=Algorithm_in.Data.toa(i);
cfo=Algorithm_in.Data.cfo(i);
x=Algorithm_in.Data.x(i);
y=Algorithm_in.Data.y(i);
msgType=Algorithm_in.Data.msgType(i);
%if the msgType==0, it means that no message was decoded
if(msgType~=0)
    mmsi=Algorithm_in.Data.mmsi(i);
    % We extract the structure containing the list of transceivers
    Struct_list_transceiver=Algorithm_in.Struct_list_transceiver;
    % we extract the observation noise one cfo
    R_cfo=Algorithm_in.Kalman.R_cfo;
    q_cfo=Algorithm_in.Kalman.q_cfo;
    %transceiver initialisation
    Transceiver=struct('mmsi',mmsi,'nb_r',1,'nb_error_x',0,'nb_error_y',0,'nb_error_cfo',0,'toa_last',toa,'x_last',x,'y_last',y,'cfo_last',cfo,'X_y_est',[y;0],'X_y_pred',[y;0],'X_x_est',[x;0],'X_x_pred',[x;0],'X_cfo_est',[cfo;0],'X_cfo_pred',[cfo;0],'R_cfo',R_cfo,'q_cfo',q_cfo,'residu_cfo',0,'P_y_pred',zeros(2,2),'P_y_est',zeros(2,2),'P_x_pred',zeros(2,2),'P_x_est',zeros(2,2),'P_cfo_pred',zeros(2,2),'P_cfo_est',zeros(2,2),'list_inno_y',[0],'list_inno_x',[0],'list_inno_cfo',[0],'list_S_y',[1],'list_S_x',[1],'list_S_cfo',[1],'list_toa_mes',[toa]);    
    Struct_list_transceiver.list_mmsi(Struct_list_transceiver.idx_new_transceiver)=mmsi;
    Struct_list_transceiver.list_transceiver(Struct_list_transceiver.idx_new_transceiver)=Transceiver;
    % number of transceiver recorded
    Struct_list_transceiver.nb_transceiver=Struct_list_transceiver.nb_transceiver+1;
    % indice of the next transceiver to record
    Struct_list_transceiver.idx_new_transceiver=Struct_list_transceiver.idx_new_transceiver+1;
    Algorithm_in.idx_transceiver=find(Struct_list_transceiver.list_mmsi==Algorithm_in.Data.mmsi(i));
    Algorithm_out=Algorithm_in;
    Algorithm_out.Struct_list_transceiver=Struct_list_transceiver;
    disp(["Recording new transceiver, mmsi =",mmsi,"toa = ",toa]);
    fprintf(fid3,"Recording new transceiver,mmsi=%d , toa=%d \n",[mmsi;toa]);
else
    Algorithm_out=Algorithm_in;
end
end