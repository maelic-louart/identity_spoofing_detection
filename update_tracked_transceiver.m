function Algorithm_out=update_tracked_transceiver(Algorithm_in)
% Update ship list. We remove ship if they do not send messages from more
% than 420s. 
j=Algorithm_in.i;
Struct_list_transceiver=Algorithm_in.Struct_list_transceiver;
nb_transceiver=Struct_list_transceiver.nb_transceiver;
toa=Algorithm_in.Data.toa(j);
new_idx_new_transceiver=1;
new_nb_transceiver=nb_transceiver;
list_mmsi=[];
list_transceiver=Struct_list_transceiver.list_transceiver;
for i=1:nb_transceiver
    transceiver=Struct_list_transceiver.list_transceiver(i);
    toa_last_transceiver=transceiver.toa_last;
    delta_t=toa-toa_last_transceiver;
    if(delta_t<420)
        list_transceiver(new_idx_new_transceiver)=transceiver;
        list_mmsi(new_idx_new_transceiver)=transceiver.mmsi;
        new_idx_new_transceiver=new_idx_new_transceiver+1;
    else
        new_nb_transceiver=new_nb_transceiver-1;
    end
end
Struct_list_transceiver.nb_transceiver=new_nb_transceiver;
Struct_list_transceiver.idx_new_transceiver=new_idx_new_transceiver;
Struct_list_transceiver.list_mmsi=list_mmsi;
Struct_list_transceiver.list_transceiver=list_transceiver;
Algorithm_in.Struct_list_transceiver=Struct_list_transceiver;
Algorithm_out=Algorithm_in;
end