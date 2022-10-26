function Algotithm_out=preprocessing(Algotithm_in)
% Each signal containing an AIS message is separated from the successive 
%message by 100 samples equal to 0.
% We extract the first and the last index of every AIS signal
list_signals=Algotithm_in.list_signals;
abs_signal=abs(list_signals);
%number of messages
nb_signals=0;
idx=2;
idx_ind=1;
list_index(1,1)=1;
for i=2:length(abs_signal)-100
    if((abs_signal(i-1)>0 && sum(abs_signal(i:i+99))==0))
        idx_ind=idx_ind+1;
        list_index(idx)=i;
        idx=idx+1;
        list_index(idx)=i+100;
        idx=idx+1;
        nb_signals=nb_signals+1;
    end
end
Algotithm_out=Algotithm_in;
Algotithm_out.list_index=list_index;
Algotithm_out.nb_signals=nb_signals;
end