function Algorithm_out=cfo_data_extraction(Algorithm_in,k)

% we extract the next signal from the list
Algorithm_in.i=k;
list_index=Algorithm_in.list_index;
signal=Algorithm_in.list_signals(list_index(2*k-1):list_index(2*k));
Algorithm_in.signal=signal;
%we extract the list of list_samp_nb
list_samp_nb=Algorithm_in.list_samp_nb;
% Checksum using comm.CRCGenerator
crcGen = comm.CRCGenerator('Polynomial','X^16 + X^12 + X^5 + 1',...
    'InitialConditions',1,'DirectMethod',true,'FinalXOR',1);
phaseCalc=dsp.PhaseExtractor;

%parameters of the signal sampling
Fe=192000;
Te=1/Fe;
fo1=25000;
fo2=-25000;
samplesPerSymbol=Fe/9600;
% parameters of the fft
f_prec=1;
Nfft=Fe/f_prec;
f=-Fe/2:Fe/Nfft:Fe/2-Fe/Nfft;

% characteristic of filtering
BT=.5;
pulseLength=3;
gx = gaussdesign(BT,pulseLength,samplesPerSymbol);
Id=ones(1,length(gx));
%training sequence
syncCalc = syncGen(samplesPerSymbol);
%instantaneous frequency training sequence
syncIdeal = [0;diff(unwrap(angle(syncCalc)))];

signal=Algorithm_in.signal;
% After the demodulation it remains a frequency component equal to +25KHz
% or -25KHz that correspond to the channel A or B. We remove this frequency
% component.
phase=step(phaseCalc,signal);
if(phase(end)-phase(1)<0)
    % the signal is shifted with fo1 which carrier freq. = 192.025MHz
    hc=comm.PhaseFrequencyOffset('FrequencyOffset',fo1,'SampleRate',Fe);
    list_shifted=hc(signal);
    rxf = filter(gx,1,list_shifted);
    rxAngles = step(phaseCalc,rxf);
else
    % the signal is shifted with fo2 which carrier freq. = 191.975MHz
    hc=comm.PhaseFrequencyOffset('FrequencyOffset',fo2,'SampleRate',Fe);
    list_shifted=hc(signal);
    rxf = filter(gx,1,list_shifted);
    rxAngles = step(phaseCalc,rxf);
end
diff_phase=diff(rxAngles);
% we remove too large phase changes due to noise not to bias the
% correlation compuation
idx=find(abs(diff_phase)>0.5);
if length(idx)>=1
    diff_phase(idx)=0;
end
%correlation computation to detect the beginning of the training sequence
syncCorr=zeros(length(diff_phase)-length(syncIdeal),1);
if (length(diff_phase) > samplesPerSymbol*50 + length(syncIdeal))
    for ii=1:(length(diff_phase)-length(syncIdeal))
        syncCorr(ii)=syncIdeal'*diff_phase(ii:ii+length(syncIdeal)-1);
    end
end
[~,idx_corr]=max(syncCorr);
% CFO computation
y=fftshift(fft(rxf(idx_corr:idx_corr+length(syncIdeal)-1).*conj(syncCalc),Nfft));
[~,index]=max(abs(y));
cfo=f(index);
%We remove the CFO from the signal
hc=comm.PhaseFrequencyOffset('FrequencyOffset',-cfo,'SampleRate',Fe);
rxf_c=hc(rxf);
diff_phase=diff(unwrap(angle(rxf_c)));

% message decoding
idx_syn=idx_corr+samplesPerSymbol/2;
abits=zeros(size(diff_phase(idx_syn:samplesPerSymbol:end)));
idx_high=find(diff_phase(idx_syn:samplesPerSymbol:end)>0);
abits(idx_high)=1;
% NRZI demodulation
last_bit=0;
abits_NRZI_decoded=[0];
for i=1:length(abits)
    tmp_bit=abits(i);
    if(tmp_bit~=last_bit)
        abits_NRZI_decoded(i)=0;
        last_bit=tmp_bit;
    else
        abits_NRZI_decoded(i)=1;
        last_bit=tmp_bit;
    end
end
abits_NRZI_decoded=abits_NRZI_decoded(:);

% Search the first 50 bits for the StartByte flag (0x7E)
sb=1;
if length(abits)>56
    for ii=2:50
        if (sum(abits_NRZI_decoded(ii:ii+6))==6 && abits_NRZI_decoded(ii-1)==0 && abits_NRZI_decoded(ii+6)==0 && sb==1)
            sb=ii+7;
        end
    end
end

% message type extraction
if length(abits_NRZI_decoded) >= sb+7
    msgType=(2.^(0:5)*abits_NRZI_decoded(sb+2:sb+7));
end

% unstuffing
ubits=aisUnstuff(abits_NRZI_decoded(sb:end));

if length(ubits)>=184
    checkSum = step(crcGen,ubits(1:168));
    % We verify the checksum
    if isequal(checkSum(169:184),ubits(169:184))
        switch msgType
            case 1
                % We flip the data
                flippedData=aisFlipBytes(ubits(1:184));
                % We extrat the data from the message
                cs=aisDecodeMsg1(flippedData);
                mmsi=cs(2);
                toa=list_samp_nb(k)*Te;
                x=cs(8);
                y=cs(9);
            case 2
                % We flip the data
                flippedData=aisFlipBytes(ubits(1:184));
                % We extrat the data from the message
                cs=aisDecodeMsg1(flippedData);
                mmsi=cs(2);
                toa=list_samp_nb(k)*Te;
                x=cs(8);
                y=cs(9);
            case 3
                % We flip the data
                flippedData=aisFlipBytes(ubits(1:184));
                % We extrat the data from the message
                cs=aisDecodeMsg1(flippedData);
                mmsi=cs(2);
                toa=list_samp_nb(k)*Te;
                x=cs(8);
                y=cs(9);
            otherwise
                x=0;
                y=0;
                toa=0;
                mmsi=0;
        end
        Algorithm_out=Algorithm_in;
        Algorithm_out.i=k;
        Algorithm_out.Data.x(k)=x;
        Algorithm_out.Data.y(k)=y;
        Algorithm_out.Data.toa(k)=toa;
        Algorithm_out.Data.mmsi(k)=mmsi;
        Algorithm_out.Data.cfo(k)=cfo;
        Algorithm_out.Data.msgType(k)=msgType;
    else
        Algorithm_out=Algorithm_in;
        Algorithm_out.i=k;
        Algorithm_out.Data.x(k)=0;
        Algorithm_out.Data.y(k)=0;
        Algorithm_out.Data.toa(k)=0;
        Algorithm_out.Data.mmsi(k)=0;
        Algorithm_out.Data.cfo(k)=0;
        Algorithm_out.Data.msgType(k)=0;
    end
    reset(crcGen);
else
    Algorithm_out=Algorithm_in;
    Algorithm_out.i=k;
    Algorithm_out.Data.x(k)=0;
    Algorithm_out.Data.y(k)=0;
    Algorithm_out.Data.toa(k)=0;
    Algorithm_out.Data.mmsi(k)=0;
    Algorithm_out.Data.cfo(k)=0;
    Algorithm_out.Data.msgType(k)=0;
    
end
end
