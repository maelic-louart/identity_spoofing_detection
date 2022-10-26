function message  = aisDecodeMsg1( bits )
%aisDecodeMsg1 reads in the bits from an AIS Msg Type 1,2 or 3 and outputs  
% the data fields for the message.  AIS Msg Types 1,2 or 3 are ship
% position reports.

% Copyright 2016, The MathWorks, Inc.


id=2.^(5:-1:0)*bits(1:6);
repeat_indicator=[2 1]*bits(7:8);
mmsi=2.^(29:-1:0)*bits(9:38);
navStat=2.^(3:-1:0)*bits(39:42);
rot=2.^(7:-1:0)*bits(43:50);
speed=2.^(9:-1:0)*bits(51:60)*.1;
posAcc=bits(61);
x=(-2^27*bits(62)+2.^(26:-1:0)*bits(63:89))/10000/60;
y=(-2^26*bits(90)+2.^(25:-1:0)*bits(91:116))/10000/60;
cog=2.^(11:-1:0)*bits(117:128)*.1;
true_heading=2.^(8:-1:0)*bits(129:137);
timestamp=2.^(5:-1:0)*bits(138:143);
rsvd=2.^(3:-1:0)*bits(144:147);
spare=bits(148);
raim=bits(149);
sync_state=[2 1]*bits(150:151);
% commState=2.^(18:-1:0)*bits(150:168);
if(id==1||id==2)
    slot_timeout=2.^(2:-1:0)*bits(152:154);
    slot_increment=0;
    number_of_slots=0;
    keep_flag=0;
    if(slot_timeout==3||slot_timeout==5||slot_timeout==7)
        received_station=2.^(13:-1:0)*bits(155:168);
        slot_number=0;
        slot_offset=0;
    elseif(slot_timeout==2||slot_timeout==4||slot_timeout==6)
        received_station=0;
        slot_number=2.^(13:-1:0)*bits(155:168);
        slot_offset=0;
    elseif(slot_timeout==1)
        received_station=0;
        slot_number=0;
        slot_offset=0;
    else
        received_station=0;
        slot_number=0;
        slot_offset=2.^(13:-1:0)*bits(155:168);
    end
elseif(id==3)
    slot_increment=2.^(12:-1:0)*bits(152:164);
    number_of_slots=2.^(2:-1:0)*bits(165:167);
    keep_flag=bits(168);
    slot_timeout=0;
    received_station=0;
    slot_number=0;
    slot_offset=0;
end
message=zeros(1,26);
message(2)=mmsi;
message(3)=id;
message(4)=navStat;
message(5)=slot_timeout;
message(6)=rot;
message(7)=speed;
message(8)=x;
message(9)=y;
message(10)=cog;
message(11)=true_heading;
message(12)=timestamp;
message(13)=slot_number;
message(14)=slot_offset;
message(15)=slot_increment;
message(16)=keep_flag;
message(17)=repeat_indicator;


end
 


