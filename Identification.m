function Identification(Algorithm_in,fid_I,fid_S)
j=Algorithm_in.i;
nb_transceiver=Algorithm_in.Struct_list_transceiver.nb_transceiver;
%     Data measured
toa_mes=Algorithm_in.Data.toa(j);
x_mes=Algorithm_in.Data.x(j);
y_mes=Algorithm_in.Data.y(j);
cfo_mes=Algorithm_in.Data.cfo(j);
mmsi_mes=Algorithm_in.Data.mmsi(j);
msgType=Algorithm_in.Data.msgType(j);
%if the msgType==0 it means that no message was decoded
if(msgType~=0)
    for i=1:nb_transceiver
        Transceiver=Algorithm_in.Struct_list_transceiver.list_transceiver(i);
        toa_last=Transceiver.toa_last;
        residu_cfo=Transceiver.residu_cfo;
        %     The last values estimated by the Kalman filter is considered
        delta_t=toa_mes-toa_last;
        %     measurement noise and model noise
        R_x=Algorithm_in.Kalman.R_x;
        q_x=Algorithm_in.Kalman.q_x;
        R_y=Algorithm_in.Kalman.R_y;
        q_y=Algorithm_in.Kalman.q_y;
        alpha_v=Algorithm_in.Kalman.alpha_v;
        R_cfo=Transceiver.R_cfo;
        q_cfo=Transceiver.q_cfo;
        
        %     The last values estimated by the Kalman filter is considered
        X_y_est=Transceiver.X_y_est;
        X_x_est=Transceiver.X_x_est;
        X_cfo_est=Transceiver.X_cfo_est;
        P_y_est=Transceiver.P_y_est;
        P_x_est=Transceiver.P_x_est;
        P_cfo_est=Transceiver.P_cfo_est;
        %   threshold fixed to delta t
        delta_t_th=80;
        
        %     Kalman matrices
        A=[1 delta_t ; 0 1 ];
        C=[1 , 0];
        
        %     K_p longitude
        X_x_pred=A*X_x_est;
        %the noise process model is correct for 50s
        if(delta_t>delta_t_th)
            A_th=[1 delta_t_th ; 0 1 ];
            A_r=[1 delta_t-delta_t_th ; 0 1 ];
            Q_x=[delta_t_th^3/3 , delta_t_th^2/2 ;delta_t_th^2/2,delta_t_th]*q_x;
            P_x_pred=A_r*(A_th*P_x_est*transpose(A_th)+Q_x)*transpose(A_r);
        else
            Q_x=[delta_t^3/3 , delta_t^2/2 ;delta_t^2/2,delta_t]*q_x;
            P_x_pred=A*P_x_est*transpose(A)+Q_x;
        end
        S_x=(R_x+C*P_x_pred*transpose(C));
        
        %     K_p latitude
        X_y_pred=A*X_y_est;
        if(delta_t>delta_t_th)
            A_th=[1 delta_t_th ; 0 1 ];
            A_r=[1 delta_t-delta_t_th ; 0 1 ];
            Q_y=[delta_t_th^3/3 , delta_t_th^2/2 ;delta_t_th^2/2,delta_t_th]*q_y;
            P_y_pred=A_r*(A_th*P_y_est*transpose(A_th)+Q_y)*transpose(A_r);
        else
            Q_y=[delta_t^3/3 , delta_t^2/2 ;delta_t^2/2,delta_t]*q_y;
            P_y_pred=A*P_y_est*transpose(A)+Q_y;
        end
        S_y=(R_y+C*P_y_pred*transpose(C));
        
        % K_p cfo
        X_cfo_pred=A*X_cfo_est;
        Q_cfo=[delta_t^3/3 , delta_t^2/2 ;delta_t^2/2,delta_t]*q_cfo;
        P_cfo_pred=A*P_cfo_est*transpose(A)+Q_cfo;
        R_cfo=alpha_v*R_cfo+(1-alpha_v)*(residu_cfo^2+C*P_cfo_pred*transpose(C));
        S_cfo=(R_cfo+C*P_cfo_pred*transpose(C));
        
        % chi2 test on longitude
        inno_x=x_mes-C*X_x_pred;
        chi2_x=inno_x^2/S_x;
        if(chi2_x<8.64)
            H0_x=1;
            fprintf(fid_I,"Alarm longitude \n");
        else
            H0_x=0;
        end
        
        % chi2 test on latitude
        inno_y=y_mes-C*X_y_pred;
        chi2_y=inno_y^2/S_y;
        if(chi2_y<8.64)
            H0_y=1;
            fprintf(fid_I,"Alarm latitude \n");
        else
            H0_y=0;
        end
        
        % chi2 test on cfo
        inno_cfo=cfo_mes-C*X_cfo_pred;
        if(2.94*sqrt(S_cfo)>200)
            t_S_cfo=(200/2.94)^2;
        else
            t_S_cfo=S_cfo;
        end
        chi2_cfo=inno_cfo^2/t_S_cfo;
        if(chi2_cfo<8.64)
            H0_cfo=1;
            fprintf(fid_I,"Alarm cfo \n");
        else
            H0_cfo=0;
        end
        % cfo ==1 means H_0 accepted
        if((H0_cfo==1&&H0_x==1)&&H0_y==1)
            disp(["Alarm: The transceiver with mmsi=",Transceiver.mmsi," has changed its mmsi with mmsi =",mmsi_mes," toa = ",toa_mes,"H0_cfo=",H0_cfo,"H0_x=",H0_x,"H0_y=",H0_y]);
            fprintf(fid_I,"Alarm: The transceiver with mmsi=%d has changed its mmsi with mmsi =%d, toa=%d, H0_cfo=%d, H0_x=%d, H0_y=%d \n",[Transceiver.mmsi;mmsi_mes;toa_mes;H0_cfo;H0_x;H0_y]);
            fprintf(fid_S,"Alarm: The transceiver with mmsi=%d has changed its mmsi with mmsi =%d, toa=%d, H0_cfo=%d, H0_x=%d, H0_y=%d \n",[Transceiver.mmsi;mmsi_mes;toa_mes;H0_cfo;H0_x;H0_y]);
        end
    end
end
