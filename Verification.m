function Algorithm_out=Verification(Algorithm_in,fid_V,fid_S)
if Algorithm_in.Struct_list_transceiver.list_transceiver(Algorithm_in.idx_transceiver).nb_r==1
    % Kalman filter initialisation
    Algorithm_in.Struct_list_transceiver.list_transceiver(Algorithm_in.idx_transceiver)=initialisation_Kalman_matrix(Algorithm_in);
    Algorithm_out=Algorithm_in;
else
    Algorithm_out=Algorithm_in;
    % We extract some data from the transceiver
    j=Algorithm_in.i;
    Struct_list_transceiver=Algorithm_in.Struct_list_transceiver;
    Transceiver=Struct_list_transceiver.list_transceiver(Algorithm_in.idx_transceiver);
    toa_last=Transceiver.toa_last;
    residu_cfo=Transceiver.residu_cfo;
    mmsi=Transceiver.mmsi;
    % we increment the number of message received by the transceiver
    nb_r=Transceiver.nb_r+1;
    % number of error encountered
    nb_error_x=Transceiver.nb_error_x;
    nb_error_y=Transceiver.nb_error_y;
    nb_error_cfo=Transceiver.nb_error_cfo;
    
    % Data measured
    toa_mes=Algorithm_in.Data.toa(j);
    x_mes=Algorithm_in.Data.x(j);
    y_mes=Algorithm_in.Data.y(j);
    cfo_mes=Algorithm_in.Data.cfo(j);
    
    % measurement noise and model noise
    R_x=Algorithm_in.Kalman.R_x;
    sigma_w_x=Algorithm_in.Kalman.sigma_w_x;
    R_y=Algorithm_in.Kalman.R_y;
    sigma_w_y=Algorithm_in.Kalman.sigma_w_y;
    R_cfo=Transceiver.R_cfo;
    sigma_w_cfo=Transceiver.sigma_w_cfo;
    alpha_v=Algorithm_in.Kalman.alpha_v;
    alpha_w=Algorithm_in.Kalman.alpha_w;
    
    % The last values estimated by the Kalman filter is considered
    delta_t=toa_mes-toa_last;
    X_x_est=Transceiver.X_x_est;
    P_x_est=Transceiver.P_x_est;
    X_y_est=Transceiver.X_y_est;
    P_y_est=Transceiver.P_y_est;
    X_cfo_est=Transceiver.X_cfo_est;
    P_cfo_est=Transceiver.P_cfo_est;
    %threshold fixed to delta t 
    delta_t_th=80;
    
    %     Kalman matrices
    A=[1 delta_t ; 0 1 ];
    C=[1 , 0];

    %K_p longitude
    X_x_pred=A*X_x_est;
    %the noise process model is correct for 50s
    if(delta_t>delta_t_th)
        A_th=[1 delta_t_th ; 0 1 ];
        A_r=[1 delta_t-delta_t_th ; 0 1 ];
        Q_x=[delta_t_th^4/4 , delta_t_th^3/2 ;delta_t_th^3/2,delta_t_th^2]*sigma_w_x^2;
        P_x_pred=A_r*(A_th*P_x_est*transpose(A_th)+Q_x)*transpose(A_r);
    else
        Q_x=[delta_t^4/4 , delta_t^3/2 ;delta_t^3/2,delta_t^2]*sigma_w_x^2;
        P_x_pred=A*P_x_est*transpose(A)+Q_x;
    end
    S_x=(R_x+C*P_x_pred*transpose(C));
    
    %     K_p latitude
    X_y_pred=A*X_y_est;
    %the noise process model is correct for 80s
    if(delta_t>delta_t_th)
        A_th=[1 delta_t_th; 0 1 ];
        A_r=[1 delta_t-delta_t_th ; 0 1 ];
        Q_y=[delta_t_th^4/4 , delta_t_th^3/2 ;delta_t_th^3/2,delta_t_th^2]*sigma_w_y^2;
        P_y_pred=A_r*(A_th*P_y_est*transpose(A_th)+Q_y)*transpose(A_r);
    else
        Q_y=[delta_t^4/4 , delta_t^3/2 ;delta_t^3/2,delta_t^2]*sigma_w_y^2;
        P_y_pred=A*P_y_est*transpose(A)+Q_y;
    end
    S_y=(R_y+C*P_y_pred*transpose(C));
    
    %     K_p cfo
    X_cfo_pred=A*X_cfo_est;
    Q_cfo=[delta_t^4/4 , delta_t^3/2 ;delta_t^3/2,delta_t^2]*sigma_w_cfo^2;
    P_cfo_pred=A*P_cfo_est*transpose(A)+Q_cfo;
    R_cfo=alpha_v*R_cfo+(1-alpha_v)*(residu_cfo^2+C*P_cfo_pred*transpose(C));
    S_cfo=(R_cfo+C*P_cfo_pred*transpose(C));
    
%     Controlled of the longitude consistency
    H_x=P_x_pred*transpose(C)*inv((R_x+C*P_x_pred*transpose(C)));
    inno_x=x_mes-C*X_x_pred;
    chi2_x=inno_x^2/S_x;
    if chi2_x<8.64
        X_x_est=X_x_pred+H_x*inno_x;
        P_x_est=(diag(ones(1,2))-H_x*C)*P_x_pred;
        nb_error_x=0;
        H0_x=1;
    else
        X_x_est=X_x_pred;
        P_x_est=P_x_pred;
        disp(["Alarm: The distance between longitude prediced in measured is to high, toa=",toa_mes,"mmsi =",mmsi,'x_mes',x_mes,'x_pred = ',X_x_pred(1),'inno_x = ',inno_x,'threshold = ',2.94*sqrt(S_x)]);
        fprintf(fid_V,"Alarm: The distance between longitude prediced in measured is to high toa=%f, mmsi=%d, x_mes=%d, x_pred=%d, inno_x=%f, threshold=%f \n",[toa_mes;mmsi;x_mes;X_x_pred(1);inno_x;2.94*sqrt(S_x)]);
        fprintf(fid_S,"Alarm: The distance between longitude prediced in measured is to high toa=%f, mmsi=%d, x_mes=%d, x_pred=%d, inno_x=%f, threshold=%f \n",[toa_mes;mmsi;x_mes;X_x_pred(1);inno_x;2.94*sqrt(S_x)]);
        nb_error_x=nb_error_x+1;
        H0_x=0;
    end

%     Controlled of the latitude consistency
    H_y=P_y_pred*transpose(C)*inv((R_y+C*P_y_pred*transpose(C)));
    inno_y=y_mes-C*X_y_pred;
    chi2_y=inno_y^2/S_y;
    if (chi2_y<8.64)
        X_y_est=X_y_pred+H_y*inno_y;
        P_y_est=(diag(ones(1,2))-H_y*C)*P_y_pred;
        nb_error_y=0;
        H0_y=1;
    else
        X_y_est=X_y_pred;
        P_y_est=P_y_pred;
        disp(["Alarm: The distance between latitude prediced in measured is to high, toa=",toa_mes,"mmsi =",mmsi,'y_mes',y_mes,'y_pred = ',X_y_pred(1),'inno_y = ',inno_y,'threshold = ',2.94*sqrt(S_y)]);
        fprintf(fid_V,"Alarm: The distance between latitude prediced in measured is to high, toa=%f ,mmsi=%d, y_mes=%d, y_pred=%d, inno_y=%f, threshold=%f \n",[toa_mes;mmsi;y_mes;X_y_pred(1);inno_y;2.94*sqrt(S_y)]);
        fprintf(fid_S,"Alarm: The distance between latitude prediced in measured is to high, toa=%f ,mmsi=%d, y_mes=%d, y_pred=%d, inno_y=%f, threshold=%f \n",[toa_mes;mmsi;y_mes;X_y_pred(1);inno_y;2.94*sqrt(S_y)]);
        nb_error_y=nb_error_y+1;
        H0_y=0;
    end
    
    %     Controlled of the cfo consistency
    H_cfo=P_cfo_pred*transpose(C)*inv((R_cfo+C*P_cfo_pred*transpose(C)));
    inno_cfo=cfo_mes-C*X_cfo_pred;
    if(2.94*sqrt(S_cfo)>200)
        t_S_cfo=(200/2.94)^2;
    else
        t_S_cfo=S_cfo;
    end
    chi2_cfo=inno_cfo^2/t_S_cfo;
    if (chi2_cfo<8.64)
        X_cfo_est=X_cfo_pred+H_cfo*inno_cfo;
        P_cfo_est=(diag(ones(1,2))-H_cfo*C)*P_cfo_pred;
        nb_error_cfo=0;
        H0_cfo=1;
    else
        disp(["Alarm: The distance between cfo prediced in measured is to high, toa=",toa_mes,"mmsi =",mmsi,'cfo_mes',cfo_mes,'cfo_pred = ',X_cfo_pred(1),'inno_cfo = ',inno_cfo,'threshold = ',2.94*sqrt(S_cfo)]);
        fprintf(fid_V,"Alarm: The distance between cfo prediced in measured is to high, toa=%f, mmsi=%d, cfo_mes=%d, cfo_pred=%d, inno_cfo=%f, threshold=%f, sqrt(R_cfo)=%f \n",[toa_mes;mmsi;cfo_mes;X_cfo_pred(1);inno_cfo;2.94*sqrt(S_cfo);sqrt(R_cfo)]);
        fprintf(fid_S,"Alarm: The distance between cfo prediced in measured is to high, toa=%f, mmsi=%d, cfo_mes=%d, cfo_pred=%d, inno_cfo=%f, threshold=%f \n",[toa_mes;mmsi;cfo_mes;X_cfo_pred(1);inno_cfo;2.94*sqrt(S_cfo)]);
        nb_error_cfo=nb_error_cfo+1;
        H0_cfo=0;
        X_cfo_est=X_cfo_pred;
        P_cfo_est=P_cfo_pred;
    end
    residu_cfo=cfo_mes-C*X_cfo_est;
    
     if(nb_error_x>=5)
        X_x_est=[x_mes;(x_mes-Transceiver.x_last)/delta_t];
        nb_error_x=0;
    end
    if(nb_error_y>=5)
        X_y_est=[y_mes;(y_mes-Transceiver.y_last)/delta_t];
        nb_error_y=0;
    end
    if(nb_error_cfo>=5)
        X_cfo_est=[cfo_mes;(cfo_mes-Transceiver.cfo_last)/delta_t];
        nb_error_cfo=0;
    end
   
    %H0 ==0 means H_0 rejected
    if ((H0_cfo==0 || H0_x==0) || H0_y==0)
        disp("Alarm: Verification is not correct");
        fprintf(fid_V,"Alarm: Verification is not correct\n");
        fprintf(fid_S,"Alarm: Verification is not correct\n");
    end
    
    % We record the number of errors encountered
    Transceiver.nb_error_x=nb_error_x;
    Transceiver.nb_error_y=nb_error_y;
    Transceiver.nb_error_cfo=nb_error_cfo;
    Transceiver.nb_r=nb_r;
    
    %  We record inno_x and S_x
    Transceiver.list_inno_x(nb_r)=inno_x;
    Transceiver.list_S_x(nb_r)=S_x;

    %  We record inno_y and S_y
    Transceiver.list_inno_y(nb_r)=inno_y;
    Transceiver.list_S_y(nb_r)=S_y;
    
    %  We record inno_cfo and S_cfo
    Transceiver.list_inno_cfo(nb_r)=inno_cfo;
    Transceiver.list_S_cfo(nb_r)=S_cfo;
    
    % we record the data predicted in Algorithm_out
    Transceiver.X_x_pred=X_x_pred;
    Transceiver.P_x_pred=P_x_pred;

    Transceiver.X_y_pred=X_y_pred;
    Transceiver.P_y_pred=P_y_pred;
    
    Transceiver.X_cfo_pred=X_cfo_pred;
    Transceiver.P_cfo_pred=P_cfo_pred;
    
    % we record the data estimated in Algorithm_out
    Transceiver.X_x_est=X_x_est;
    Transceiver.P_x_est=P_x_est;

    Transceiver.X_y_est=X_y_est;
    Transceiver.P_y_est=P_y_est;
    
    Transceiver.X_cfo_est=X_cfo_est;
    Transceiver.P_cfo_est=P_cfo_est;
    
    % We update the data to compute R_cfo
    Transceiver.residu_cfo=residu_cfo;
    Transceiver.R_cfo=R_cfo;
    
    % We update the data to compute Q_cfo
    new_Q_wfo=H_cfo*inno_cfo^2*H_cfo';
    sigma_w_cfo_square=alpha_w*sigma_w_cfo^2+(1-alpha_w)*new_Q_wfo(2,2)/delta_t^2;
    Transceiver.sigma_w_cfo=sqrt(sigma_w_cfo_square);
    
    % We record measurements
    Transceiver.toa_last=toa_mes;
    Transceiver.x_last=x_mes;
    Transceiver.y_last=y_mes;
    Transceiver.cfo_last=cfo_mes;
    
    Algorithm_out.Struct_list_transceiver.list_transceiver(Algorithm_out.idx_transceiver)=Transceiver;
    Algorithm_out.list_delta_t(j)=delta_t;
    
end

end













