function Transceiver_update=initialisation_Kalman_matrix(Algorithm_in)

Struct_list_transceiver=Algorithm_in.Struct_list_transceiver;
% transceiver extraction
Transceiver_update=Struct_list_transceiver.list_transceiver(Algorithm_in.idx_transceiver);
j=Algorithm_in.i;
sigma_v_x=Algorithm_in.Kalman.sigma_v_x;
sigma_v_y=Algorithm_in.Kalman.sigma_v_y;
sigma_v_cfo=Algorithm_in.Kalman.sigma_v_cfo;

toa_last=Transceiver_update.toa_last;
last_x_mes=Transceiver_update.X_x_est(1);
last_y_mes=Transceiver_update.X_y_est(1);
last_cfo_mes=Transceiver_update.X_cfo_est(1);

toa=Algorithm_in.Data.toa(j);
x_mes=Algorithm_in.Data.x(j);
y_mes=Algorithm_in.Data.y(j);
cfo_mes=Algorithm_in.Data.cfo(j);
delta_t=toa-toa_last;
nb_r=2;
x_speed_mes=(x_mes-last_x_mes)/delta_t;
y_speed_mes=(y_mes-last_y_mes)/delta_t;
cfo_speed_mes=(cfo_mes-last_cfo_mes)/delta_t;
Transceiver_update.nb_r=nb_r;
Transceiver_update.toa_last=toa;
Transceiver_update.x_last=x_mes;
Transceiver_update.y_last=y_mes;
Transceiver_update.cfo_last=cfo_mes;
Transceiver_update.X_x_pred=[x_mes;x_speed_mes];
Transceiver_update.X_x_est=[x_mes;x_speed_mes];
Transceiver_update.X_y_pred=[y_mes;y_speed_mes];
Transceiver_update.X_y_est=[y_mes;y_speed_mes];
Transceiver_update.X_cfo_pred=[cfo_mes;cfo_speed_mes];
Transceiver_update.X_cfo_est=[cfo_mes;cfo_speed_mes];
Transceiver_update.P_x_pred=[sigma_v_x^2,sigma_v_x^2/delta_t;sigma_v_x^2/delta_t,2*sigma_v_x^2/delta_t^2];
Transceiver_update.P_x_est=[sigma_v_x^2,sigma_v_x^2/delta_t;sigma_v_x^2/delta_t,2*sigma_v_x^2/delta_t^2];
Transceiver_update.P_y_pred=[sigma_v_y^2,sigma_v_y^2/delta_t;sigma_v_y^2/delta_t,2*sigma_v_y^2/delta_t^2];
Transceiver_update.P_y_est=[sigma_v_y^2,sigma_v_y^2/delta_t;sigma_v_y^2/delta_t,2*sigma_v_y^2/delta_t^2];
Transceiver_update.P_cfo_pred=[sigma_v_cfo^2,sigma_v_cfo^2/delta_t;sigma_v_cfo^2/delta_t,2*sigma_v_cfo^2/delta_t^2];
Transceiver_update.P_cfo_est=[sigma_v_cfo^2,sigma_v_cfo^2/delta_t;sigma_v_cfo^2/delta_t,2*sigma_v_cfo^2/delta_t^2];
Transceiver_update.list_inno_x(nb_r)=0;
Transceiver_update.list_inno_y(nb_r)=0;
Transceiver_update.list_inno_cfo(nb_r)=0;
Transceiver_update.residu_cfo=0;
Transceiver_update.list_S_x(nb_r)=1;
Transceiver_update.list_S_y(nb_r)=1;
Transceiver_update.list_S_cfo(nb_r)=1;
end

