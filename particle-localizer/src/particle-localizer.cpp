//============================================================================
// Name        : localizer.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <signal.h>
#include <stdint.h>

#include <ypspur.h>

#include <ssm.h>
#include <ssm.hpp>
#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>
#include <ssmtype/ypspur-ad.h>

#include "ssm-particles.hpp"

#include "gnd-util.h"
#include "gnd-random.hpp"
#include "gnd-shutoff.hpp"

#include "particle-localizer-opt.hpp"
#include "particle-localizer-cui.hpp"

const char _Debug_Log_[] = "debug.log";


int read_kinematics_config(double* radius_r, double* radius_l, double* tread,
		double* gear, double* count_rev, const char* fname);

/*
 * @brief main
 */
int main(int argc, char* argv[], char *envp[]) {
	SSMApi<PWSMotor>  mtr;
	SSMApi<YP_ad> ad;
	SSMApi<Spur_Odometry> ssm_position;
	SSMParticles ssm_particle;
	SSMParticleEvaluation ssm_estimation;

	gnd::cui_reader gcui;
	Localizer::proc_configuration pconf;
	Localizer::proc_option opt(&pconf);
	gnd::matrix::fixed<1, PARTICLE_DIM> myu_ini;

	double wheel_mean = 0,
			wheel_ratio = 0,
			tread_ratio = 0;
	double gear = 0,
			count_rev = 0,
			revl_ratio = 0;
	Spur_Odometry_Property odm_prop;




	{ // ---> Initialize
		uint32_t phase = 1;
		double inittime = -1;
		// todo make parameter file
		gnd::matrix::set_zero(&myu_ini);

		// get option
		if( !opt.get_option(argc, argv) ){
			return 0;
		}

		{ // ---> allocate SIGINT to shut-off
			proc_shutoff_clear();
			proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off


		::fprintf(stderr, "========== Initialize ==========\n");
		::fprintf(stderr, " %d. allocate signal \"\x1b[4mSIGINT\x1b[0m\" to shut-off\n", phase++);
		::fprintf(stderr, " %d. get kinematics parameter\n", phase++);
		if(opt.op_mode.start_at)
			::fprintf(stderr, " %d. load start way-point\n", phase++);
		::fprintf(stderr, " %d. initialize \x1b[4mssm\x1b[0m\n", phase++);
		::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_ADJUST );
		::fprintf(stderr, " %d. create initial particles and wirte initial ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++ , SNAME_ADJUST);
		::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_PARTICLES );
		::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_PARTICLES_EVALUATION );
		::fprintf(stderr, " %d. open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_PWS_MOTOR );
		if( pconf.gyro.value)
			::fprintf(stderr, " %d. open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SNAME_YPSPUR_AD );
		::fprintf(stderr, "\n");


		if( !is_proc_shutoff() ) { // ---> set initial kinematics
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => get initial kinematics parameter\n");

			{ // ---> set-zero kinematics property
				odm_prop.radius_l = pconf.k_lwheel.value;
				odm_prop.radius_r = pconf.k_rwheel.value;
				odm_prop.tread = pconf.k_tread.value;
				gear = pconf.k_gear.value;
				count_rev = pconf.k_encoder.value;
			} // <--- set-zero kinematics property

			// read kinematics configure file
			if(*pconf.kfile.value != '\0'){
				::fprintf(stderr, "    load kinematics parameter file \"\x1b[4m%s\x1b[0m\"\n", pconf.kfile.value);
				read_kinematics_config(&odm_prop.radius_r, &odm_prop.radius_l, &odm_prop.tread,
						&gear, &count_rev, pconf.kfile.value);
			}

			// check kinematic parameter
			if( odm_prop.radius_r > 0 && odm_prop.radius_l > 0 && odm_prop.tread > 0 && gear > 0 && count_rev > 0){
			}
			else {
				// initialize for using ypspur-coordinator
				::fprintf(stderr, "    load lacking parameter from \x1b[4mypsypur-coordinator\x1b[0m\n");
				if( Spur_init() < 0){
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to connect ypspur-coordinator.\n");
					proc_shutoff();
				}

				// get kinmeatics from ypspur-coordinator
				if( !is_proc_shutoff() && YP_get_parameter( YP_PARAM_RADIUS_R, &odm_prop.radius_r ) < 0 ){
					proc_shutoff();
				}
				if( !is_proc_shutoff() && YP_get_parameter( YP_PARAM_RADIUS_L, &odm_prop.radius_l ) < 0 ){
					proc_shutoff();
				}
				if( !is_proc_shutoff() && YP_get_parameter( YP_PARAM_TREAD, &odm_prop.tread ) < 0 ){
					proc_shutoff();
				}
				if( !is_proc_shutoff() && YP_get_parameter(YP_PARAM_COUNT_REV, &count_rev) < 0 ){
					proc_shutoff();
				}
				if( !is_proc_shutoff() && YP_get_parameter(YP_PARAM_GEAR, &gear) < 0 ){
					proc_shutoff();
				}
				::fprintf(stdout, "%d %d %d\n", YP_PARAM_NUM, YP_PARAM_RADIUS_R, YP_PARAM_RADIUS_L);
				::fprintf(stdout, "%lf %lf %lf %lf %lf\n",
						odm_prop.radius_r, odm_prop.radius_l, odm_prop.tread, count_rev, gear);
			}
			// <--- get parameter from ypspur-coordinater

			// check kinematic parameter
			if( odm_prop.radius_r <= 0 || odm_prop.radius_l <= 0 || odm_prop.tread <= 0 || gear <= 0 || count_rev <= 0){
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m : Incomplete loading kinematics parameter.\n");
				proc_shutoff();
			}

			// set kinematics
			if( !is_proc_shutoff() ){
				// wheel mean
				wheel_mean = (odm_prop.radius_l + odm_prop.radius_r) / 2.0;
				wheel_ratio = odm_prop.radius_l / wheel_mean;
				tread_ratio = odm_prop.tread / wheel_mean;

				if( !pconf.gyro.value ){

					myu_ini[0][PARTICLE_WHEEL_MEAN] = wheel_mean;
					myu_ini[0][PARTICLE_WHEEL_RATIO] = wheel_ratio;
					myu_ini[0][PARTICLE_TREAD_RATIO] = tread_ratio;
				}
				else {
					myu_ini[0][PARTICLE_WHEEL_MEAN] = wheel_mean;
					myu_ini[0][PARTICLE_GYRO_BIAS] = pconf.gyro_bias.value;
					myu_ini[0][PARTICLE_GYRO_SF] = pconf.gyro_sf.value;
				}
				ssm_particle.data.init_kinematics(count_rev, gear);

				revl_ratio = count_rev * gear;
			}

		} // <--- set initial kinematics



		 // ---> get configure
		if( !is_proc_shutoff() ){

			Localizer::proc_conf_sampling_ratio_normalize(&pconf);
			Localizer::proc_conf_set_kinematics_parameter(&pconf, wheel_mean, wheel_ratio, tread_ratio);

			{ // ---> initlaize min cov
				gnd::matrix::fixed<3, 3> min_cov;

				gnd::matrix::set_zero(&min_cov);

				gnd::matrix::set(&min_cov, 0, 0, 1.0e-20);
				gnd::matrix::set(&min_cov, 1, 1, 1.0e-20);
				gnd::matrix::set(&min_cov, 2, 2, 1.0e-20);

				gnd::matrix::add(&pconf.randerr_covar,			&min_cov,		&pconf.randerr_covar);
				gnd::matrix::add(&pconf.syserr_cover,				&min_cov,		&pconf.syserr_cover);
				gnd::matrix::add(&pconf.randerr_covar_offset,	&min_cov,		&pconf.randerr_covar_offset);
				gnd::matrix::add(&pconf.syserr_cover,				&min_cov,		&pconf.syserr_cover);
				gnd::matrix::add(&pconf.syserr2_covar, 			&min_cov, 		&pconf.syserr2_covar);

			} // <--- initlaize min cov
		} // <--- get configure


		// ---> set initialize position covariance matrix
		if( !is_proc_shutoff() ){
			ssm_estimation.property.n = pconf.particles.value + pconf.random_sampling.value;
			ssm_estimation.data.n = pconf.particles.value + pconf.random_sampling.value;
			ssm_estimation.data.value = new double [ssm_estimation.data.n];
		} // <--- set initialize position covariance matrix


		// ---> initialize ssm
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => initialize SSM\n");
			if( !initSSM() ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: SSM is not available.\n");
				proc_shutoff();
			}
			else {
				::fprintf(stderr, "   ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- initialize ssm


		// ---> ssm open
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_ADJUST );
			if( !ssm_position.create( SNAME_ADJUST, 0, 1, 0.005) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", SNAME_ADJUST );
				proc_shutoff();
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm open



		if( !is_proc_shutoff() ) { // ---> write initial position
			::fprintf(stderr, " => create initial particles and write into ssm\n" );

			// create initialize position
			ssm_particle.data.init_particle(&myu_ini,
					&pconf.poserr_cover_ini, &pconf.syserr_cover_ini, pconf.particles.value + pconf.random_sampling.value);

			// set property
			ssm_particle.property.n = pconf.particles.value + pconf.random_sampling.value;

			// set position
			ssm_position.data.x = ssm_particle.data.pos.odo.x;
			ssm_position.data.y = ssm_particle.data.pos.odo.y;
			ssm_position.data.theta = ssm_particle.data.pos.odo.theta;
			ssm_position.data.v = 0;
			ssm_position.data.w = 0;

			// show
			::fprintf(stderr, "  x  - %lf\n", ssm_position.data.x);
			::fprintf(stderr, "  y  - %lf\n", ssm_position.data.y);
			::fprintf(stderr, "  th - %lf\n", ssm_position.data.theta);

			// ---> write position to ssm
			if( inittime > 0){
				if( !ssm_position.write( inittime ) && !ssm_position.write( ) ){
					::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to write \"\x1b[4m%s\x1b[0m\"\n", SNAME_ADJUST );
					proc_shutoff();
				}
				else {
					::fprintf(stderr, "   ... \x1b[1mOK\x1b[0m\n");
				}
			}
			else if( !is_proc_shutoff() ) {
				if( !ssm_position.write( ) ){
					::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to write \"\x1b[4m%s\x1b[0m\"\n", SNAME_ADJUST );
					proc_shutoff();
				}
				else {
					::fprintf(stderr, "   ... \x1b[1mOK\x1b[0m\n");
				}
			} // <--- write position to ssm
		} // <--- write initial position


		// ---> ssm particles open
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_PARTICLES );
			if( !ssm_particle.create( SNAME_PARTICLES, 0, 5, 0.005 ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", SNAME_PARTICLES );
				proc_shutoff();
			}
			else {
				ssm_particle.setProperty();
				ssm_particle.write();
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm particles open


		// ---> ssm perticles evaluation open
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_PARTICLES_EVALUATION );
			if( !ssm_estimation.create( SNAME_PARTICLES_EVALUATION, 0, 5, 0.1 ) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", SNAME_PARTICLES_EVALUATION );
				proc_shutoff();
			}
			else {
				ssm_estimation.setProperty();
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		}// <--- ssm perticles evaluation open

		// ---> ssm pws motor open
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_PWS_MOTOR );
			if( !is_proc_shutoff() && !mtr.openWait( SNAME_PWS_MOTOR, 0, 0.0) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", SNAME_PWS_MOTOR );
				proc_shutoff();
			}
			else {
				// next read new data
				mtr.readLast();
				// blocking
				mtr.setBlocking(true);
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm pws motor open


		// ---> ssm ad open
		if( pconf.gyro.value && !is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open ssm-data \"\x1b[4m%s\x1b[0m\"\n", SNAME_YPSPUR_AD );
			if( !is_proc_shutoff() && !ad.openWait( SNAME_YPSPUR_AD, 0, 0.0) ){
				::fprintf(stderr, "  \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to create \"\x1b[4m%s\x1b[0m\"\n", SNAME_YPSPUR_AD );
				proc_shutoff();
			}
			else {
				// next read new data
				ad.readLast();
				// blocking
				ad.setBlocking(false);
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- ssm ad open

		// initialize cui
		gcui.set_command(cui_cmd, sizeof(cui_cmd) / sizeof(cui_cmd[0]));

	} // <--- Initialize







	if( !is_proc_shutoff() ){ // ---> operation
		int rsmpl_cnt = 0,
			reject_cnt = 0;
		ssmTimeT rsmp_time = 0;
		ssmTimeT prev_time = mtr.time;
		bool show_st = true;

		double eval_ave_slow = 0;
		static const double eval_alpha_slow = 1.0 / 180;
		double eval_ave_fast = 0;
		static const double eval_alpha_fast = 1.0 / 60;
		uint32_t nparticle_remain = pconf.particles.value * pconf.resmp_rate_remain.value;
		uint32_t nparticle_pos = pconf.particles.value * pconf.resmp_rate_randerr.value;
		uint32_t nparticle_knm = pconf.particles.value * pconf.resmp_rate_syserr.value;
		uint32_t nparticle_wknm = pconf.particles.value * pconf.resmp_rate_syserr2.value;
		uint32_t nparticle_knm_reset = pconf.particles.value - nparticle_remain - nparticle_pos
				- nparticle_knm - nparticle_wknm;

		int enc_cnt_pos = 0;
		int enc_cnt_knm = 0;
		int enc_cnt_wknm = 0;
		FILE *dbgfp = 0;
		double cuito = 0;;

		{ // ---> show
			::fprintf(stderr, "\n\n");
			::fprintf(stderr, "========== Oeration ==========\n");
			::fprintf(stderr, " => start\n");
			::fprintf(stderr, " \x1b[1m%s\x1b[0m > ", Localizer::particle_filter);
		} // <--- show

		// ---> debug log open
		if(opt.op_mode.debug){
			::fprintf(stderr, " => debug-log mode\n");
			if( !(dbgfp = fopen(_Debug_Log_, "w")) ) {
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"", _Debug_Log_);
				opt.op_mode.debug = false;
			}
			else {
				fprintf(dbgfp, "# 1.[time] 2.[x] 3.[y] 4.[theta] 5.[v] 6.[w] 7.[wh mean] 8.[wh ratio] 9.[trd ratio] 10.[pwm1] 11.[pwm2] 12.[counter1] 13.[counter2]\n");
			}
		} // <--- debug log open


		while( !is_proc_shutoff() ){

			{ // ---> cui
				int cuival = 0;
				char cuiarg[512];

				::memset(cuiarg, 0, sizeof(cuiarg));
				if( gcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito) > 0 ){
					if( show_st ){
						// if show status mode, quit show status mode
						show_st = false;
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					}
					else {
						switch(cuival) {
						// exit
						case 'e': proc_shutoff(); break;
						// help
						case 'h': gcui.show(stderr, "   "); break;
						// debug log-mode
						case 'd': {
							::fprintf(stderr, "   => debug-log mode\n");
							if( ::strncmp("on", cuiarg, 2) == 0){
								bool flg = dbgfp ? true : false;

								if( !dbgfp && !(dbgfp = fopen(_Debug_Log_, "w")) ) {
									::fprintf(stderr, "    ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"", _Debug_Log_);
									opt.op_mode.debug = false;
								}
								else if( flg ){
									fprintf(dbgfp, "# 1.[time] 2.[x] 3.[y] 4.[theta] 5.[v] 6.[w] 7.[wh mean] 8.[wh ratio] 9.[trd ratio] 10.[pwm1] 11.[pwm2] 12.[counter1] 13.[counter2]\n");
									opt.op_mode.debug = true;
									::fprintf(stderr, "    ... \x1b[1mon\x1b[0m\n");
								}
								else {
									opt.op_mode.debug = true;
									::fprintf(stderr, "    ... \x1b[1mon\x1b[0m\n");
								}
							}
							else if( ::strncmp("off", cuiarg, 3) == 0){
								opt.op_mode.debug = false;
								::fprintf(stderr, "    ... \x1b[1moff\x1b[0m\n");
							}
							else {
								::fprintf(stderr, "   ... %s\n", opt.op_mode.debug ? "on": "off");
								::fprintf(stderr, "   if you want to change mode, input \"on/off\"\n");
							}
						}
						break;
						// show status
						case 's': show_st = true;	break;
						// stand-by mode
						case 'B': cuito = -1;		break;
						// start
						case 'o': cuito = 0;		break;

						case '\0':
							break;
						default:
							::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid command\n");
							::fprintf(stderr, "       Please input \x1b[4mhelp\x1b[0m/\x1b[4mh\x1b[0m to show command\n");
							break;

						}
					}
					::fprintf(stderr, "  \x1b[33m\x1b[1m%s\x1b[0m\x1b[39m > ", Localizer::particle_filter);
					gcui.poll(&cuival, cuiarg, sizeof( cuiarg ), 0);
				}
			}// ---> cui


			if( show_st ){ // ---> show status
				struct timespec cur;
				static struct timespec next;
				clock_gettime(CLOCK_REALTIME, &cur);

				if( cur.tv_sec > next.tv_sec ||
						( cur.tv_sec == next.tv_sec && cur.tv_nsec > next.tv_nsec )){

					::fprintf(stderr, "\x1b[0;0H\x1b[2J");	// display clear
					::fprintf(stderr, "-------------------- \x1b[33m\x1b[1m%s\x1b[0m\x1b[39m --------------------\n", Localizer::particle_filter);
					::fprintf(stderr, "    particle : %ld\n", ssm_particle.data.size() );
					::fprintf(stderr, "             : r %d, p %d, k %d, wk %d rk %d\n", nparticle_remain, nparticle_pos, nparticle_knm, nparticle_wknm, nparticle_knm_reset);
					::fprintf(stderr, "    resample : %d,   reject %d\n", rsmpl_cnt, reject_cnt );
					::fprintf(stderr, "    position : %.02lf %.02lf %.01lf\n",  ssm_particle.data.pos.odo.x,  ssm_particle.data.pos.odo.y,  gnd_ang2deg(ssm_particle.data.pos.odo.theta) );
					::fprintf(stderr, "    velocity : v %.02lf  w %.03lf\n", ssm_position.data.v, gnd_ang2deg(ssm_position.data.w) );
					::fprintf(stderr, "  kinematics : %.05lf %.05lf %.05lf\n",
							ssm_particle.data.pos.prop.wheel_odm.wheel_mean, ssm_particle.data.pos.prop.wheel_odm.wheel_ratio,
							pconf.gyro.value ? 1 / gnd_rad2deg( ssm_particle.data.pos.prop.wheel_odm.tread_ratio ) : ssm_particle.data.pos.prop.wheel_odm.tread_ratio );
					::fprintf(stderr, " counter rot : left %s, right %s\n", pconf.k_lwheel_crot.value ? "on" : "off", pconf.k_rwheel_crot.value ?  "on" : "off" );
					::fprintf(stderr, "    odometry : %s-odometry\n", pconf.gyro.value ? "gyro" : "wheel" );

					::fprintf(stderr, "\n");
					::fprintf(stderr, " Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
					next = cur;
					next.tv_sec++;
				}
			} // <--- show status



			// stand-by mode
			if( cuito < 0)	continue;


			// read ssm motor
			if( mtr.readNext() ){
				int cnt1 = (pconf.k_lwheel_crot.value ? -1 : 1) * mtr.data.counter1;
				int cnt2 = (pconf.k_rwheel_crot.value ? -1 : 1) * mtr.data.counter2;
				// compute pertilecs motion
				if( !pconf.gyro.value ) {
					ssm_particle.data.odometry_motion(cnt1, cnt2);
				}
				else {
					if( !ad.readTime(mtr.time) ) continue;
					ssm_particle.data.gyro_odometry_motion(cnt1, cnt2, ad.data.ad[0] * pconf.gyro_vol.value / (1 << pconf.gyro_bits.value), mtr.time - prev_time);
				}

				ssm_particle.write( mtr.time );

				// set position
				ssm_position.data.x = ssm_particle.data.pos.odo.x;
				ssm_position.data.y = ssm_particle.data.pos.odo.y;
				ssm_position.data.theta = ssm_particle.data.pos.odo.theta;


				{ // ---> compute velocity and angular velocity
					double wr = ( 2.0 * M_PI * ( (double) cnt1 ) ) / ( revl_ratio ),
							wl = ( 2.0 * M_PI * ( (double) cnt2 ) ) / ( revl_ratio );
					double wr2 = wr * ( 2.0 - ssm_particle.data.pos.prop.wheel_odm.wheel_ratio );
					double wl2 = wl * ssm_particle.data.pos.prop.wheel_odm.wheel_ratio;
					// robot translation quantity
					double tq = ( ssm_particle.data.pos.prop.wheel_odm.wheel_mean * ( wr2 + wl2 ) ) / 2.0;
					// robot rotation quantity
					double rq = ( wr2 - wl2 ) / ssm_particle.data.pos.prop.wheel_odm.tread_ratio ;

					ssm_position.data.v = tq / (mtr.time - prev_time);
					if( !pconf.gyro.value ){
						ssm_position.data.w = rq / (mtr.time - prev_time);
					}
					else {
						ssm_position.data.w = -(ad.data.ad[0] * pconf.gyro_vol.value / (1 << pconf.gyro_bits.value)
								- ssm_particle.data.pos.prop.gyro_odm.bias) * ssm_particle.data.pos.prop.gyro_odm.sf;
					}
				} // <--- compute velocity and angular velocity


				// write position to ssm
				ssm_position.write( mtr.time );
				prev_time = mtr.time;


				// log file out
				if( opt.op_mode.debug ){
					if( dbgfp ) {
						fprintf(dbgfp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf %lf %lf\n",
								mtr.time,
								ssm_particle.data.pos.odo.x, ssm_particle.data.pos.odo.y, ssm_particle.data.pos.odo.theta,
								ssm_position.data.v, ssm_position.data.w,
								ssm_particle.data.pos.prop.wheel_odm.wheel_mean,
								ssm_particle.data.pos.prop.wheel_odm.wheel_ratio,
								ssm_particle.data.pos.prop.wheel_odm.tread_ratio,
								mtr.data.pwm1, mtr.data.pwm2,
								mtr.data.counter1, mtr.data.counter2,
								eval_ave_slow, eval_ave_fast,
								eval_ave_slow == 0 ? 0 : 1.0 - eval_ave_fast / eval_ave_slow );
					}
				}

				// increment encoder count
				enc_cnt_pos += abs(cnt1) + abs(cnt2);
				enc_cnt_knm += abs(cnt1) + abs(cnt2);
				enc_cnt_wknm += abs(cnt1) + abs(cnt2);
			}

			// ---> resampling
			if( ssm_estimation.readNext() ){
				int ret;
				size_t remain = nparticle_remain;
				size_t rnoise = nparticle_pos;			// local sampling
				size_t wknm = nparticle_wknm;			// wide kinematics sampling
				size_t lknm = nparticle_knm;			// local kinematics sampling
				size_t knm_reset = nparticle_knm_reset;	// kinematics reset
				double eval_ave = 0;

				// count resampling
				rsmpl_cnt++;

				// ---> check time
				if( ssm_estimation.time < rsmp_time ){
					reject_cnt++;
					continue;
				} // <--- check time

				if(enc_cnt_knm < 0.25 * count_rev * gear){
					remain += lknm;
					lknm = 0;
				}
				else {
					enc_cnt_knm = 0;
				}


				if(enc_cnt_wknm < 10 * count_rev * gear){
					remain += wknm;
					wknm = 0;
				}
				else {
					enc_cnt_wknm = 0;
				}


				// select remaining particle
				ret = ssm_particle.data.resampling_remain(ssm_estimation.data.value, remain, &eval_ave);


				{ // ---> deternimation wide sampling num
					eval_ave_fast += eval_alpha_fast * (eval_ave - eval_ave_fast);
					eval_ave_slow += eval_alpha_slow * (eval_ave - eval_ave_slow);
				} // <--- deternimation wide sampling num


				if(opt.op_mode.debug) { // ---> open log file
				} // <--- open log file

				// ---> resampling
				if(ret >= 0){
					{ // ---> resampling position change particle
						gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM> covp;

						gnd::matrix::scalar_prod( &pconf.randerr_covar, enc_cnt_pos / (count_rev * gear), &covp);
						gnd::matrix::add( &covp, &pconf.randerr_covar_offset, &covp );
						ssm_particle.data.resampling_position(&covp, rnoise);
						enc_cnt_pos = 0;
					} // <--- resampling position change particle

					// resampling kinematics change particle (global)
					if(wknm > 0){
						ssm_particle.data.resampling_kinematics(&pconf.syserr2_covar, wknm);
					}
					// resampling kinematics change particle (local)
					if(lknm > 0){
						ssm_particle.data.resampling_kinematics(&pconf.syserr_cover, lknm);
					}
					// reset kinematics change particle
					if(knm_reset > 0){
						ssm_particle.data.resampling_kinematics_reset(&myu_ini, &pconf.reset_syserr_covar , knm_reset);
					}

					// save resampling time
					rsmp_time = prev_time + 1.0e-6;
					// write ssm
					ssm_particle.write( rsmp_time );


				} // <--- resampling


			} // <--- resampling

		}

		// close debug log
		if(opt.op_mode.debug && dbgfp){
			::fclose(dbgfp);
		}

	} // <--- operation


	{ // ---> finalize
		::endSSM();

		::fprintf(stderr, "Finish.\x1b[49m\n");
	} // <--- finalize

	return 0;
}


int read_kinematics_config(double* radius_r, double* radius_l, double* tread,
		double* gear, double* count_rev, const char* fname)
{
	FILE* fp;
	char buf[256];
	char s, e;
	char *p;
	int l;

	// ---> initialize
	if((fp = fopen(fname, "r")) == 0){
		return -1;
	}
	// <--- initialize

	while(1){
		::memset(buf, 0, sizeof(buf));
		if(::fgets(buf, sizeof(buf), fp) == 0)	break;

		// check comment out
		for(e = 0; *(buf+e) != '\0' && *(buf+e) != '#' && *(buf+e) != '\r' && *(buf+e) != '\n'; e++ );
		*(buf+e) = '\0';
		// delete space head
		for(s = 0; s < e && ::isspace(*(buf+s)); s++);

		p = 0;
		if(      (l = ::strlen("RADIUS_R"))	&& ::strncmp(buf + s, "RADIUS_R", l) == 0)	*radius_r = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("RADIUS_L"))	&& ::strncmp(buf + s, "RADIUS_L", l) == 0)	*radius_l = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("TREAD"))	&& ::strncmp(buf + s, "TREAD", l) == 0)		*tread = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("GEAR"))		&& ::strncmp(buf + s, "GEAR", l) == 0)		*gear = ::strtod( buf + s + l, &p);
		else if( (l = ::strlen("COUNT_REV"))&& ::strncmp(buf + s, "COUNT_REV", l) == 0)	*count_rev = ::strtod( buf + s + l, &p);

		if(!p){
			// syntax error
		}
	}

	{ // ---> finalize
		fclose(fp);
	} // <--- finalize

	return 0;
}





