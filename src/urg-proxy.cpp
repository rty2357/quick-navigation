//============================================================================
// Name        : urg-handler.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <sys/time.h>

#include <scip2awd.h>

#include <ssm.hpp>
#include <ssmtype/spur-odometry.h>
#include "ssm-laser.hpp"

#include "gnd-shutoff.hpp"
#include "gnd-timer.hpp"
#include "gnd-config-file.hpp"
#include "gnd-util.h"
#include "gnd-matrix-coordinate.hpp"
#include "gnd-matrix-base.hpp"

#include "urg-proxy.hpp"
#include "urg-proxy-opt.hpp"
#include "urg-proxy-device-conf.hpp"

struct Scanning {
	SSMScanPoint2D ssm;
	gnd::urg_proxy::ScanningProperty prop;
	S2Param_t param;
};
typedef struct Scanning Scanning;

/**
 * @brief callback function
 * @param *aScan Scan data
 * @param *aData User's data
 * @retval 1 success to analyze scan data.
 * @retval 0 failed to analyze scan data and stop thread of duall buffer.
 */
int callback( S2Scan_t *s, void *u)
{
	struct timeval tv;
	double *t = (double*)u;

	gettimeofday(&tv, 0);
	*t = gnd_timeval2time(&tv);

	return 1;
}


int main(int argc, char* argv[]) {
	SSMScanPoint2D							scan_ssm;	// laser scanner reading ssm
	gnd::urg_proxy::ScanningProperty		scan_prop;	// scan property
	gnd::urg_proxy::ssm_property			ssm_prop;	// laser scanner configuration
	gnd::urg_proxy::device_configuration	dev_conf;	// device configuration

	gnd::urg_proxy::TimeAdjust				tmadj;		// time adjust variables
	gnd::urg_proxy::TimeAdjustProperty		tmadj_prop;	// time adjust properties
	double									recv_time;

	S2Port* port = 0;
	S2Sdd_t buffer;
	speed_t bitrate = B0;
	S2Ver_t version;
	S2Param_t param;

	gnd::urg_proxy::proc_configuration 		proc_conf;	// process configuration parameter
	gnd::urg_proxy::options 				proc_opt(&proc_conf);	// option reader

	{ // ---> initialize
		int ret;
		gnd::conf::file_stream fst_conf;			// device configuration file

		// ---> read process options
		if( (ret = proc_opt.get_option(argc, argv)) != 0 ) {
			return ret;
		} // <--- read process options


		{ // ---> show task
			int phase = 1;
			::fprintf(stderr, "========== Initialize ==========\n");
			::fprintf(stderr, " %d. read device configuration file\n", phase++);
			::fprintf(stderr, " %d. open device port\n", phase++);
			::fprintf(stderr, " %d. initialize SSM\n", phase++);
			::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, SSM_NAME_SCAN_POINT_2D);
			::fprintf(stderr, "\n");
		} // <--- show task



		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
			::proc_shutoff_alloc_signal(SIGTERM);
		} // <--- allocate SIGINT to shut-off



		// ---> read device configuration
		if( !::is_proc_shutoff() && *proc_conf.dev_conf.value ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => read device configuration file\n");
			if( fst_conf.read(proc_conf.dev_conf.value) < 0 ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to read configuration file \"\x1b[4m%s\x1b[0m\"\n", proc_conf.dev_conf.value);
			}
			else {
				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- read device configuration



		// ---> open device port
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open device port \"%s\"\n", proc_conf.dev_port.value);

			// initialize
			::S2Sdd_Init( &buffer );
			::S2Sdd_setCallback(&buffer, callback, &recv_time);

			// port open
			if( !(port = ::Scip2_Open(proc_conf.dev_port.value, bitrate) ) ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: faile to open port \"\x1b[4m%s\x1b[0m\"\n", "/dev/ttyACM0");
			}
			else {
				gnd::conf::configuration *pconf;

				// get device version infomation
				::Scip2CMD_VV(port, &version);
				gnd::urg_proxy::show_version(stderr, &version);

				// get device parameter infomation
				::Scip2CMD_PP(port, &param);


				scan_prop.step.min = param.step_min;
				scan_prop.step.max = param.step_max;

				// ---> find device configuration
				if( ( pconf = fst_conf.child_find(version.serialno, 0) ) == 0) {
					::fprintf(stderr, "     \x1b[1mWarnning\x1b[0m:missing device configuration\n");
				}
				else {
					// get configuration parameter
					gnd::urg_proxy::dev_conf_get(pconf, &dev_conf);

					// ssm-id
					ssm_prop.id = dev_conf.id.value;
					::strcpy(ssm_prop.name, dev_conf.name.value);
					::fprintf(stderr, " ... ssm-id is %d\n", ssm_prop.id);

					{ // ---> coordinate matrix
						gnd::matrix::coordinate_converter(&ssm_prop.coord,
								dev_conf.position.value[0], dev_conf.position.value[1], dev_conf.position.value[2],
								dev_conf.orient.value[0], dev_conf.orient.value[1], dev_conf.orient.value[2],
								dev_conf.upside.value[0], dev_conf.upside.value[1], dev_conf.upside.value[2]);
						::fprintf(stderr, " ... coordinate matrix is following ...\n");
						gnd::matrix::show(stderr, &ssm_prop.coord, "%.03lf", "     ");
					} // <--- coordinate matrix

					// ---> time adjust
					tmadj_prop.min_poll = dev_conf.timeadjust.value[0] > dev_conf.timeadjust.value[1] ? 0: dev_conf.timeadjust.value[0];
					tmadj_prop.max_poll = dev_conf.timeadjust.value[1];
					if( tmadj_prop.min_poll > 0 ){
						struct timeval htime;
						unsigned long dtime;

						::fprintf(stderr, " ... time adjust initialize.\n");
						if( !::Scip2CMD_TM_GetSyncTime(port, &dtime, &htime) ) {
							::proc_shutoff();
						}
						else {
							gnd::urg_proxy::timeadjust_initialize( &tmadj_prop, &tmadj, &dtime, &htime );
						}
					} // <--- time adjust

					{ // ---> scanning property
						// step min, max
						scan_prop.step.min = param.step_front
								+ ((double) dev_conf.angle_range.value[0] / (360.0 / param.step_resolution));
						scan_prop.step.min = scan_prop.step.min > param.step_min ? scan_prop.step.min : param.step_min;
						scan_prop.step.max = param.step_front
								+ (dev_conf.angle_range.value[1] / (360.0 / param.step_resolution));
						scan_prop.step.max = scan_prop.step.max < param.step_max ? scan_prop.step.max : param.step_max;
						::fprintf(stderr, " ... scanning range is from %d to %d\n", scan_prop.step.min, scan_prop.step.max);
						dev_conf.angle_range.value[0] = (double) 360.0 * (scan_prop.step.min - param.step_front) / param.step_resolution;
						dev_conf.angle_range.value[1] = (double) 360.0 * (scan_prop.step.max - param.step_front) / param.step_resolution;
						// reflect
						scan_prop.intensity = dev_conf.reflect.value;
						::fprintf(stderr, " ... get intensity \"%s\"\n", scan_prop.intensity ? "on" : "off");
					} // <--- scanning property


					{ // ---> scan reading property
						scan_prop.step_front = param.step_front;
						scan_prop.angle_resolution = 2 * M_PI / param.step_resolution;
					} // <--- scan reading property
				}
				// ---> find device configuration

				{ // ---> ssm-property
					scan_ssm.property.numPoints = ( scan_prop.step.max - scan_prop.step.min + 1 );
					scan_ssm.property.distMin = gnd_mm2dist(param.dist_min);// mm -> m
					scan_ssm.property.distMax = gnd_mm2dist(param.dist_max);// mm -> m
					scan_ssm.property.angMin =  ( scan_prop.step.min - param.step_front ) * param.revolution;
					scan_ssm.property.angMax =  ( scan_prop.step.max - param.step_front ) * param.revolution;
					scan_ssm.property.angResolution =  param.revolution;
					scan_ssm.property.cycle = 1.0/ ( (double)(param.revolution) / 60.0 );
					gnd::matrix::copy( &scan_ssm.property.coordm, &ssm_prop.coord);

					strncpy( scan_ssm.property.sensorInfo.firmware, version.firmware, ssm::ScanPoint2DProperty::LENGTH_MAX );
					strncpy( scan_ssm.property.sensorInfo.product,  version.product,  ssm::ScanPoint2DProperty::LENGTH_MAX );
					strncpy( scan_ssm.property.sensorInfo.protocol, version.protocol, ssm::ScanPoint2DProperty::LENGTH_MAX );
					strncpy( scan_ssm.property.sensorInfo.id,       version.serialno, ssm::ScanPoint2DProperty::LENGTH_MAX );
					strncpy( scan_ssm.property.sensorInfo.vendor,   version.vender,   ssm::ScanPoint2DProperty::LENGTH_MAX );

				} // <--- ssm-property

				// allocate data buffer
				scan_ssm.data.alloc( scan_ssm.property.numPoints );

				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- open device port



		// ---> initialize ssm
		if(!::is_proc_shutoff()){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Initailize SSM\n");
			if( !::initSSM() ) {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to initialize \x1b[4mssm\x1b[0m\n");
			}
			else {
				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- initialize ssm


		// ---> create sokuiki-data ssm
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Create ssm-data \"\x1b[4m%s\x1b[0m\"\n", ssm_prop.name);
			if(!scan_ssm.create(ssm_prop.name, ssm_prop.id, 5.0, scan_ssm.property.cycle) ){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", ssm_prop.name);
			}
			else {
				if(!scan_ssm.setProperty()) {
					::proc_shutoff();
					::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", ssm_prop.name);
				}
				else {
					::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", ssm_prop.name);
				}
			}
		} // <--- create sokuiki-data ssm
		::fprintf(stderr, "\n\n");

	} // <--- initialize





	// ---> operation
	if(!::is_proc_shutoff()){
		S2Scan_t *scan_data;
		double scan_htime = 0;
		double recv_htime = 0;
		gnd::inttimer timer_scan;

		gnd::inttimer timer_tmadj;
		double left_tmadj = 0.0;
		bool flg_tmadj = false;
		gnd::urg_proxy::TimeAdjust prev_tmadj = tmadj;

		gnd::inttimer timer_show(CLOCK_REALTIME, 1.0);
		bool show_st = true;

		int nline_show = 0;

		int total = 0;
		int scan_psec = 0;
		int npoints = 0;

		{ // ---> scan cycle
			double cycle = 1.0 / ((double)param.revolution / 60);
			timer_scan.begin(CLOCK_REALTIME, cycle / 2.0);
		} // <--- scan cycle

		// ---> time adjust
		if( tmadj_prop.min_poll > 0 ) {
			double tmadj_next;
			struct timeval htime;
			unsigned long dclock;

			if( !::Scip2CMD_TM_GetSyncTime(port, &dclock, &htime) )		::proc_shutoff();
			else gnd::urg_proxy::timeadjust( &tmadj_prop, &tmadj, &dclock, &htime );

			tmadj_next = gnd::urg_proxy::timeadjust_waittime( &tmadj );
			timer_tmadj.begin(CLOCK_REALTIME, tmadj_next);
		} // <--- time adjust



		// start scan
		if( !gnd::urg_proxy::scanning_begin(port, &scan_prop, &buffer) < 0) {
			::proc_shutoff();
		}
		while( !::is_proc_shutoff() ) {
			// wait
			timer_scan.wait();

			// ---> show status
			if( timer_show.clock() && show_st ){
				// back cursor
				if( nline_show ) {
					::fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}

				total += scan_psec;
				nline_show++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", "urg-proxy");
				nline_show++; ::fprintf(stderr, "\x1b[K          serial : %s\n", version.serialno );
				nline_show++; ::fprintf(stderr, "\x1b[K      total scan : %d\n", total );
				nline_show++; ::fprintf(stderr, "\x1b[K    scan / frame : %d\n", scan_psec );
				nline_show++; ::fprintf(stderr, "\x1b[K  number of read : %d\n", npoints );
				nline_show++; ::fprintf(stderr, "\x1b[K   angular field : %lf to %lf\n", dev_conf.angle_range.value[0], dev_conf.angle_range.value[1] );
				nline_show++; ::fprintf(stderr, "\x1b[K      time-stamp : %lf\n", scan_htime );

				nline_show++; ::fprintf(stderr, "\x1b[K     time-adjust : %s\n", flg_tmadj ? "on" : "off" );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :        host %.06lf, device %.06lf\n", tmadj.host, tmadj.device );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :   prev host %.06lf, device %.06lf\n", prev_tmadj.host, prev_tmadj.device );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :transit host %.06lf, device %.06lf\n", tmadj.host - prev_tmadj.host, tmadj.device - prev_tmadj.device );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :       drift %.06lf\n", tmadj.drift );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :        next %.03lf, poll %d\n", left_tmadj, tmadj.poll );
				nline_show++; ::fprintf(stderr, "\x1b[K                 :        diff %.03lf\n", recv_htime - scan_htime );
				nline_show++; ::fprintf(stderr, "\x1b[K\n");
				nline_show++; ::fprintf(stderr, "\x1b[K Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
				scan_psec = 0;
			} // <--- show status


			// ---> time adjust.
			if( tmadj_prop.min_poll > 0 && timer_tmadj.clock(&left_tmadj) > 0 ){
				double tmadj_next;
				struct timeval htime;
				unsigned long dclock;

				prev_tmadj = tmadj;

				// stop ms
				if( !::Scip2CMD_StopMS(port, &buffer) ){
					::proc_shutoff();
				}
				else if( !::Scip2CMD_TM_GetSyncTime(port, &dclock, &htime) ) {
					::proc_shutoff();
				}
				else {
					// restart scan
					::S2Sdd_Init( &buffer );
					::S2Sdd_setCallback(&buffer, callback, &recv_time);
					if( !gnd::urg_proxy::scanning_begin(port, &scan_prop, &buffer) < 0) {
						::proc_shutoff();
					}

					// set clock
					gnd::urg_proxy::timeadjust( &tmadj_prop, &tmadj, &dclock, &htime );
					tmadj_next = gnd::urg_proxy::timeadjust_waittime( &tmadj );
					timer_tmadj.begin(CLOCK_REALTIME, tmadj_next);
				}
			} // <--- time adjust



			// ---> sensor reading
			if( S2Sdd_Begin(&buffer, &scan_data) > 0){
				recv_htime = recv_time;
				npoints = gnd::urg_proxy::scanning_reading(scan_data, &scan_prop, &scan_ssm.data);
				scan_ssm.data.timeStamp( scan_data->time );

				{ // ---> time stamp
					if( tmadj_prop.min_poll > 0 ) {
						gnd::urg_proxy::timeadjust_device2host(gnd_msec2time(scan_data->time), &tmadj, &scan_htime );
						flg_tmadj = true;
					}
					else {
						scan_htime = recv_htime;
						flg_tmadj = false;
					}
					scan_ssm.write(scan_htime);
				} // <--- time stamp
				// count total scan
				scan_psec++;
				// Don't forget S2Sdd_End to unlock buffer
				S2Sdd_End( &buffer );
			}// <--- sensor reading


			if( ::S2Sdd_IsError(&buffer)){
				break;
			}
		}
	} // <--- operation



	{ // ---> finalize
		::endSSM();
		if(port)	::Scip2_Close(port);
	} // <--- finalize


	return 0;
}
