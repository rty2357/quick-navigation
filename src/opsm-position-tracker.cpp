//============================================================================
// Name        : obserbation-probability-position-tracker.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <time.h>

#include <ssmtype/spur-odometry.h>
#include <ssm.hpp>
#include <ssm-log.hpp>
#include "ssm-laser.hpp"

#include "opsm-position-tracker-opt.hpp"
#include "opsm-position-tracker-cui.hpp"

#include "gnd-coord-tree.hpp"
#include "gnd-matrix-coordinate.hpp"
#include "gnd-matrix-base.hpp"

#include "gnd-opsm.hpp"
#include "gnd-odometry-correction.hpp"
#include "gnd-gridmap.hpp"
#include "gnd-shutoff.hpp"
#include "gnd-timer.hpp"
#include "gnd-bmp.hpp"


static const double ShowCycle = gnd_sec2time(1.0);
static const double ClockCycle = gnd_sec2time(1.0) / 1000.0 ;

int main(int argc, char* argv[]) {
	gnd::opsm::optimize_basic	*optimizer = 0;		// optimizer class
	void 						*optim_ini = 0;		// optimization starting value

	gnd::opsm::cmap_t			cnt_smmap;			// probabilistic scan matching counting map
	gnd::opsm::map_t				smmap;				// probabilistic scan matching map

	SSMScanPoint2D				ssm_sokuikiraw;		// sokuiki raw streaming data
	SSMApi<Spur_Odometry>		ssm_odometry;		// odometry
	SSMApi<Spur_Odometry>		ssm_position_write;	// corrected position
	SSMApi<Spur_Odometry>		ssm_position_read;	// corrected position


	gnd::matrix::coord_tree coordtree;				// coordinate tree
	int coordid_gl = -1,							// global coordinate node id
			coordid_rbt = -1,						// robot coordinate node id
			coordid_sns = -1,						// sensor coordinate node id
			coordid_odm = -1,						// odometry coordinate node id
			coordid_sm	= -1,						// this coordinate's origin is scan matching result position
			coordid_sns_sm = -1;					// sensor coordinate on scan matching result position node id

	gnd::cui_reader gcui;							// cui manager

	gnd::odometry::cmap cmap;

	opsm_pt::proc_configuration pconf;			// configuration parameter
	opsm_pt::options popt(&pconf);				// process option analyze class

	FILE *tlog_fp = 0;
	FILE *llog_fp = 0;
	FILE *t4re_fp = 0;

	{
		gnd::opsm::debug_set_log_level(0);
		gnd::opsm::debug_set_fstream("debug.log");
	}


	{ // ---> initialization
		int ret;								// function return value
		uint32_t phase = 1;						// initialize phase

		// ---> read process options
		if( (ret = popt.get_option(argc, argv)) != 0 ) {
			return ret;
		} // <--- read process options



		{ // ---> coordinate-tree set robot coordinate
			gnd::coord_matrix cc; // coordinate relation matrix

			// set global coordinate
			gnd::matrix::set_unit(&cc);
			coordid_gl = coordtree.add("global", "root", &cc);

			// set robot coordinate
			gnd::matrix::set_unit(&cc);
			coordid_rbt = coordtree.add("robot", "global", &cc);

			// set odometry coordinate
			gnd::matrix::set_unit(&cc);
			coordid_odm = coordtree.add("odometry", "root", &cc);	// local dead-reckoning

			// set scan matching coordinate
			gnd::matrix::set_unit(&cc);
			coordid_sm = coordtree.add("scan-matching", "global", &cc);	// this coordinate's origin is scan matching result position

			// set scan matching coordinate
			gnd::matrix::set_unit(&cc);
			coordid_sns = coordtree.add("sensor", "robot", &cc);

			// set scan matching coordinate
			gnd::matrix::set_unit(&cc);
			coordid_sns_sm = coordtree.add("sensor", "scan-matching", &cc);

		} // <--- coordinate-tree set robot coordinate



		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off



		{ // ---> show initialize task
			::fprintf(stderr, "==========Initialize==========\n");
			if( *pconf.cmap.value ) {
				::fprintf(stderr, " %d. load correction map from \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.cmap.value);
			}
			::fprintf(stderr, " %d. create optimizer class \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.optimizer.value);
			if( pconf.map_update.value && *pconf.init_opsm_map.value ) {
				::fprintf(stderr, " %d. Map Data Load\n", phase++);
				::fprintf(stderr, " %d. Build %sMap\n", phase++, pconf.ndt.value ? "NDT " : "");
			}
			::fprintf(stderr, " %d. Init ssm\n", phase++ );
			::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.corrected_name.value);
			::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.ls_name.value);
			::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.odm_name.value);
			::fprintf(stderr, " %d. Initialize viewer\n", phase++);
			::fprintf(stderr, "\n\n");
		} // <--- show initialize task




		// ---> load odometry correction map
		if( !::is_proc_shutoff() && *pconf.cmap.value ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => load odometry error correction map \"\x1b[4m%s\x1b[0m\"\n", pconf.cmap.value);
			if( cmap.fread(pconf.cmap.value) < 0 ) {
				::proc_shutoff();
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to load odometry error correction map \"\x1b[4m%s\x1b[0m\"\n", pconf.cmap.value);
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: load odometry error correction map \"\x1b[4m%s\x1b[0m\"\n", pconf.cmap.value);
			}
		} // <--- load odometry correction map




		// ---> set optimizer
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create optimizer class \"\x1b[4m%s\x1b[0m\"\n", pconf.optimizer.value);
			// Newton Method
			if( !::strcmp(pconf.optimizer.value, opsm_pt::OptNewton) ){
				optimizer = new gnd::opsm::newton;
				optimizer->initial_parameter_create(&optim_ini);
				optimizer->set_converge_threshold(pconf.converge_dist.value, pconf.converge_orient.value );
				::fprintf(stderr, " ... newton's method \x1b[1mOK\x1b[0m\n");
			}
			// Monte Calro Method
			else if( !::strcmp(pconf.optimizer.value, opsm_pt::OptMCL)){
				gnd::opsm::mcl::initial_parameter *p;
				optimizer = new gnd::opsm::mcl;
				optimizer->initial_parameter_create(&optim_ini);
				p = static_cast<gnd::opsm::mcl::initial_parameter*>(optim_ini);
				p->n = 250;
				optim_ini = static_cast<void*>(p);
				optimizer->set_converge_threshold(pconf.converge_dist.value, pconf.converge_orient.value );
				::fprintf(stderr, "  ... monte calro method \x1b[1mOK\x1b[0m\n");
			}
			// Quasi Monte Calro Method
			else if( !::strcmp(pconf.optimizer.value, opsm_pt::OptQMC)){
				gnd::opsm::qmc::initial_parameter *p;
				optimizer = new gnd::opsm::qmc;
				optimizer->initial_parameter_create(&optim_ini);
				p = static_cast<gnd::opsm::qmc::initial_parameter*>(optim_ini);
				p->n = 2;
				optim_ini = static_cast<void*>(p);
				optimizer->set_converge_threshold(pconf.converge_dist.value, pconf.converge_orient.value );
				::fprintf(stderr, "  ... quasi monte calro method \x1b[1mOK\x1b[0m\n");
			}
			else if( !::strcmp(pconf.optimizer.value, opsm_pt::OptQMC2Newton)){
				gnd::opsm::hybrid_q2n::initial_parameter *p;
				optimizer = new gnd::opsm::hybrid_q2n;
				optimizer->initial_parameter_create(&optim_ini);
				p = static_cast<gnd::opsm::hybrid_q2n::initial_parameter*>(optim_ini);
				p->n = 2;
				optim_ini = static_cast<void*>(p);
				optimizer->set_converge_threshold(pconf.converge_dist.value, pconf.converge_orient.value );
				::fprintf(stderr, "  ... quasi monte calro and newton hybrid \x1b[1mOK\x1b[0m\n");
			}
			else {
				::proc_shutoff();
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid optimizer type\n");
			}

		} // ---> set optimizer




		// ---> build map
		if( !::is_proc_shutoff() && pconf.map_update.value && *pconf.init_opsm_map.value ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => load scan matching map from \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
			if( gnd::opsm::read_counting_map(&cnt_smmap, pconf.init_opsm_map.value) < 0){
				::proc_shutoff();
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to load scan matching map \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
			}
			else if( !pconf.ndt.value){
				if( gnd::opsm::build_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0) {
					::proc_shutoff();
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build scan matching map \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
				}
				else {
					::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: load scan matching map \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
				}
			}
			else {
				if(gnd::opsm::build_ndt_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0){
					::proc_shutoff();
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build scan matching map \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
				}
				else {
					::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: load scan matching map \"\x1b[4m%s\x1b[0m\"\n", pconf.init_opsm_map.value);
				}
			}
		} // <--- build map


		// set map
		if(!::is_proc_shutoff() )	optimizer->set_map(&smmap);


		// ---> initialize ssm
		if(!::is_proc_shutoff()){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Initailize SSM\n");
			if( !::initSSM() ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to initialize \x1b[4mssm\x1b[0m\n");
			}
			else {
				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- initialize ssm


		// ---> create corrected position ssmdata
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create corrected position ssmdata  \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.corrected_name.value, pconf.corrected_id.value);

			// ---> create log file
			if( !ssm_position_write.create(pconf.corrected_name.value, pconf.corrected_id.value, 1, 0.005) ){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to create ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.corrected_name.value, pconf.corrected_id.value);
			} // <--- create log file
			else if(!ssm_position_read.openWait(pconf.corrected_name.value, pconf.corrected_id.value, 0.0)){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to create ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.corrected_name.value, pconf.corrected_id.value);
			}
			else {
				::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.corrected_name.value);
				ssm_position_write.data.x = 0;
				ssm_position_write.data.y = 0;
				ssm_position_write.data.theta = 0;
			}
		} // <--- create corrected position ssmdata




		// ---> open ssm odometry
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => load ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.odm_name.value);
			if( !(*pconf.odm_name.value) ){
				// shut off
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: missing log file operand\n" );
				::fprintf(stderr, "     please show help, ./%s -S <sokuiki.log> -O <odometry.log>\n", opsm_pt::proc_name );
			}
			else {
				::fprintf(stderr, "    File \"\x1b[4m%s\x1b[0m\"\n", pconf.odm_name.value);

				if( !ssm_odometry.openWait( pconf.odm_name.value, pconf.odm_id.value, 0.0, SSM_READ ) ){
					::proc_shutoff();
					::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.odm_name.value);
				}
				else {
					::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.odm_name.value);
					{ // ---> set coordinate
						gnd::matrix::fixed<4,4> pos_cc;

						gnd::matrix::coordinate_converter(&pos_cc,
								0, 0, 0,
								::cos(0), ::sin(0), 0,
								 0, 0, 1);

						coordtree.set_coordinate(coordid_odm, &pos_cc);
					} // ---> set coordinate
					ssm_odometry.setBlocking(true);
					ssm_odometry.readLast();
				}
			}
		} // <--- open ssm odometry



		// ---> open ssm sokuiki raw data
		if(!::is_proc_shutoff()){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => load ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.ls_name.value);

			if( ! *pconf.ls_name.value ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: missing log file operand\n" );
				::fprintf(stderr, "     please show help, ./%s -S <sokuiki.log> -O <odometry.log>\n", opsm_pt::proc_name );
			}
			else {
				::fprintf(stderr, "    File \"\x1b[4m%s\x1b[0m\"\n", pconf.odm_name.value);

				if( !ssm_sokuikiraw.openWait( pconf.ls_name.value, pconf.ls_id.value, 0.0, SSM_READ ) ){
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to load ssm \"\x1b[4m%s\x1b[0m\"\n", pconf.ls_name.value);
				}
				// get property
				else if( !ssm_sokuikiraw.getProperty() ){
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to get property of \"\x1b[4m%s\x1b[0m\"\n", pconf.ls_name.value);
				}
				else {
					//					ssm_sokuikiraw.setBlocking(true);

					// allocate
					ssm_sokuikiraw.data.alloc(ssm_sokuikiraw.property.numPoints);

					// define coordinate
					coordtree.set_coordinate(coordid_sns, &ssm_sokuikiraw.property.coordm);
					coordtree.set_coordinate(coordid_sns_sm, &ssm_sokuikiraw.property.coordm);

					ssm_sokuikiraw.readLast();
					::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
				}
			}
		} // <--- open ssm sokuiki raw data


		// ---> make output directory
		if( !::is_proc_shutoff() && *pconf.output_dir.value ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => make output directory \"\x1b[4m%s\x1b[0m\"\n", pconf.output_dir.value);

			errno = 0;
			if( mkdir(pconf.output_dir.value, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IROTH ) < 0) {
				if( errno != EEXIST ) {
					::proc_shutoff();
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to make output directory \"\x1b[4m%s\x1b[0m\"\n", pconf.output_dir.value);
				}
				else {
					struct stat st;
					::stat(pconf.output_dir.value, &st);

					if( S_ISDIR(st.st_mode)) {
						::fprintf(stderr, " ...\x1b[1mOK\x1b[0m: output directory \"\x1b[4m%s\x1b[0m\" is already exist\n", pconf.output_dir.value);
					}
					else {
						::proc_shutoff();
						::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: \"\x1b[4m%s\x1b[0m\" is already exist and it is not directory\n", pconf.output_dir.value);
					}
				}
			}
			else {
				::fprintf(stderr, " ...\x1b[1mOK\x1b[0m: make output directory \"\x1b[4m%s\x1b[0m\"\n", pconf.output_dir.value);
			}
		} // <--- make output directory


		if ( !::is_proc_shutoff() && *pconf.trajectory_log.value) {
			char fname[512];
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open trajectory log file\n");

			if( ::snprintf(fname, sizeof(fname), "%s/%s", *pconf.output_dir.value ? pconf.output_dir.value : "./", pconf.trajectory_log.value) == sizeof(fname) ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: file path is too long\n");
			}
			else if( !(tlog_fp = fopen( fname, "w" )) ) {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
			}
			else {
				::fprintf(tlog_fp, "# 1.[time, s] 2.[x] 3.[y] 4.[theta] 5.[v] 6.[w]. 7.[adjust-x] 8.[adjust-y] 9.[adjust-theta] 10.[adjust-v] 11.[adjust-w].\n");
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		}

		if ( !::is_proc_shutoff() && *pconf.laserpoint_log.value) {
			char fname[512];
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open laser point log file\n");

			if( ::snprintf(fname, sizeof(fname), "%s/%s", *pconf.output_dir.value ? pconf.output_dir.value : "./", pconf.laserpoint_log.value) == sizeof(fname) ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: file path is too long\n");
			}
			else if( !(llog_fp = fopen( fname, "w" )) ) {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		}

		if ( !::is_proc_shutoff() && *pconf.trajectory4route.value) {
			char fname[512];
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open trajectory file for route edit\n");

			if( ::snprintf(fname, sizeof(fname), "%s/%s", *pconf.output_dir.value ? pconf.output_dir.value : "./", pconf.trajectory4route.value) == sizeof(fname) ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: file path is too long\n");
			}
			else if( !(t4re_fp = fopen( fname, "w" )) ) {
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
			}
			else {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m\n");
			}
		}


		if ( !::is_proc_shutoff() && !cmap.is_allocate()) {
			// create map
			gnd::odometry::correction::create(&cmap, pconf.pos_gridsizex.value, pconf.pos_gridsizey.value, pconf.ang_rsl.value);
		}


		// set cui command
		gcui.set_command(opsm_pt::cui_cmd, sizeof(opsm_pt::cui_cmd) / sizeof(opsm_pt::cui_cmd[0]));


		// fin of initialization
		::fprintf(stderr, "\n\n");
	} // <--- initialization












	// ---> operation
	if(!::is_proc_shutoff() ){
		int ret = 0;									// function return value
		int cnt = 0;

		Spur_Odometry prev_odometry = ssm_odometry.data; // previous odometry position
		Spur_Odometry move_est;							// estimation of movement quantity

		double culling_sqdist							// data decimation threshold
		= gnd_square( pconf.culling.value );
		double lkl = 0;									// likelihood
		Spur_Odometry pos_opt = ssm_position_read.data; // optimized position
		int cnt_opt = 0;								// optimization loop counter
		int cnt_correct = 0;

		gnd::vector::fixed_column<3> move_opt;			// position estimation movement by optimization
		double change_dist = 0;							// change value on distance between previous and current frame
		double change_orient = 0;						// change value on orientation between previous and current frame

		double init_time = ssm_odometry.time;
		//		Spur_Odometry pos = ssm_odometry.data();
		Spur_Odometry pos_premap;						// odometry position streaming data
		ssmTimeT time_premap = 0;						// previous map update time

		gnd::matrix::fixed<4,4> coordm_sns2rbt;			// coordinate convert matrix from sensor to robot
		gnd::matrix::fixed<4,4> coordm_sns2gl;			// coordinate convert matrix from sensor to global

		double cuito = 0;								// blocking time out for cui input

		gnd::timer::interval_timer timer_clock;			// clock
		gnd::timer::interval_timer timer_operate;			// operation timer
		gnd::timer::interval_timer timer_show;			// time operation timer
		int nline_show;

		bool mapupdate = false;
		int cnt_mapupdate = 0;

		int cnt_fail = 0;

		// get coordinate convert matrix
		coordtree.get_convert_matrix(coordid_sns, coordid_rbt, &coordm_sns2rbt);

		{ // ---> set zero
			move_est.x = 0;
			move_est.y = 0;
			move_est.theta = 0;
			move_est.v = 0;
			move_est.w = 0;
		} // <--- set zero



		// ---> memory allocate counting map
		if( !cnt_smmap.plane[0].is_allocate() ){
			gnd::opsm::init_counting_map(&cnt_smmap, 0.4, 10);
		} // <--- memory allocate counting map


		if( !(*pconf.init_opsm_map.value) ){ // ---> map initialization
			int cnt_ls = 0;

			::fprintf(stderr, "-------------------- map initialize  --------------------\n");
			// ---> map initialization loop
			while( !::is_proc_shutoff() && cnt_ls < pconf.ini_map_cnt.value ) {
				// ---> read ssm-sokuikiraw-data
				if(ssm_sokuikiraw.readNew()) {
					{ // ---> 1. compute position estimation from odometry
						// get position on odometry position (on odometry cooordinate)
						if( !ssm_odometry.readTime( ssm_sokuikiraw.time ) ) continue;

						{ // ---> compute the movement estimation
							gnd::vector::fixed_column<4> odov_cprev;		// current odometry position vector on previous odometry coordinate

							{ // ---> compute current odometry position on previous odometry coordinate
								gnd::matrix::fixed<4,4> coordm_r2podo;		// coordinate matrix of previous odometry position
								gnd::vector::fixed_column<4> ws4x1;

								// get previous odometry coordinate matrix
								coordtree.get_convert_matrix(0, coordid_odm, &coordm_r2podo);

								// multiply previous odometry coordinate matrix with current position vector
								ws4x1[0] = ssm_odometry.data.x;
								ws4x1[1] = ssm_odometry.data.y;
								ws4x1[2] = 0;
								ws4x1[3] = 1;
								gnd::matrix::prod(&coordm_r2podo, &ws4x1, &odov_cprev);
							} // <--- compute current odometry position on previous odometry coordinate

							// get movement estimation by odometry
							move_est.x = odov_cprev[0];
							move_est.y = odov_cprev[1];
							move_est.theta = ssm_odometry.data.theta - prev_odometry.theta;
						} // <--- compute the movement estimation


						{ // ---> add movement estimation
							gnd::vector::fixed_column<4> pos_odmest;

							{ // ---> compute position estimation by odometry on global coordinate
								gnd::matrix::fixed<4,4> coordm_rbt2gl;		// coordinate convert matrix from robot to global
								gnd::vector::fixed_column<4> ws4x1;

								// set search position on sensor-coordinate
								coordtree.get_convert_matrix(coordid_rbt, coordid_gl, &coordm_rbt2gl);

								ws4x1[0] = move_est.x;
								ws4x1[1] = move_est.y;
								ws4x1[2] = 0;
								ws4x1[3] = 1;

								gnd::matrix::prod(&coordm_rbt2gl, &ws4x1, &pos_odmest);
							} // <--- compute position estimation by odometry on global coordinate

							// set position
							ssm_position_write.data.x = pos_odmest[0];
							ssm_position_write.data.y = pos_odmest[1];
							ssm_position_write.data.theta += move_est.theta;
						} // <--- add movement estimation
					}  // <--- 1. compute position estimation from odometry


					{ // ---> 2. update robot position coordinate and odometory position coordinate
						gnd::matrix::fixed<4,4> coordm;

						// odometry coordinate
						gnd::matrix::coordinate_converter(&coordm,
								ssm_odometry.data.x, ssm_odometry.data.y, 0,
								::cos(ssm_odometry.data.theta), ::sin(ssm_odometry.data.theta), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_odm, &coordm);

						// robot position coordinate
						gnd::matrix::coordinate_converter(&coordm,
								ssm_position_write.data.x, ssm_position_write.data.y, 0,
								::cos( ssm_position_write.data.theta ), ::sin( ssm_position_write.data.theta ), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_rbt, &coordm);

						// get coordinate convert matrix
						coordtree.get_convert_matrix(coordid_sns, coordid_gl, &coordm_sns2gl);
					} // ---> 2. update robot position coordinate and odometory position coordinate

					gnd::matrix::set_zero(&move_opt);
					{ // ---> 3. entry laser scanner reading
						gnd::vector::fixed_column<3> delta;
						gnd::vector::fixed_column<2> reflect_prevent;

						// clear previous entered sensor reading
						gnd::matrix::set_zero(&reflect_prevent);

						// ---> scanning loop for sokuikiraw-data
						for(size_t i = 0; i < ssm_sokuikiraw.data.numPoints(); i++){
							// ---> entry laser scanner reflection
							gnd::vector::fixed_column<4> reflect_csns, reflect_cgl;
							gnd::vector::fixed_column<3> ws3x1;
							gnd::matrix::fixed<3,3> ws3x3;

							// if range data is null because of no reflection
							if(ssm_sokuikiraw.data[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
							// ignore error data
							else if(ssm_sokuikiraw.data[i].isError()) 	continue;
							else if(ssm_sokuikiraw.data[i].r < ssm_sokuikiraw.property.distMin * 1.1)	continue;
							else if(ssm_sokuikiraw.data[i].r > ssm_sokuikiraw.property.distMax * 0.9)	continue;
							else if( pconf.use_range_dist.value > 0 && ssm_sokuikiraw.data[i].r > pconf.use_range_dist.value )		continue;
							else if( pconf.use_range_orient.value > 0 && ssm_sokuikiraw.data[i].th > pconf.use_range_orient.value )	continue;


							{ // ---> compute laser scanner reading position on robot coordinate
								// set search position on sensor-coordinate
								reflect_csns[0] = ssm_sokuikiraw.data[i].r * ::cos(ssm_sokuikiraw.data[i].th);
								reflect_csns[1] = ssm_sokuikiraw.data[i].r * ::sin(ssm_sokuikiraw.data[i].th);
								reflect_csns[2] = 0;
								reflect_csns[3] = 1;

								// data decimation with distance threshold
								if( gnd_square(reflect_csns[0] - reflect_prevent[0]) + gnd_square(reflect_csns[1] - reflect_prevent[1]) < culling_sqdist ){
									continue;
								}
								else {
									// update previous entered data
									gnd::matrix::copy(&reflect_prevent, &reflect_csns);
								}

								// convert from sensor coordinate to robot coordinate
								gnd::matrix::prod(&coordm_sns2gl, &reflect_csns, &reflect_cgl);
							} // <--- compute laser scanner reading position on robot coordinate

							// data entry
							gnd::opsm::counting_map(&cnt_smmap, reflect_cgl[0], reflect_cgl[1]);

							// log
							if( llog_fp )	::fprintf(llog_fp, "%lf %lf\n", reflect_cgl[0], reflect_cgl[1]);
						} // <--- scanning loop for sokuikiraw-data

						// log
						if( llog_fp )	::fprintf(llog_fp, "\n" );
					} // <--- 3. entry laser scanner reading



					prev_odometry = ssm_odometry.data;
					cnt_ls++;
					::fprintf(stderr, ".");
				} // <--- read ssm sokuikiraw
			} // <--- map initialization loop

			// ---> map build
			if( !pconf.ndt.value ){
				if( gnd::opsm::build_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0 ){
					::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
				}
				else {
					::fprintf(stderr, "\n... \x1b[1mOK\x1b[0m success to build psm map\n");
				}
			}
			else if( pconf.ndt.value && gnd::opsm::build_ndt_map(&smmap, &cnt_smmap ) < 0 ){
				::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
			}
			else {
				::fprintf(stderr, "\n... \x1b[1mOK\x1b[0m success to build ndt map\n");
			} // <--- map build
		} // <--- map initialization


		{ // ---> initialize corrected position
			ssm_position_write.data = ssm_odometry.data;
			ssm_position_read.data = ssm_odometry.data;;
			pos_premap =  ssm_odometry.data;
		} // <--- initialize corrected position

		{ // ---> timer
			// set parameter-cycle
			timer_operate.begin(CLOCK_REALTIME, pconf.cycle.value, -pconf.cycle.value);
			timer_clock.begin(CLOCK_REALTIME,
					pconf.cycle.value < ClockCycle ? pconf.cycle.value :  ClockCycle);
			::fprintf(stderr, "\n");
			if( pconf.cui_show.value ) {
				timer_show.begin(CLOCK_REALTIME, ShowCycle, -ShowCycle);
				// console clear
				nline_show = 0;
			}
			else {
				// console clear
				::fprintf(stderr, "-------------------- cui mode --------------------\n");
				::fprintf(stderr, "  > ");
			}
		} // <--- timer



		{ // ---> timer
			// set parameter-cycle
			timer_clock.begin(CLOCK_REALTIME, pconf.cycle.value );
			if( pconf.cui_show.value )	timer_show.begin(CLOCK_REALTIME, ShowCycle, -ShowCycle);
			else 							::fprintf(stderr, "  > ");
		} // <--- timer



		// ---> operation loop
		while ( !::is_proc_shutoff() ) {
			//			timer_clock.wait();

			{ // ---> cui
				int cuival = 0;
				char cuiarg[512];
				// zero reset buffer
				::memset(cuiarg, 0, sizeof(cuiarg));

				// ---> get command
				if( gcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito) > 0 ){
					if( timer_show.cycle() > 0 ){
						// quit show status mode
						timer_show.end();
						nline_show = 0;
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					}
					else { // ---> cui command operation
						switch(cuival){
						// exit
						case 'Q': ::proc_shutoff(); break;
						// help
						default:
						case '\0':
						case 'h': gcui.show(stderr, "   "); break;
						// show status
						case 's': {
							// console clear
							timer_show.begin(CLOCK_REALTIME, ShowCycle, -ShowCycle);
							break;
						}
						case 'f': {
							double freq = ::strtod(cuiarg, 0);
							if( freq <= 0 ){
								::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid argument value (frequency 0)\n");
								::fprintf(stderr, "       if you want to stop estimator, send \"\x1b[4mstand-by\x1b[0m\" command\n");
							}
							else {
								double cyc = 1.0 / freq;
								::fprintf(stderr, "   ... cycle %.03lf\n", cyc);
							}
						} break;

						// set freq
						case 'c': {
							double cyc = ::strtod(cuiarg, 0);
							if( cyc <= 0 ){
								::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid argument value (frequency 0)\n");
								::fprintf(stderr, "       if you want to stop estimator, send \"\x1b[4mstand-by\x1b[0m\" command\n");
							}
							else {
								::fprintf(stderr, "   ... cycle %.03lf\n", cyc);
							}
						} break;

						// start
						case 't':{
							cuito = 0.0;
						} break;
						// stand-by
						case 'B':{
							::fprintf(stderr, "   stand-by mode\n");
							cuito = -1;
						} break;

						}
					} // <--- cui command operation
					::fprintf(stderr, "  > ");
					gcui.poll(&cuival, cuiarg, sizeof( cuiarg ), 0);
				} // <--- get command
			}  // <--- cui


			// ---> show status
			if( timer_show.clock() > 0){
				// back cursor
				if( nline_show ) {
					::fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}

				nline_show++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", opsm_pt::proc_name);
				nline_show++; ::fprintf(stderr, "\x1b[K matching method : %s\n", pconf.ndt.value ? "ndt" : "opsm");
				nline_show++; ::fprintf(stderr, "\x1b[K       optimizer : %s\n", pconf.optimizer.value );
				nline_show++; ::fprintf(stderr, "\x1b[K            loop : %d\n", cnt);

				nline_show++; ::fprintf(stderr, "\x1b[K   optimize loop : %d\n", cnt_opt);
				nline_show++; ::fprintf(stderr, "\x1b[K      likelihood : %.03lf\n", lkl );
				nline_show++; ::fprintf(stderr, "\x1b[K        position : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						ssm_position_write.data.x, ssm_position_write.data.y, gnd_ang2deg( ssm_position_write.data.theta ) );
				nline_show++; ::fprintf(stderr, "\x1b[K        optimize : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						move_opt[0], move_opt[1], gnd_ang2deg( move_opt[2] ) );
				nline_show++; ::fprintf(stderr, "\x1b[K        move est : %4.03lf[m], %4.03lf[m], %4.02lf[deg]\n",
						move_est.x, move_est.y, gnd_ang2deg( move_est.theta ) );
				//				::fprintf(stderr, "      cycle : %.03lf\n", timer_operate.cycle() );
				nline_show++; ::fprintf(stderr, "\x1b[K        optimize : %s\n", ret == 0 ? "success" : "failure" );
				nline_show++; ::fprintf(stderr, "\x1b[K      map update : %d\n", cnt_mapupdate );
				if( pconf.use_range_dist.value > 0){
					nline_show++; ::fprintf(stderr, "\x1b[K*      use range : %lf [m]\n", pconf.use_range_dist.value );
				}
				if( pconf.use_range_orient.value > 0){
					nline_show++; ::fprintf(stderr, "\x1b[K*      use range : %lf [deg]\n", pconf.use_range_orient.value );
				}
				nline_show++; ::fprintf(stderr, "\x1b[K   matching fail : %d\n", cnt_fail );
				nline_show++; ::fprintf(stderr, "\x1b[K\n");
				nline_show++; ::fprintf(stderr, "\x1b[K Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
			} // <--- show status


			// ---> update position
			if( ssm_odometry.readNext() ){
				{ // ---> compute the movement estimation
					gnd::vector::fixed_column<4> odov_cprev;			// current odometry position vector on previous odometry coordinate

					{ // ---> compute current odometry position on previous odometry coordinate
						gnd::matrix::fixed<4,4> coordm_r2podo;		// coordinate matrix of previous odometry position
						gnd::vector::fixed_column<4> ws4x1;

						// get previous odometry coordinate matrix
						coordtree.get_convert_matrix(0, coordid_odm, &coordm_r2podo);

						// multiply previous odometry coordinate matrix with current position vector
						ws4x1[0] = ssm_odometry.data.x;
						ws4x1[1] = ssm_odometry.data.y;
						ws4x1[2] = 0;
						ws4x1[3] = 1;
						gnd::matrix::prod(&coordm_r2podo, &ws4x1, &odov_cprev);
					} // <--- compute current odometry position on previous odometry coordinate

					// get movement estimation by odometry
					move_est.x = odov_cprev[0];
					move_est.y = odov_cprev[1];
					move_est.theta = ssm_odometry.data.theta - prev_odometry.theta;
				} // <--- compute the movement estimation

				{ // ---> add movement estimation
					gnd::vector::fixed_column<4> pos_odmest;

					{ // ---> compute position estimation by odometry on global coordinate
						gnd::matrix::fixed<4,4> coordm_rbt2gl;		// coordinate convert matrix from robot to global
						gnd::vector::fixed_column<4> ws4x1;

						// set search position on sensor-coordinate
						coordtree.get_convert_matrix(coordid_rbt, coordid_gl, &coordm_rbt2gl);

						ws4x1[0] = move_est.x;
						ws4x1[1] = move_est.y;
						ws4x1[2] = 0;
						ws4x1[3] = 1;

						gnd::matrix::prod(&coordm_rbt2gl, &ws4x1, &pos_odmest);
					} // <--- compute position estimation by odometry on global coordinate

					// set position
					ssm_position_write.data.x = pos_odmest[0];
					ssm_position_write.data.y = pos_odmest[1];
					ssm_position_write.data.theta += move_est.theta;
				} // <--- add movement estimation

				{// ---> update
					gnd::matrix::fixed<4,4> coordm;

					// write into ssm
					ssm_position_write.write(ssm_odometry.time);

					// update previous odometry estimation
					prev_odometry = ssm_odometry.data;


					{ // ---> update coordinate
						gnd::matrix::coordinate_converter(&coordm,
								ssm_odometry.data.x, ssm_odometry.data.y, 0,
								::cos(ssm_odometry.data.theta), ::sin(ssm_odometry.data.theta), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_odm, &coordm);

						// robot position coordinate
						gnd::matrix::coordinate_converter(&coordm,
								ssm_position_write.data.x, ssm_position_write.data.y, 0,
								::cos( ssm_position_write.data.theta ), ::sin( ssm_position_write.data.theta ), 0,
								 0, 0, 1);
						coordtree.set_coordinate(coordid_rbt, &coordm);
					} // <--- update coordinate
				} // <--- update
			} // <--- update position


			// ---> read ssm-sokuikiraw-data
			if( timer_operate.clock() && ssm_sokuikiraw.readNew()  ) {
				// ---> position tracking
				// ... operation flow
				//      *0. get laser scanner reading
				//		 2. set position estimation by odometry to optimization starting value
				//		 3. optimization iteration by matching laser scanner reading to map(likelihood field)
				//		 4. optimization error test and write position ssm-data
				//		 5. robot current position coordinate and odometory position coordinate

				// read sokuiki data
				if( !ssm_position_read.readTime( ssm_sokuikiraw.time) ) continue;

				// ---> 2. set position estimation by odometry as optimization starting value
				optimizer->initial_parameter_set_position( optim_ini, ssm_position_read.data.x, ssm_position_read.data.y, ssm_position_read.data.theta );
				optimizer->begin(optim_ini);


				gnd::matrix::set_zero(&move_opt);
				{ // ---> 3. optimization iteration by matching laser scanner reading to map(likelihood field)
					gnd::vector::fixed_column<3>	delta;
					gnd::vector::fixed_column<2>	reflect_prevent;

					// clear previous entered sensor reading
					gnd::matrix::set_zero(&reflect_prevent);

					// ---> scanning loop for sokuikiraw-data
					for(size_t i = 0; i < ssm_sokuikiraw.data.numPoints(); i++){
						// ---> entry laser scanner reflection
						gnd::vector::fixed_column<4> reflect_csns, reflect_crbt;
						gnd::vector::fixed_column<3> ws3x1;
						gnd::matrix::fixed<3,3>		 ws3x3;

						// if range data is null because of no reflection
						if( ssm_sokuikiraw.data[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
						// ignore error data
						else if( ssm_sokuikiraw.data[i].isError()) 	continue;
						else if( ssm_sokuikiraw.data[i].r < ssm_sokuikiraw.property.distMin * 1.1)	continue;
						else if( ssm_sokuikiraw.data[i].r > ssm_sokuikiraw.property.distMax * 0.9)	continue;
						else if( pconf.use_range_dist.value > 0 && ssm_sokuikiraw.data[i].r > pconf.use_range_dist.value )		continue;
						else if( pconf.use_range_orient.value > 0 && ssm_sokuikiraw.data[i].th > pconf.use_range_orient.value )	continue;

						{ // ---> compute laser scanner reading position on robot coordinate
							// set search position on sensor-coordinate
							reflect_csns[0] = ssm_sokuikiraw.data[i].r * ::cos( ssm_sokuikiraw.data[i].th );
							reflect_csns[1] = ssm_sokuikiraw.data[i].r * ::sin( ssm_sokuikiraw.data[i].th );
							reflect_csns[2] = 0;
							reflect_csns[3] = 1;

							// data decimation with distance threshold
							if( gnd_square(reflect_csns[0] - reflect_prevent[0]) + gnd_square(reflect_csns[1] - reflect_prevent[1]) < culling_sqdist ){
								continue;
							}
							else {
								// update previous entered data
								gnd::matrix::copy(&reflect_prevent, &reflect_csns);
							}

							// convert from sensor coordinate to robot coordinate
							gnd::matrix::prod(&coordm_sns2rbt, &reflect_csns, &reflect_crbt);
						} // <--- compute laser scanner reading position on robot coordinate

						// data entry
						optimizer->set_scan_point( reflect_crbt[0], reflect_crbt[1] );
						// <--- entry laser scanner reflection
					} // <--- scanning loop for sokuikiraw-data

					if( optimizer->nscan_point() <= 0 )	continue;

					// zero reset likelihood
					lkl = 0;
					// zero reset optimization iteration counter
					cnt_opt = 0;
					gnd::matrix::set_zero(&move_opt);
					do{
						// store previous optimization position likelihood

						{ // ---> step iteration of optimization
							gnd::vector::fixed_column<3> ws3x1;
							if( (ret = optimizer->iterate(&delta, &ws3x1, &lkl)) < 0 ){
								break;
							}

							// get optimized position
							pos_opt.x = ws3x1[0];
							pos_opt.y = ws3x1[1];
							pos_opt.theta = ws3x1[2];
							// get movement by optimization
							gnd::matrix::add(&move_opt, &delta, &move_opt);
						} // <--- step iteration of optimization

						// loop counting
						cnt_opt++;
						// convergence test
					} while( !optimizer->converge_test() ); // <--- position optimization loop

				} // ---> 3. optimization iteration by matching laser scanner reading to map(likelihood field)



				// ---> 4. optimization error test and write position ssm-data
				// check --- 1st. function error, 2nd. distance, 3rd. orient difference
				if( ret >= 0 &&
						gnd_square( ssm_position_read.data.x - pos_opt.x ) + gnd_square( ssm_position_read.data.y - pos_opt.y ) < gnd_square( pconf.failure_dist.value ) &&
						::fabs( gnd_rad_normalize( ssm_position_read.data.theta - pos_opt.theta) ) < pconf.failure_orient.value ) {


					if( (cnt_correct >= pconf.ini_match_cnt.value ||
							( change_dist * change_dist > pconf.pause_dist.value * pconf.pause_dist.value &&
									change_orient > ::fabs(pconf.pause_orient.value) ) ) && change_dist > 0 ) {
						gnd::odometry::correction::counting(&cmap, pos_opt.x, pos_opt.y, pos_opt.theta,
								change_dist, (ssm_position_read.data.x - pos_opt.x), (ssm_position_read.data.y - pos_opt.y), (ssm_position_read.data.theta - pos_opt.theta) );
					}

					ssm_position_write.data.x += move_opt[0];
					ssm_position_write.data.y += move_opt[1];
					ssm_position_write.data.theta += move_opt[2];
					cnt_correct++;


					if( tlog_fp ){
						::fprintf(tlog_fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
								ssm_odometry.time - init_time,
								ssm_odometry.data.x, ssm_odometry.data.y, ssm_odometry.data.theta,
								ssm_odometry.data.v, ssm_odometry.data.w,
								ssm_position_write.data.x, ssm_position_write.data.y, ssm_position_write.data.theta,
								ssm_position_write.data.v, ssm_position_write.data.w );
					}

					if( t4re_fp ) {
						::fprintf(t4re_fp, "A %lf %lf\n",
								ssm_position_write.data.x, ssm_position_write.data.y );
					}

					change_dist = 0;
				}
				else {
					cnt_fail++;
					//					ssmlog_pos.write( ssmlog_sokuikiraw.time() );
				} // <--- 4. optimization error test and write position ssm-data


				{ // ---> 5. update scan matching result coordinate
					gnd::matrix::fixed<4,4> coordm;

					// robot position coordinate
					gnd::matrix::coordinate_converter(&coordm,
							pos_opt.x, pos_opt.y, 0,
							::cos( pos_opt.theta ), ::sin( pos_opt.theta ), 0,
							 0, 0, 1);
					coordtree.set_coordinate(coordid_sm, &coordm);

				} // ---> 5. update scan matching result coordinate



				{ // ---> 6. update map
					// get coordinate convert matrix
					coordtree.get_convert_matrix(coordid_sns_sm, coordid_gl, &coordm_sns2gl);

					// map update check, 1st. time, 2nd. position, 3rd. orient
					mapupdate = ssm_sokuikiraw.time - time_premap > pconf.map_update_time.value ||
							gnd_square( ssm_position_write.data.x - pos_premap.x) + gnd_square( ssm_position_write.data.y - pos_premap.y) > gnd_square(pconf.map_update_dist.value) ||
							::fabs( ssm_position_write.data.theta - pos_premap.theta ) > pconf.map_update_orient.value;

					if( mapupdate ) cnt_mapupdate++;
					if( mapupdate && !pconf.map_update.value ){ // ---> clear
						gnd::opsm::clear_counting_map(&cnt_smmap);
					} // <--- clear

					{// ---> scanning loop for sokuikiraw-data
						gnd::vector::fixed_column<4> reflect_csns;
						gnd::vector::fixed_column<4> reflect_cgl;
						gnd::vector::fixed_column<2> reflect_prevent;

						// ---> scanning loop of laser scanner reading
						for(size_t i = 0; i < ssm_sokuikiraw.data.numPoints(); i++){
							gnd::vector::fixed_column<3> ws3x1;
							gnd::matrix::fixed<3,3> ws3x3;

							// if range data is null because of no reflection
							if( ssm_sokuikiraw.data[i].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
							// ignore error data
							else if( ssm_sokuikiraw.data[i].isError()) 	continue;
							else if( ssm_sokuikiraw.data[i].r < ssm_sokuikiraw.property.distMin)	continue;
							else if( ssm_sokuikiraw.data[i].r > ssm_sokuikiraw.property.distMax)	continue;
							else if( pconf.use_range_dist.value > 0 && ssm_sokuikiraw.data[i].r > pconf.use_range_dist.value )		continue;
							else if( pconf.use_range_orient.value > 0 && ssm_sokuikiraw.data[i].th > pconf.use_range_orient.value )	continue;



							{ // ---> compute laser scanner reading position on global coordinate
								// set search position on sensor-coordinate
								gnd::matrix::set(&reflect_csns, 0, 0, ssm_sokuikiraw.data[i].r * ::cos( ssm_sokuikiraw.data[i].th ));
								gnd::matrix::set(&reflect_csns, 1, 0, ssm_sokuikiraw.data[i].r * ::sin( ssm_sokuikiraw.data[i].th ) );
								gnd::matrix::set(&reflect_csns, 2, 0, 0);
								gnd::matrix::set(&reflect_csns, 3, 0, 1);

								// data decimation with distance threshold
								if( gnd_square(reflect_csns[0] - reflect_prevent[0]) + gnd_square(reflect_csns[1] - reflect_prevent[1]) < culling_sqdist ){
									continue;
								}
								else {
									// update previous entered data
									gnd::matrix::copy(&reflect_prevent, &reflect_csns);
								}

								// convert from sensor coordinate to global coordinate
								gnd::matrix::prod(&coordm_sns2gl, &reflect_csns, &reflect_cgl);
							} // <--- compute laser scanner reading position on global coordinate

							// ---> enter laser scanner reading to map
							if( mapupdate ) {
								if( pconf.map_update.value ){
									if( pconf.ndt.value ) {
										gnd::opsm::update_ndt_map(&cnt_smmap, &smmap, reflect_cgl[0], reflect_cgl[1], gnd_mm2dist(1));
									}
									else {
										gnd::opsm::update_map(&cnt_smmap, &smmap, reflect_cgl[0], reflect_cgl[1], gnd_mm2dist(1));
									}
								}
								else {
									gnd::opsm::counting_map(&cnt_smmap, reflect_cgl[0], reflect_cgl[1]);
								}
								// update
								time_premap = ssm_sokuikiraw.time;
								pos_premap = ssm_position_write.data;
							} // <--- enter laser scanner reading to map

							// log
							if( llog_fp )	::fprintf(llog_fp, "%lf %lf\n", reflect_cgl[0], reflect_cgl[1]);
						} // <--- scanning loop of laser scanner reading

						// log
						if( llog_fp )	::fprintf(llog_fp, "\n" );

					} // <--- scanning loop for sokuikiraw-data


					// ---> rebuild map
					if( mapupdate ) {
						if( pconf.map_update.value ){
						}
						else {
							if( !pconf.ndt.value ){
								if( gnd::opsm::build_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0 ){
									::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
								}
							}
							else{
								if( gnd::opsm::build_ndt_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0 ){
									::fprintf(stderr, "\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: invalid map property\n");
								}
							}
						}
					} // <--- rebuild map
				} // ---> 6. update map
				cnt++;
			} // <--- read ssm sokuikiraw

		} // <--- operation loop

	} // <--- operation



	{ // ---> finalization
		if(tlog_fp) ::fclose(tlog_fp);
		if(llog_fp) ::fclose(llog_fp);
		if(t4re_fp) ::fclose(t4re_fp);

		ssm_odometry.close();
		ssm_position_write.close();
		ssm_sokuikiraw.close();
		::endSSM();

		optimizer->initial_parameter_delete(&optim_ini);
		delete optimizer;

		// slam
		if( pconf.map_update.value ) {

			// ---> build map
			if( pconf.ndt.value ) {
				gnd::opsm::build_ndt_map(&smmap, &cnt_smmap );
			}
			else {
				gnd::opsm::build_map(&smmap, &cnt_smmap, gnd_mm2dist(10));
			} // <--- build map

			if( pconf.opsm_map.value[0] ){ // ---> write opsm map
				char dname[512];
				bool flg = false;

				::fprintf(stderr, " => write intermediate file\n");
				::sprintf(dname, "%s/%s/", *pconf.output_dir.value ? pconf.output_dir.value : "./", pconf.opsm_map.value);

				errno = 0;
				if( mkdir(dname, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IROTH ) < 0) {
					if( errno != EEXIST ) {
						::proc_shutoff();
						::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to make output directory \"\x1b[4m%s\x1b[0m\"\n", dname);
					}
					else {
						struct stat st;
						::stat(pconf.output_dir.value, &st);

						if( S_ISDIR(st.st_mode)) {
							::fprintf(stderr, " ...\x1b[1mOK\x1b[0m: output directory \"\x1b[4m%s\x1b[0m\" is already exist\n", dname);
							flg = true;
						}
						else {
							::proc_shutoff();
							::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: \"\x1b[4m%s\x1b[0m\" is already exist and it is not directory\n", dname);
						}
					}
				}
				else {
					::fprintf(stderr, "  ... make output directory \"\x1b[4m%s\x1b[0m\"\n", dname);
					flg = true;
				}


				if( flg && gnd::opsm::write_counting_map(&cnt_smmap, dname) ) {
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open\n");
				}
				else {
					::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save counting map data\n");
				}
			} // <--- write opsm map



			if( pconf.bmp.value ) { // ---> bmp (32bit)
				gnd::bmp32_t bmp;

				// bmp file building
				gnd::opsm::build_bmp32(&bmp, &smmap, gnd_m2dist( 1.0 / 10));
				{ // ---> bmp
					char fname[512];
					::fprintf(stderr, " => write psm-image in bmp(32bit)\n");

					if( ::snprintf(fname, sizeof(fname), "%s/%s.%s", pconf.output_dir.value, "out", "bmp" ) == sizeof(fname) ){
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open. file name is too long\n");
					}
					else if( gnd::bmp::write32(fname, &bmp) < 0) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
					else {
						::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save map data into \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
				} // <--- bmp

				{ // ---> origin
					char fname[512];
					FILE *fp = 0;
					double x, y;

					if( ::snprintf(fname, sizeof(fname), "%s/%s.%s", pconf.output_dir.value, "out", "origin.txt"  ) == sizeof(fname) ){
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open. file name is too long\n");
					}
					else if( !(fp = fopen(fname, "w")) ) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}

					bmp.pget_origin(&x, &y);
					fprintf(fp, "%lf %lf\n", x, y);
					fclose(fp);
				} // --->  origin
			} // <--- bmp (32bit)


			if( pconf.bmp.value ) { // ---> bmp (8bit)
				gnd::bmp8_t bmp8;

				gnd::opsm::build_bmp8(&bmp8, &smmap, gnd_m2dist( 1.0 / 10));
				{ // ---> bmp
					char fname[512];
					::fprintf(stderr, " => write psm-image in bmp(8bit)\n");

					if( ::snprintf(fname, sizeof(fname), "%s/%s.%s", pconf.output_dir.value, "out8", "bmp" ) == sizeof(fname) ){
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open. file name is too long\n");
					}
					else if( gnd::bmp::write8(fname, &bmp8) < 0) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
					else {
						::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: save map data into \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
				} // <--- bmp

				{ // ---> origin
					char fname[512];
					FILE *fp = 0;
					double x, y;

					if( ::snprintf(fname, sizeof(fname), "%s/%s.%s", pconf.output_dir.value, "out8", "origin.txt"  ) == sizeof(fname) ){
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open. file name is too long\n");
					}
					else if( !(fp = fopen(fname, "w")) ) {
						::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
					}
					bmp8.pget_origin(&x, &y);
					fprintf(fp, "%lf %lf\n", x, y);
					fclose(fp);
				} // --->  origin
			} // ---> bmp (8bit)
		}


		if(pconf.debug_odo_err_map.value) { // ---> file out
			gnd::odometry::correction::vxl *p;
			double x, y, t;
			char fname[128];
			char mapfname[128];

			for( unsigned int zi = 0; zi < cmap.zsize(); zi++ ) {
				FILE *fp;
				double uz, lz;
				::sprintf(fname, "map%d.dat", zi);
				fp = ::fopen(fname, "w");
				cmap.sget_pos_lower(0, 0, zi, 0, 0, &lz);
				cmap.sget_pos_upper(0, 0, zi, 0, 0, &uz);
				::fprintf(fp, "# angular range %lf %lf\n", lz, uz);

				::sprintf(mapfname, "map%d.rmap", zi);
				// ---> row
				for( unsigned int yi = 0; yi < cmap.ysize(); yi++ ) {
					// ---> column
					for( unsigned int xi = 0; xi < cmap.xsize(); xi++ ) {
						p = cmap.pointer(xi, yi, zi);
						cmap.sget_pos_core(xi,yi,zi, &x, &y, &t);
						::fprintf( fp, "%lf %lf, %lf %lf %lf, %lf\n", x, y,
								p->dist > 0.01 ? p->dx / p->dist : 0, p->dist > 0.01 ? p->dy / p->dist : 0, p->dist > 0.01 ? gnd_ang2deg(p->dtheta / p->dist) : 0,
										p->dist);
					} // <--- column
				} // <--- row
				::fclose(fp);
			}

			cmap.fwrite("odometry-correction.cmap");
			//			gnd::odometry::rsemap::write("rse-map", &cmap);
		} // <--- file out

		::fprintf(stderr, "\n");
		::fprintf(stderr, "Finish\n");

	} // <--- finalization

	return 0;
}





