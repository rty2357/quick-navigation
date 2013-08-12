//============================================================================
// Name        : opsm-particle-evaluator.cpp
// Author      : tyamada
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdint.h>

#include <ssm-laser.hpp>
#include <ssmtype/spur-odometry.h>
#include <ssm.hpp>
#include "ssm-opsm-map.hpp"
#include "ssm-particles.hpp"

#include "opsm-particle-evaluator-cui.hpp"
#include "opsm-particle-evaluator-opt.hpp"
#include "opsm-particle-evaluator-conf.hpp"

#include "gnd-util.h"
#include "gnd-timer.hpp"
#include "gnd-shutoff.hpp"
#include "gnd-bmp.hpp"
#include "gnd-gridmap.hpp"
#include "gnd-coord-tree.hpp"
#include "gnd-shutoff.hpp"

int main(int argc, char *argv[], char **env) {
	gnd::opsm::map_t 		opsm_map;
	gnd::bmp32_t			map;			// map

	SSMApi<Spur_Odometry>	ssm_odometry;	//
	SSMScanPoint2D			ssm_sokuikiraw;	// ssm sokuiki raw data
	SSMParticles			ssm_particles;	// ssm particles
	SSMParticleEvaluation	ssm_evaluation;	// ssm evaluation

	gnd::matrix::coord_tree coordtree;
	int coordid_gl = -1,
			coordid_rt = -1,
			coordid_sns = -1;


	opsm::peval::proc_configuration pconf;	// process configuration
	opsm::peval::proc_option_reader popt;	// process option
	gnd::cui_reader					pcui;	// cui


	{ // ---> initialize
		int ret;
		uint32_t phase = 1;

		// ---> read option
		popt.set(&pconf);
		if( (ret = popt.read(argc, argv)) != 0){
			return ret;
		} // <--- read option


		{ // ---> allocate SIGINT to shutoff
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shutoff


		// ---> show initialize sequence
		if( !is_proc_shutoff() ) {
			::fprintf(stderr, "==========Initialize==========\n");
			::fprintf(stderr, " %d. Map Data Load\n", phase++);
			::fprintf(stderr, " %d. Initailize SSM\n", phase++);
			::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.sokuikiraw_name.value);
			if( pconf.sleeping_time.value > 0)
				::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.odometry_name.value);
			::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.particle_name.value);
			::fprintf(stderr, " %d. Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.eval_name.value);
			::fprintf(stderr, " %d. set property of ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.eval_name.value);
			::fprintf(stderr, " %d. Create ssm-data \"%s\"\n", phase++, SNAME_OPSM_MAP);
			::fprintf(stderr, "\n\n");
		} // <--- show initialize sequence


		// ---> read map raw data
		if( !is_proc_shutoff() && pconf.raw_map.value[0] != '\0' ) {
			gnd::opsm::cmap_t cnt_map;			// counting map
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Raw Map Data Reading\n");

			::fprintf(stderr, "   map file is \"\x1b[4m%s\x1b[0m\"\n", pconf.raw_map.value);
			if( gnd::opsm::read_counting_map(&cnt_map, pconf.raw_map.value) < 0){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to read map data\n");
			}
			else {
				if( gnd::opsm::build_map(&opsm_map, &cnt_map, gnd_mm2dist(1)) < 0 ) {
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build map\n");
				}
				else if( gnd::opsm::build_bmp32(&map, &opsm_map, gnd_m2dist( 1.0 / 20)) < 0 ) {
					::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to convert bmp\n");
				}
				else {
					::fprintf(stderr, " ...\x1b[1mOK\x1b[0m\n");
				}
			}
		} // <--- read map raw data


		// ---> ssm initlaize
		if( !::is_proc_shutoff() ) {
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => Initialize SSM\n");
			if( !::initSSM() ){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to initialize SSM\n");
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: initialize SSM\n");
			}
		} // ---> ssm initlaize

		// ---> write map info for displaying the map
		if( !::is_proc_shutoff() ){
			SSMOPSMMap				ssm_map;		// ssm map (dummy)
			gnd::bmp8_t				bmp8;
			char path[256];

			// build bmp 8bit map
			gnd::opsm::build_bmp( &bmp8, &opsm_map, gnd_m2dist(1.0/10) );

			// write 8bit map file
			gnd_get_working_directory(env, path, sizeof(path));
			::strcat(path, "/view-map.bmp");
			gnd::bmp::write( path, &bmp8 );

			{ // ---> set parameter
				::strcpy( ssm_map.property.fname, path);
				// cooridnate
				ssm_map.property.point[0].x = bmp8.xupper();
				ssm_map.property.point[0].y = bmp8.ylower();

				ssm_map.property.point[1].x = bmp8.xlower();
				ssm_map.property.point[1].y = bmp8.ylower();

				ssm_map.property.point[2].x = bmp8.xlower();
				ssm_map.property.point[2].y = bmp8.yupper();

				ssm_map.property.point[3].x = bmp8.xupper();
				ssm_map.property.point[3].y = bmp8.yupper();

				ssm_map.property.scale = bmp8.xrsl();

				ssm_map.property.offset = 0;

			} // <--- set parameter

			if(!ssm_map.create(SNAME_OPSM_MAP, 0, 1, 1)){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to open ssm \"%s\"\n", SNAME_OPSM_MAP);
				::proc_shutoff();
			}
			else if(!ssm_map.setProperty() ){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to set property ssm \"%s\"\n", SNAME_OPSM_MAP);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"%s\"\n", SNAME_OPSM_MAP);
			}
		} // <--- write map info for displaying the map


		// ---> sokuiki data ssm open
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, " => Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.sokuikiraw_name.value);
			if( !ssm_sokuikiraw.openWait(pconf.sokuikiraw_name.value, pconf.sokuikiraw_id.value, 0.0)){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", pconf.sokuikiraw_name.value);
				::proc_shutoff();
			}
			// get property
			else if( !ssm_sokuikiraw.getProperty() ){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to get the property of \"\x1b[4m%s\x1b[0m\"\n", pconf.sokuikiraw_name.value);
				::proc_shutoff();
			}
			else {
				// allocate
				ssm_sokuikiraw.data.alloc(ssm_sokuikiraw.property.numPoints);
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.sokuikiraw_name.value);
			}
		} // <--- sokuiki data ssm open


		// ---> open odometry ssm data
		if( !::is_proc_shutoff() && *pconf.odometry_name.value && pconf.odometry_id.value >= 0 ){
			::fprintf(stderr, " => Open ssm-data \"%s\"\n", pconf.odometry_name.value);
			if(!ssm_odometry.openWait(pconf.odometry_name.value, pconf.odometry_id.value, 0.0)){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", pconf.odometry_name.value);
			}
			else {
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.odometry_name.value);
				ssm_odometry.readLast();
			}
		} // ---> open odometry ssm data

		// ---> particle data ssm open
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, " => Open ssm-data \"%s\"\n", pconf.particle_name.value );
			if(!ssm_particles.openWait( pconf.particle_name.value, pconf.particle_id.value, 0.0)){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", pconf.particle_name.value);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.particle_name.value);
			}
		} // <--- particle data ssm open


		// ---> particle evaluation data ssm open
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, " => Open ssm-data \"\x1b[4m%s\x1b[0m\"\n", pconf.eval_name.value);
			if(!ssm_evaluation.openWait(pconf.eval_name.value, pconf.eval_id.value, 0.0)){
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: Fail to ssm open \"\x1b[4m%s\x1b[0m\"\n", pconf.eval_name.value);
				::proc_shutoff();
			}
			else {
				::fprintf(stderr, "  [\x1b[1mOK\x1b[0m]: Open ssm-data \"%s\"\n", pconf.eval_name.value);
			}
		} // <--- particle  evaluation data ssm open


		{ // ---> coord tree
			gnd::coord_matrix cm;

			// define global coordinate
			gnd::matrix::set_unit(&cm);
			coordid_gl = coordtree.add("global", "root", &cm);

			// init robot coordinate
			gnd::matrix::set_unit(&cm);
			coordid_rt = coordtree.add("robot", "global", &cm);

			// define sensor coordinate
			coordid_sns = coordtree.add("sensor", "robot", &ssm_sokuikiraw.property.coordm);
		} // <--- coord tree


		// ---> initialize cui
		if( !::is_proc_shutoff() ){
			pcui.set_command(opsm::peval::cui_cmd, sizeof(opsm::peval::cui_cmd) / sizeof(opsm::peval::cui_cmd[0]));
		} // ---> initialize cui

	} // <--- initialize



	if( !::is_proc_shutoff() ){ // ---> operation
		Spur_Odometry prev;				// previous position
		uint32_t nalloc = 0;
		double sleep_time;
		int cnt_eval = 0;
		int nline_show = 0;
		uint32_t i;

		gnd::inttimer timer_clock;
		gnd::inttimer timer_operate;
		gnd::inttimer timer_show;
		gnd::inttimer timer_sleeping;

		double cuito = 0;

		{ // ---> initialize previoous position
			if( ssm_odometry.isOpen() )		prev = ssm_odometry.data;
			else							memset(&prev, 0, sizeof(prev));
		} // <--- initialize previous position

		{ // ---> initialize timer
			timer_operate.begin(CLOCK_REALTIME, pconf.cycle.value, -pconf.cycle.value);
			timer_clock.begin(CLOCK_REALTIME,
					pconf.cycle.value / 2.0 < opsm::peval::Frame ? pconf.cycle.value / 2.0 : opsm::peval::Frame);
			timer_show.begin(CLOCK_REALTIME, opsm::peval::ShowUpdateCycle, -opsm::peval::ShowUpdateCycle);
			if( pconf.sleeping_time.value > 0)
				timer_sleeping.begin(CLOCK_REALTIME, pconf.sleeping_time.value, -pconf.sleeping_time.value);
		} // <--- initialize timer

		// ---> operation loop
		while( !::is_proc_shutoff() ) {
			timer_clock.wait();

			// get current time
			{ // ---> cui
				int cuival = 0;
				char cuiarg[512];

				::memset(cuiarg, 0, sizeof(cuiarg));

				// ---> get command
				if( pcui.poll(&cuival, cuiarg, sizeof(cuiarg), cuito) > 0 ){
					if( timer_show.cycle() > 0){
						// quit show status mode
						timer_show.end();
						::fprintf(stderr, "-------------------- cui mode --------------------\n");
					}
					else { // ---> cui command operation
						switch(cuival){
						// exit
						case 'Q': ::proc_shutoff(); break;
						// help
						default:
						case '\0':
						case 'h': pcui.show(stderr, "   "); break;
						// show status
						case 's': timer_show.begin(CLOCK_REALTIME, opsm::peval::ShowUpdateCycle, -opsm::peval::ShowUpdateCycle); break;
						case 'f': {
							double freq = ::strtod(cuiarg, 0);
							if( freq <= 0 ){
								::fprintf(stderr, "   ... \x1b[31m\x1b[1mError\x1b[0m\x1b[39m: invalid argument value (frequency 0)\n");
								::fprintf(stderr, "       if you want to stop estimator, send \"\x1b[4mstand-by\x1b[0m\" command\n");
							}
							else {
								double cyc = 1.0 / freq;
								timer_operate.begin(CLOCK_REALTIME, cyc);
								timer_clock.begin(CLOCK_REALTIME,
										cyc < opsm::peval::Frame ? cyc : opsm::peval::Frame);
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
								timer_operate.begin(CLOCK_REALTIME, cyc);
								timer_clock.begin(CLOCK_REALTIME,
										cyc < opsm::peval::Frame ? cyc : opsm::peval::Frame);
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
					pcui.poll(&cuival, cuiarg, sizeof( cuiarg ), 0);
				} // <--- get command
			}  // <--- cui


			// ---> show status
			if( timer_show.clock() > 0){
				// back cursor
				if( nline_show ) {
					::fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}

				nline_show++;	::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", opsm::peval::ProcName);
				nline_show++;	::fprintf(stderr, "\x1b[K       count : %d\n", cnt_eval );
				nline_show++;	::fprintf(stderr, "\x1b[K       cycle : %lf [s]\n", timer_operate.cycle() );
				nline_show++;	::fprintf(stderr, "\x1b[K        prev : %lf %lf, %lf\n", prev.x, prev.y, gnd_ang2deg(prev.theta) );
				nline_show++;	::fprintf(stderr, "\x1b[K       sleep : %lf [s]\n", sleep_time );
				//				::fprintf(stderr, "     average : %.03lf\n", est_ave );
				//				::fprintf(stderr, "   max - min : max %.03lf, min %.03lf\n", est_max, est_min );
				//				::fprintf(stderr, "perform eval : %.03lf\n", perform);
				//				::fprintf(stderr, " fail weight : %.03lf\n", fault_weight );
				//				::fprintf(stderr, "   rest-mode : %s\n", ssm_position.isOpen() ? "on" : "off"  );
				nline_show++;	::fprintf(stderr, "\x1b[K\n");
				nline_show++;	::fprintf(stderr, "\x1b[K Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
			} // <--- show status


			// ---> particle evaluation with laser-scanner reading
			if( timer_operate.clock() > 0 && ssm_sokuikiraw.readNew()) {
				// ---> check sleeping mode
				if( ssm_odometry.isOpen() ) {
					if( timer_sleeping.clock(&sleep_time) <= 0 ) {
						double sqdist = 0,
								diff_orient = 0;
						if( !ssm_odometry.readTime( ssm_sokuikiraw.time) ){
							continue;
						}
						// check moving distance
						sqdist = gnd_square( ssm_odometry.data.x - prev.x ) + gnd_square( ssm_odometry.data.y - prev.y );
						diff_orient = ::fabs( ssm_odometry.data.theta - prev.theta);
						if( sqdist < gnd_square( pconf.sleeping_dist.value) && diff_orient < pconf.sleeping_orient.value){
							continue;
						}
					}
					else {
					}
					// set sleeping timer
					timer_sleeping.begin(CLOCK_REALTIME, pconf.sleeping_time.value);
					prev = ssm_odometry.data;
				} // <--- check sleeping mode


				{ // ---> get particles
					if( !ssm_particles.readTime( ssm_sokuikiraw.time ) ) continue;

					ssm_evaluation.data.n = ssm_particles.data.size();

					// ---> reallocate
					if( nalloc < ssm_evaluation.data.n){
						if(!ssm_evaluation.data.value)	delete[] ssm_evaluation.data.value;
						ssm_evaluation.data.value = new double[ssm_evaluation.data.n];
						nalloc = ssm_evaluation.data.n;
					}// ---> reallocate

				} // <--- get particles



				// ---> scanning loop (particle)
				for( i = 0 ; i <  ssm_particles.data.size(); i++ ) {
					gnd::matrix::fixed< 1, PARTICLE_DIM >* p = ssm_particles.data.begin() + i;
					gnd::matrix::fixed<4, 1> prev_csns;

					gnd::matrix::set_zero(&prev_csns);
					{ // ---> set particle coordinate
						gnd::coord_matrix cm;

						gnd::matrix::coordinate_converter(&cm,
								(*p)[0][PARTICLE_X], (*p)[0][PARTICLE_Y], 0,
								::cos((*p)[0][PARTICLE_THETA]), ::sin((*p)[0][PARTICLE_THETA]), 0,
								 0, 0, 1);

						coordtree.set_coordinate(coordid_rt, &cm);

					} // <--- set particle coordinate



					{ // ---> particle evaluation with laser scanner reading
						double eval = 0;
						int cnt = 0;
						double sqdist = 0;
						gnd::matrix::fixed<4, 1> pos_sns;
						gnd::matrix::fixed<4, 1> pos_gl;
						gnd::matrix::fixed<4, 1> prev_pos_sns;
						gnd::coord_matrix cm_sn2gl;

						gnd::matrix::set_zero(&prev_pos_sns);
						prev_pos_sns[3][0] = 1;

						coordtree.get_convert_matrix(coordid_sns, coordid_gl, &cm_sn2gl);

						// ---> scanning loop (sokuiki data)
						for( int j = 0; j < (signed)ssm_sokuikiraw.data.numPoints(); j++ ) {
							double cosv, sinv;

							if(ssm_sokuikiraw.data[j].status == ssm::laser::STATUS_NO_REFLECTION)	continue;
							else if( ssm_sokuikiraw.data[j].isError() ) continue;
							else if( ssm_sokuikiraw.data[j].r < 0.3 ) continue;

							cosv = ::cos(ssm_sokuikiraw.data[j].th);
							sinv = ::sin(ssm_sokuikiraw.data[j].th);

							// set search position on sensor-coordinate
							pos_sns[0][0] = ssm_sokuikiraw.data[j].r * cosv;
							pos_sns[1][0] = ssm_sokuikiraw.data[j].r * sinv;
							pos_sns[2][0] = 0;
							pos_sns[3][0] = 1;

							sqdist = gnd_square(pos_sns[0][0] - prev_pos_sns[0][0]) + gnd_square(pos_sns[1][0] - prev_pos_sns[1][0]);
							if( sqdist <= gnd_square( pconf.cull.value ) )	continue;

							// coordinate convert from sensor coordinate to global coordinate
							gnd::matrix::prod(&cm_sn2gl, &pos_sns, &pos_gl);


							if( map.ppointer(pos_gl[0][0], pos_gl[1][0]) ){
								eval += (double) map.pvalue(pos_gl[0][0], pos_gl[1][0]);
							}
							cnt++;
						} // <--- scanning loop (sokuiki data)

						// normalization
						if(eval < 0 || cnt <= 0)	eval = 0;
						else 						eval = (eval / (double) (0x8000 * cnt )) * (1.0 - pconf.mfailure.value) + pconf.mfailure.value;

						ssm_evaluation.data.value[i] = eval;

					} // <--- particle evaluation with laser scanner reading

				} // <--- scanning loop (particle)
				ssm_evaluation.write( ssm_sokuikiraw.time );
				cnt_eval++;


			} // <--- particle evaluation with laser-scanner reading


		} // <--- operation loop

	} // <--- operation




	{ // ---> finalize
		::endSSM();

		::fprintf(stdout, "\n\n");
		::fprintf(stdout, "...Finish\n");
	} // <--- finalize

	return 0;
}
