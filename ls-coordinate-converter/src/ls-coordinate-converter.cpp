/*
 * ls-coordinate-converter-conf.hpp
 * laser scanner data coordinate converter
 *
 *  Created on: 2013/05/23
 *      Author: tyamada
 */


#include <stdio.h>

#include "ssm-laser.hpp"
#include <ssmtype/spur-odometry.h>

#include "ls-coordinate-converter-opt.hpp"

#include "gnd-matrix-base.hpp"
#include "gnd-vector-base.hpp"
#include "gnd-matrix-coordinate.hpp"
#include "gnd-timer.hpp"
#include "gnd-shutoff.hpp"
#include "gnd-util.h"


int main(int argc, char* argv[], char* env[]) {
	SSMScanPoint2D						ssm_sokuiki_raw;	// laser scanner raw data
	SSMSOKUIKIData3D					ssm_sokuiki_fs;		// laser scanner data on robot coordinate
	SSMSOKUIKIData3D					ssm_sokuiki_gl;		// laser scanner data on global coordinate
	SSMApi<Spur_Odometry> 				ssm_position;		// position data for global coordinate convert

	gnd::matrix::fixed<4,4> 			cconvert_mat;		// coordinate convert matrix

	gnd::ls_cc::proc_configuration		pconf;	// proccess configuration
	gnd::ls_cc::options					opt_reader(&pconf);

	FILE *fp_gltxtlog = 0;

	{ // ---> initialize
		int ret;

		// ---> read process options
		if( (ret = opt_reader.get_option(argc, argv)) != 0 ) {
			return ret;
		} // <--- read process options

		{ // ---> allocate SIGINT to shut-off
			::proc_shutoff_clear();
			::proc_shutoff_alloc_signal(SIGINT);
		} // <--- allocate SIGINT to shut-off

		{ // ---> show task
			int phase = 1;
			::fprintf(stderr, "========== Initialize ==========\n");
			::fprintf(stderr, " %d. initialize SSM\n", phase++);
			::fprintf(stderr, " %d. open ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.raw_name.value);
			::fprintf(stderr, " %d. create ssm-data \"\x1b[4m%s\x1b[0m\"\n", phase++, pconf.fs_name.value);
			::fprintf(stderr, "\n");
		} // <--- show task


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


		// ---> open sokuiki raw ssmdata
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open sokuiki raw ssm  \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.raw_name.value, pconf.raw_id.value);

			// ---> create log file
			if( !ssm_sokuiki_raw.openWait( pconf.raw_name.value, pconf.raw_id.value, 0.0, SSM_READ ) ){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to open ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.raw_name.value, pconf.raw_id.value);
			}
			// get property
			else if( !ssm_sokuiki_raw.getProperty() ){
				::proc_shutoff();
				::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to get property of \"\x1b[4m%s\x1b[0m\"\n", pconf.raw_name.value);
			}
			else {
				gnd::matrix::fixed<4,4> ws;
				// allocate
				ssm_sokuiki_raw.data.alloc(ssm_sokuiki_raw.property.numPoints);

				ssm_sokuiki_raw.readLast();

				ssm_sokuiki_raw.setBlocking(true);

				gnd::matrix::copy(&cconvert_mat, &ssm_sokuiki_raw.property.coordm);
				::fprintf(stderr, " ... \x1b[1mOK\x1b[0m\n");
			}
		} // <--- open sokuiki raw ssmdata


		// ---> create sokuiki fs ssmdata
		if( !::is_proc_shutoff() ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => create sokuiki fs ssmdata  \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.fs_name.value, (pconf.fs_id.value < 0 ? pconf.raw_id.value : pconf.fs_id.value));

			// allocate
			ssm_sokuiki_fs.data.alloc(ssm_sokuiki_raw.property.numPoints);
			ssm_sokuiki_fs.property = ssm_sokuiki_raw.property;

			// ---> create log file
			if( !ssm_sokuiki_fs.create(pconf.fs_name.value, (pconf.fs_id.value < 0 ? pconf.raw_id.value : pconf.fs_id.value), gnd_time2sec(pconf.fs_storage.value), ssm_sokuiki_raw.property.cycle) ){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to create ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.fs_name.value, pconf.fs_id.value);
			} // <--- create log file
			else {
				ssm_sokuiki_fs.setProperty();
				::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m\n");
			}
		} // <--- create sokuiki fs ssmdata


		// ---> create sokuiki gl ssmdata
		if( !::is_proc_shutoff() && pconf.gl_name.value[0] && pconf.gl_pos_name.value[0] ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => open position ssm, for global coorinate convert \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.gl_pos_name.value, pconf.gl_pos_id.value );


			// ---> open position ssm
			if( !ssm_position.openWait( pconf.gl_pos_name.value, pconf.gl_pos_id.value, 0.0, SSM_READ ) ){
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to open ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.gl_name.value, pconf.gl_id.value);
			}
			// create sokuiki gl
			else {
				::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m\n");

				::fprintf(stderr, "\n");
				::fprintf(stderr, " => create sokuiki gl ssmdata  \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.gl_name.value, (pconf.gl_id.value < 0 ? pconf.raw_id.value : pconf.gl_id.value));

				// allocate
				ssm_sokuiki_gl.data.alloc(ssm_sokuiki_raw.property.numPoints);
				ssm_sokuiki_gl.property = ssm_sokuiki_raw.property;

				// ---> create log file
				if( !ssm_sokuiki_gl.create(pconf.gl_name.value, (pconf.gl_id.value < 0 ? pconf.raw_id.value : pconf.gl_id.value), gnd_time2sec(pconf.gl_storage.value), ssm_sokuiki_raw.property.cycle) ){
					::proc_shutoff();
					::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to create ssm-data \"\x1b[4m%s\x1b[0m\" id %d\n", pconf.gl_name.value, pconf.gl_id.value);
				} // <--- create log file
				else {
					ssm_sokuiki_gl.setProperty();
					::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m\n");
				}
			}

		} // <--- create sokuiki gl ssmdata

		// --->log file open
		if( !::is_proc_shutoff() && pconf.gl_txtlog.value[0] ){
			::fprintf(stderr, "\n");
			::fprintf(stderr, " => scan gl log file \"\x1b[4m%s\x1b[0m\" open \n", pconf.gl_txtlog.value );

			if( ! (fp_gltxtlog = ::fopen(pconf.gl_txtlog.value, "w")) ) {
				::proc_shutoff();
				::fprintf(stderr, "  [\x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m]: fail to open \"\x1b[4m%s\x1b[0m\"\n", pconf.gl_txtlog.value);
			}
			else {
				::fprintf(fp_gltxtlog, "# 1.[x] 2.[y] 3.[z] 4.[intensity] 5.[org-x] 6.[org-y] 7.[org-z]\n");

				::fprintf(stderr, "   ...\x1b[1mOK\x1b[0m\n");
			}

		} // <--- log file open

	} // <--- initialize



	{ // ---> operation
		int nline_show = 0;
		unsigned long cnt_scan = 0;
		gnd::inttimer timer_show(CLOCK_REALTIME, 1.0);

		// ---> main loop
		while( !::is_proc_shutoff() ) {
			// ---> show status
			if( timer_show.clock() ){
				// back cursor
				if( nline_show ) {
					::fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}

				nline_show++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", gnd::ls_cc::proc_name);
				nline_show++; ::fprintf(stderr, "\x1b[K            prod : %s\n", ssm_sokuiki_raw.property.sensorInfo.product );
				nline_show++; ::fprintf(stderr, "\x1b[K          serial : %s\n", ssm_sokuiki_raw.property.sensorInfo.id );
				nline_show++; ::fprintf(stderr, "\x1b[K              fs : %s : id %d\n", pconf.fs_name.value, (pconf.fs_id.value < 0 ? pconf.raw_id.value : pconf.fs_id.value)  );
				nline_show++; ::fprintf(stderr, "\x1b[K      total scan : %ld\n", cnt_scan );
				nline_show++; ::fprintf(stderr, "\x1b[K\n");
				nline_show++; ::fprintf(stderr, "\x1b[K Push \x1b[1mEnter\x1b[0m to change CUI Mode\n");
			} // <--- show status

			// ---> get sokuiki raw and coordinate convert
			if( ssm_sokuiki_raw.readNext() ) {
				cnt_scan++;

				// ---> scanning loop (sokuiki raw)
				for( int i = 0; i < (signed)ssm_sokuiki_raw.data.numPoints(); i++ ) {
					ssm_sokuiki_fs.data[i].status = ssm_sokuiki_raw.data[i].status;

					if( !ssm_sokuiki_fs.data[i].isError() && ssm_sokuiki_fs.data[i].status != ssm::laser::STATUS_NO_REFLECTION) {
						{ // ---> coordinate convert
							gnd::vector::fixed_column<4> point_sns, point_fs;

							point_sns[0] = ssm_sokuiki_raw.data[i].r * ::cos( ssm_sokuiki_raw.data[i].th );
							point_sns[1] = ssm_sokuiki_raw.data[i].r * ::sin( ssm_sokuiki_raw.data[i].th );
							point_sns[2] = 0;
							point_sns[3] = 1;

							gnd::matrix::prod(&cconvert_mat, &point_sns, &point_fs );

							ssm_sokuiki_fs.data[i].reflect.x = point_fs[0];
							ssm_sokuiki_fs.data[i].reflect.y = point_fs[1];
							ssm_sokuiki_fs.data[i].reflect.z = point_fs[2];
						} // <--- coordinate convert

						{ // --->  set value of other kinds
							ssm_sokuiki_fs.data[i].intensity = ssm_sokuiki_raw.data[i].intensity;
							ssm_sokuiki_fs.data[i].origin.x = ssm_sokuiki_raw.property.coordm[0][3];
							ssm_sokuiki_fs.data[i].origin.y = ssm_sokuiki_raw.property.coordm[1][3];
							ssm_sokuiki_fs.data[i].origin.z = ssm_sokuiki_raw.property.coordm[1][3];
						} // <---  set value of other kinds
					}
				} // <--- scanning loop (sokuiki raw)

				// write ssm
				ssm_sokuiki_fs.write( ssm_sokuiki_raw.time );

				// global cooridnate convert
				if( pconf.gl_name.value[0] && pconf.gl_pos_name.value[0] && ssm_position.readTime( ssm_sokuiki_raw.time) ){
					gnd::vector::fixed_column<4> ws, org_gl;
					gnd::coord_matrix cm;

					gnd::matrix::coordinate_converter(&cm,
							ssm_position.data.x, ssm_position.data.y, 0,
							::cos(ssm_position.data.theta), ::sin(ssm_position.data.theta), 0,
							 0, 0, 1);

					{ // origin
						ws[0] = ssm_sokuiki_raw.property.coordm[0][3];
						ws[1]= ssm_sokuiki_raw.property.coordm[1][3];
						ws[2] = ssm_sokuiki_raw.property.coordm[1][3];
						ws[3] = 1.0;

						gnd::matrix::prod(&cm, &ws, &org_gl );
					} // origin



					// ---> scanning loop (sokuiki raw)
					for( int i = 0; i < (signed)ssm_sokuiki_raw.data.numPoints(); i++ ) {
						gnd::vector::fixed_column<4> point_sns, point_gl;

						ssm_sokuiki_gl.data[i].status = ssm_sokuiki_raw.data[i].status;

						if( !ssm_sokuiki_gl.data[i].isError() && ssm_sokuiki_gl.data[i].status != ssm::laser::STATUS_NO_REFLECTION) {
							{ // ---> coordinate convert
								gnd::vector::fixed_column<4> point_sns, point_gl;

								point_sns[0] = ssm_sokuiki_fs.data[i].reflect.x;
								point_sns[1] = ssm_sokuiki_fs.data[i].reflect.y;
								point_sns[2] = ssm_sokuiki_fs.data[i].reflect.z;
								point_sns[3] = 1;

								gnd::matrix::prod(&cm, &point_sns, &point_gl);

								ssm_sokuiki_gl.data[i].reflect.x = point_gl[0];
								ssm_sokuiki_gl.data[i].reflect.y = point_gl[1];
								ssm_sokuiki_gl.data[i].reflect.z = point_gl[2];
							} // <--- coordinate convert

							{ // ---> set value of other kind
								ssm_sokuiki_gl.data[i].intensity = ssm_sokuiki_raw.data[i].intensity;
								ssm_sokuiki_gl.data[i].origin.x = org_gl[0];
								ssm_sokuiki_gl.data[i].origin.y = org_gl[1];
								ssm_sokuiki_gl.data[i].origin.z = org_gl[2];
							} // <--- set value of other kind

							if( fp_gltxtlog ) {
								::fprintf(fp_gltxtlog, "%lf %lf %lf %lf %lf %lf %lf\n",
										ssm_sokuiki_gl.data[i].reflect.x, ssm_sokuiki_gl.data[i].reflect.y, ssm_sokuiki_gl.data[i].reflect.z,
										ssm_sokuiki_gl.data[i].intensity,
										ssm_sokuiki_gl.data[i].origin.x, ssm_sokuiki_gl.data[i].origin.y, ssm_sokuiki_gl.data[i].origin.z);
							}
						}
					} // <--- scanning loop (sokuiki raw)
					// write ssm
					ssm_sokuiki_gl.write( ssm_sokuiki_raw.time );

				} // global cooridnate convert


			} // <--- get sokuiki raw and coordinate convert

		} // <--- main loop

	} // <--- operation



	{ // ---> finalize

		{ // ---> finalize ssm
			if( ssm_sokuiki_raw.isOpen() )	ssm_sokuiki_raw.close() ;
			if( ssm_sokuiki_fs.isOpen() )	ssm_sokuiki_fs.close();
			::endSSM();
		} // <--- finalize ssm

		if( fp_gltxtlog ) ::fclose(fp_gltxtlog);

	} // <--- finalize


	return 0;
}
