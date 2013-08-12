/*
 * likelihood.cpp
 *
 *  Created on: 2013/06/11
 *      Author: tyamada
 */

#include <stdio.h>
#include "gnd-opsm.hpp"
#include "gnd-config-file.hpp"



int main( int argc, char* argv[] ) {
	gnd::opsm::cmap_t	cmap;					// counting map
	gnd::opsm::map_t		map;					// scan matching map
	FILE				*fp;					// laser scanner data file

	gnd::conf::parameter_array<char, 256> cmap_dir = {
			"cmap-dir", 							// item name
			"./", 									// default value
			"cmap directory path" 					// comment
		};
	gnd::conf::parameter_array<double, 3> pos_ini = {
			"init-pos", 							// item name
			{0, 0, 0}, 								// default value
			"initial position for scan matching (x[m], y[m], theta[deg])" 	// comment
		};
	gnd::conf::parameter_array<char, 256> scan_file = {
			"scan-data",							// item name
			"",										// default value
			"scan data file path"					// comment
		};


	{ // ---> initialize
		int ret;
		gnd::conf::file_stream fs;

		// set parameter item and default parameter
		if( argc < 2 ) {
			static const char fname[] = "_scan-matching.conf";
			::fprintf(stderr, " error: missing file operand\n");
			::fprintf(stderr, "        please input configuration file path\n");
			::fprintf(stderr, "        e.g.: %s scan-matching.conf\n", argv[0]);
			::fprintf(stderr, "        ... \n");
			// write configuration file
			gnd::conf::set_parameter(&fs, &cmap_dir);
			gnd::conf::set_parameter(&fs, &pos_ini);
			gnd::conf::set_parameter(&fs, &scan_file);
			fs.write(fname);
			::fprintf(stderr, "         => generate configuration file template \"%s\"\n", fname);

			return -1;
		}

		// configuration file read
		ret = fs.read(argv[1]);
		if( ret < 0 ) {
			::fprintf(stderr, " error: fail to read configuration file\n");
			return -1;
		}

		// get parameter value
		gnd::conf::get_parameter(&fs, &cmap_dir);
		gnd::conf::get_parameter(&fs, &pos_ini);
		gnd::conf::get_parameter(&fs, &scan_file);

		if( !cmap_dir.value[0] ) {
			::fprintf(stdout, " error: missing counting map\n");
			::fprintf(stdout, " please set counting map directory\n");
			return -1;
		}
		// read counting map
		ret = gnd::opsm::read_counting_map( &cmap, cmap_dir.value );
		if( ret < 0 ) {
			::fprintf(stderr, " error: fail to read counting map form directory \"%s\"\n", cmap_dir.value);
		}
		// build scan matching map
		ret = gnd::opsm::build_map( &map, &cmap );

		if( !scan_file.value[0] ) {
			::fprintf(stdout, " error: missing scan data\n");
			::fprintf(stdout, "        please set scan data file\n");
			return -1;
		}
		// file open scan data file
		fp = ::fopen( scan_file.value, "r" );
		if( !fp ) {
			::fprintf(stdout, " error: fail to open \"%s\"\n", scan_file.value);
			return -1;
		}

	} // <--- initialize



	{ // ---> operation
		int ret;
		double orientation = pos_ini.value[2] * M_PI / 180;
		double cosv = ::cos(orientation);
		double sinv = ::sin(orientation);
		double likelihood = 0;
		int cnt = 0;

		// ---> set scan data
		while( !::feof(fp)  ) {
			double x_s, y_s;
			double x_g, y_g;
			double l;

			// read data file
			ret = ::fscanf(fp, "%lf %lf\n", &x_s, &y_s);
			// when read all data, break
			if( ret < 0 ) break;

			// coordinate convert
			x_g = x_s * cosv - y_s * sinv + pos_ini.value[0];
			y_g = x_s * sinv + y_s * cosv + pos_ini.value[1];

			// likelihood
			gnd::opsm::likelihood(&map, x_g, y_g, &l);
			likelihood += l;

			// count
			cnt++;
		} // ---> set scan data

		if( cnt )	likelihood /= cnt;

		::fprintf(stdout, "likelihood = %lf\n", likelihood);
	} // <--- operation


	{ // ---> finalize
		// file close
		::fclose(fp);

		//destroy map data
		gnd::opsm::destroy_counting_map(&cmap);
		gnd::opsm::destroy_map(&map);
	} // <--- finalize


	return 0;
}



