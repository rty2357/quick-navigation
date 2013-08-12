/*
 * config-file-read.cpp
 *
 *  Created on: 2013/06/16
 *      Author: tyamada
 */

#include <stdio.h>
#include "gnd-config-file.hpp"


int main( int argc, char* argv[] ) {
	int ret;
	gnd::conf::file_stream fs;

	// character string
	gnd::conf::parameter<int> int_item = {
		"item-int", 	// item name
	};

	// character string
	gnd::conf::parameter_array<char, 64> str_item = {
		"item-string", 		// item name
	};

	// double array
	gnd::conf::parameter_array<double, 3> dblarray_item = {
		"item-double-array",// item name
	};

	// bool
	gnd::conf::parameter<bool> bool_item;
	strncpy( bool_item.item, "item-bool", gnd::conf::ItemBufferSize );	// item name

	// read from file
	ret = fs.read("configfile");
	if( ret < 0 ) {
		fprintf(stderr, "file to read \"%s\"\n", "configfile");
		return 1;
	}

	// get each item's value
	// * seek item name and get value
	gnd::conf::get_parameter( &fs, &int_item );
	gnd::conf::get_parameter( &fs, &str_item );
	gnd::conf::get_parameter( &fs, &dblarray_item );
	gnd::conf::get_parameter( &fs, &bool_item );

	// show
	fprintf(stdout, "%s = %d\n", int_item.item, int_item.value);
	fprintf(stdout, "%s = %s\n", str_item.item, str_item.value);
	fprintf(stdout, "%s = %lf, %lf, %lf\n", dblarray_item.item, dblarray_item.value[0], dblarray_item.value[1], dblarray_item.value[2]);
	fprintf(stdout, "%s = %d\n", bool_item.item, bool_item.value);

	return 0;
}




