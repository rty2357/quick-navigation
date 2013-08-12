/*
 * config-file-write.cpp
 *
 *  Created on: 2013/06/16
 *      Author: tyamada
 */

#include "gnd-config-file.hpp"

#include <stdio.h>

int main( int argc, char* argv[] ) {
	gnd::conf::file_stream fs;

	// character string
	gnd::conf::parameter<int> int_item = {
		"item-int", 	// item name
		0,				// default value
		"integer parameter"		// comment
	};

	// character string
	gnd::conf::parameter_array<char, 64> str_item = {
		"item-string", 		// item name
		"string",			// default value
		"string parameter"	// comment
	};

	// double array
	gnd::conf::parameter_array<double, 3> dblarray_item = {
		"item-double-array",// item name
		{1.0, 2.0, 3.0},	// default value
		"double array"	// comment
	};

	// bool
	gnd::conf::parameter<bool> bool_item;
	strncpy( bool_item.item, "item-bool", gnd::conf::ItemBufferSize );	// item name
	bool_item.value = true;												// value
	strncpy( bool_item.comment, "", gnd::conf::CommentBufferSize );		// comment (no comment)

	// set
	gnd::conf::set_parameter( &fs, &int_item );
	gnd::conf::set_parameter( &fs, &str_item );
	gnd::conf::set_parameter( &fs, &dblarray_item );
	gnd::conf::set_parameter( &fs, &bool_item );

	// write
	::fprintf(stdout, " write \"configfile\"\n");
	fs.write("configfile");

	return 0;
}

