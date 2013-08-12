/*
 * vector.cpp
 *
 *  Created on: 2013/06/19
 *      Author: tyamada
 */

#include "gnd-vector-base.hpp"


int main( int argc, char* argv[] ) {
	gnd::vector::fixed_column<3>	cv;
	gnd::vector::fixed_row<3> 		rv;

	{ // ---> set_uniform
		::fprintf(stdout, "set_unit\n");
		gnd::matrix::set_uniform(&cv, 1);
		gnd::matrix::show(stdout, &cv);
		::fprintf(stdout, "\n");
	} // <--- set_uniform


	{ // ---> set_zero
		::fprintf(stdout, "set_zero\n");
		gnd::matrix::set_zero(&rv);
		gnd::matrix::show(stdout, &rv);
		::fprintf(stdout, "\n");
	} // <--- set_zero


	{ // ---> indexer
		::fprintf(stdout, "indexer\n");
		for( uint64_t i = 0; i < rv.column; i++ ){
			rv[i] = (double)i;
			::fprintf(stdout, "%lf ", rv[i]);
		}
		::fprintf(stdout, "\n");
		::fprintf(stdout, "\n");
	} // <--- indexer


	{ // ---> prod (1)
		gnd::matrix::fixed<1,1> tmp;

		::fprintf(stdout, "prod (1)\n");

		gnd::matrix::prod(&rv, &cv, &tmp);
		gnd::matrix::show(stdout, &tmp);
		::fprintf(stdout, "\n");
	} // <--- prod (1)


	{ // ---> prod (2)
		gnd::matrix::fixed<3,3> tmp;

		::fprintf(stdout, "prod (2)\n");

		gnd::matrix::prod(&cv, &rv, &tmp);
		gnd::matrix::show(stdout, &tmp);
		::fprintf(stdout, "\n");
	} // <--- prod (2)

	return 0;
}


