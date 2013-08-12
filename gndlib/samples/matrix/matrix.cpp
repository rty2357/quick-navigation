/*
 * sample.cpp
 *
 *  Created on: 2013/06/19
 *      Author: tyamada
 */

#include "gnd-matrix-base.hpp"


int main( int argc, char* argv[] ) {
	gnd::matrix::fixed<3,3> A3x3, B3x3;

	{ // ---> set_unit
		::fprintf(stdout, "set_unit\n");
		gnd::matrix::set_unit(&A3x3);
		gnd::matrix::show(stdout, &A3x3);
		::fprintf(stdout, "\n");
	} // <--- set_unit

	{ // ---> scalar_prod
		::fprintf(stdout, "scalar_prod\n");
		gnd::matrix::scalar_prod(&A3x3, 2.0, &A3x3);
		gnd::matrix::show(stdout, &A3x3);
		::fprintf(stdout, "\n");
	} // <--- scalar_prod

	{ // ---> scalar_div
		::fprintf(stdout, "scalar_div\n");
		gnd::matrix::scalar_div(&A3x3, 2.0, &A3x3);
		gnd::matrix::show(stdout, &A3x3);
		::fprintf(stdout, "\n");
	} // <--- scalar_div

	{ // ---> copy
		::fprintf(stdout, "copy\n");
		gnd::matrix::copy(&B3x3, &A3x3);
		gnd::matrix::show(stdout, &B3x3);
		::fprintf(stdout, "\n");
	} // <--- copy

	{ // ---> matrix indexer
		int cnt = 0;

		::fprintf(stdout, "indexer\n");
		for( int r = 0; r < A3x3.row; r++ ){
			for( int c = 0; c < A3x3.column; c++ ){
				A3x3[r][c] = (double) cnt++;
				::fprintf(stdout, "%lf ", A3x3[r][c] );
			}
			::fprintf(stdout, "\n");
		}
		::fprintf(stdout, "\n");
	} // <--- matrix indexer

	{ // ---> add
		::fprintf(stdout, "add\n");
		gnd::matrix::add(&A3x3, &B3x3, &A3x3);
		gnd::matrix::show(stdout, &A3x3);
		::fprintf(stdout, "\n");
	} // <--- add

	{ // ---> sub
		::fprintf(stdout, "sub\n");
		gnd::matrix::sub(&A3x3, &B3x3, &A3x3);
		gnd::matrix::show(stdout, &A3x3);
		::fprintf(stdout, "\n");
	} // <--- sub

	{ // ---> set_zero
		::fprintf(stdout, "set_zero\n");
		gnd::matrix::set_zero(&B3x3);
		gnd::matrix::show(stdout, &B3x3);
		::fprintf(stdout, "\n");
	} // <--- set_zero

	{ // ---> prod
		gnd::matrix::fixed<3,3> C3x3;

		B3x3[0][0] = 1;
		::fprintf(stdout, "prod\n");
		gnd::matrix::prod(&A3x3, &B3x3, &C3x3);
		gnd::matrix::show(stdout, &C3x3);
		::fprintf(stdout, "\n");
	} // <--- prod



	return 0;
}

