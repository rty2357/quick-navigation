/*
 * matrix-coordinate-convert.hpp
 *
 *  Created on: 2011/07/03
 *      Author: tyamada
 */

#ifndef GND_MATRIX_COORDINATE_CONVERT_HPP_
#define GND_MATRIX_COORDINATE_CONVERT_HPP_

#include "gnd-lib-error.h"
#include "gnd-matrix-base.hpp"

#include <math.h>

/**
 * @ifnot GNDStorage
 * @defgroup GNDStorage storage
 * @endif
 */
/**
 * @ifnot GNDMatrix
 * @defgroup GNDMatrix matrix
 * supply matrix operation
 * @endif
 */

namespace gnd { // ---> namespace gnd
	namespace matrix { // ---> namespace matrix

		double __coordinate_matrix_inner_product__(
				const double x1, const double y1, const double z1,
				const double x2, const double y2, const double z2 )
		{
			return x1*x2 + y1*y2 + z1*z2;
		}


		inline
		int coordinate_convert( coord_matrix *mt, pos_vector *v1, pos_vector *v2) {
			gnd_assert(!mt, -1, "invalid null pointer");
			gnd_assert(!v1, -1, "invalid null pointer");
			gnd_assert(!v2, -1, "invalid null pointer");

			{
				fixed<4,1> ws1, ws2;

				submatrix_copy(&ws1, 0, 0, v1, 0, 0, 3, 1);
				ws1[3][0] = 1;
				prod(mt, &ws1, &ws2 );
				submatrix_copy(v2, 0, 0, &ws2, 0, 0, 3, 1);
			}
			return 0;
		}



		/**
		 * @brief compute coordinate convert matrix
		 * @param[out]  mt : cooridnate convert matix
		 * @param[in]    x : origin x position
		 * @param[in]    y : origin y position
		 * @param[in]    z : origin z position
		 * @param[in] xaxx : x element of x-axis unit vector
		 * @param[in] xaxy : y element of x-axis unit vector
		 * @param[in] xaxz : z element of x-axis unit vector
		 * @param[in] zaxx : x element of z-axis unit vector
		 * @param[in] zaxy : y element of z-axis unit vector
		 * @param[in] zaxz : z element of z-axis unit vector
		 */
		template< typename MTRX >
		int coordinate_converter( MTRX *mt,
				const double x, const double y, const double z,
				const double xaxx, const double xaxy, const double xaxz,
				const double zaxx, const double zaxy, const double zaxz)
		{
			static const double error_margin = 1.0e-8;
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert( gnd::matrix::row(mt) < 4, -1, "invalid matrix property");
			gnd_assert( gnd::matrix::column(mt) < 4, -1, "invalid matrix property");

			{ // ---> operation
				double xx, xy, xz;
				double yx, yy, yz;
				double zx, zy, zz;

				{ // ---> x-axis
					double norm =
							__coordinate_matrix_inner_product__( xaxx, xaxy, xaxz, xaxx, xaxy, xaxz );

					// normalization
					gnd_error(norm == 0, -1, "x-axis is 0 vector");
					if(1.0 - norm < 1.0e-8){
						xx = xaxx;
						xy = xaxy;
						xz = xaxz;
					}
					else {
						norm = ::sqrt(norm);
						xx = xaxx / norm;
						xy = xaxy / norm;
						xz = xaxz / norm;
					}
				} // <--- x-axis


				{ // ---> z-axis
					double inner_prod;
					double norm;

					inner_prod =
							__coordinate_matrix_inner_product__( xaxx, xaxy, xaxz, zaxx, zaxy, zaxz );

					gnd_warnning(fabs(inner_prod) > error_margin, "input z-axis not a vertical axis of x" );
					if(::fabs(inner_prod) > error_margin){
						zx = zaxz;
						zy = zaxy;
						zz = zaxz;
					}
					else {
						zx = zaxx - xaxx * inner_prod;
						zy = zaxy - xaxy * inner_prod;
						zz = zaxz - xaxz * inner_prod;
					}

					// normalization
					norm = __coordinate_matrix_inner_product__( zx, zy, zz, zx, zy, zz );
					gnd_error(norm == 0, -1, "z-axis is 0 vector");
					if(1.0 - norm >= 1.0e-8){
						norm = ::sqrt(norm);
						zx = zaxz / norm;
						zy = zaxy / norm;
						zz = zaxz / norm;
					}
				} // <--- z-axis


				{ // ---> y-axis
					// external product ZxX
					yx = zy * xz - zz * xy;
					yy = zz * xx - zx * xz;
					yz = zx * xy - zy * xx;
				} // <--- y-axis

				{ // ---> integrate
					gnd::matrix::set(mt, 0, 0, xx);
					gnd::matrix::set(mt, 1, 0, xy);
					gnd::matrix::set(mt, 2, 0, xz);
					gnd::matrix::set(mt, 3, 0, 0);

					gnd::matrix::set(mt, 0, 1, yx);
					gnd::matrix::set(mt, 1, 1, yy);
					gnd::matrix::set(mt, 2, 1, yz);
					gnd::matrix::set(mt, 3, 1, 0);

					gnd::matrix::set(mt, 0, 2, zx);
					gnd::matrix::set(mt, 1, 2, zy);
					gnd::matrix::set(mt, 2, 2, zz);
					gnd::matrix::set(mt, 3, 2, 0);

					gnd::matrix::set(mt, 0, 3, x);
					gnd::matrix::set(mt, 1, 3, y);
					gnd::matrix::set(mt, 2, 3, z);
					gnd::matrix::set(mt, 3, 3, 1);
				} // <--- integrate
			} // <--- operation
			return 0;
		}
	} // <--- namespace matrix
} // <--- namespace gnd



#endif /* YP_MATRIX_COORDINATE_CONVERT_HPP_ */
