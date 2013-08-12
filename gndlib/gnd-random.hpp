/*
 * gnd_random.hpp
 *
 *  Created on: 2011/06/23
 *      Author: tyamada
 */

#ifndef GND_RANDOM_HPP_
#define GND_RANDOM_HPP_


#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include "gnd-matrix-base.hpp"
#include "gnd-vector-base.hpp"

#define __gnd_random_set_seed__()	(srand(time(0)))
#define __gnd_random_uniform__()	((double)rand() / RAND_MAX)

// ---> function declaration
namespace gnd {
	inline
	void random_set_seed( void );

	/**
	 * @brief generate randam value [0;1] with uniform probability
	 */
	inline
	double random_uniform( void );

	/*
	 * @brief generate random value following a gaussian distribution (box-muller method)
	 */
	inline
	double random_gaussian(const double sigma);

	/*
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < typename MTRX1, typename MTRX2, typename MTRX3 >
	inline
	int random_gaussian_mult(MTRX1 *cov, const size_t n, MTRX2 *ws, MTRX3 *out);

	/*
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < uint32_t D >
	inline
	int random_gaussian_mult(gnd::matrix::fixed<D,D> *cov, const size_t n, gnd::matrix::fixed<D,D> *ws, gnd::matrix::fixed<1,D> *out);

	/*
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < uint32_t D >
	inline
	int random_gaussian_mult(gnd::matrix::fixed<D,D> *cov, const size_t n, gnd::matrix::fixed<D,D> *ws, gnd::matrix::fixed<D,1> *out);
} // <--- function declaration


namespace gnd {// ---> namespace gnd

	/**
	 * @brief set random seed
	 */
	inline
	void random_set_seed( void ) {
		__gnd_random_set_seed__();
	}

	/**
	 * @brief generate randam value [0;1] with uniform probability
	 */
	inline
	double random_uniform( void ) {
		return __gnd_random_uniform__();
	}



	/**
	 * @brief generate random value following a gaussian distribution (box-muller method)
	 * @param [in] sigma : standard deviation
	 */
	inline
	double random_gaussian(const double sigma)
	{
		double x, y, r;

		do {
			x = - 1 + 2 * random_uniform();
			y = - 1 + 2 * random_uniform();

			r = x * x + y * y;
		} while( r > 1.0 || r == 0 );

		return sigma * y * sqrt(-2.0 * log(r) / r);
	}


	/**
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < typename MTRX1, typename MTRX2, typename MTRX3 >
	inline
	int random_gaussian_mult(MTRX1 *cov, const size_t n, MTRX2 *ws, MTRX3 *out)
	{
		gnd_assert(!cov || !out , -1, "null pointer");
		gnd_assert(_gnd_matrix_row_(cov) < n, -1, "invalid matrix property");
		gnd_assert(_gnd_matrix_column_(cov) < n, -1,  "invalid matrix property");
		gnd_assert(_gnd_vector_size_(out) < n, -1, "invalid matrix property");
		gnd_assert(_gnd_vector_size_(ws) < n, -1, "invalid matrix property");
		gnd_error(n == 0, 0, "this argument have no effect");

		{ // ---> initial
			size_t i;

			for(i = 0; i < n; i++)
				_gnd_matrix_ref_(ws, 0, i) = random_gaussian(1.0);
		} // <--- initial

		{ // ---> operation
			size_t i, j;
			double l00;

			gnd_error(_gnd_matrix_ref_(cov, 0, 0) <= 0, -1, "general failure.");
			l00 = ::sqrt(_gnd_matrix_ref_(cov, 0, 0));
			_gnd_matrix_ref_(cov, 0, 0) = l00;

			_gnd_vector_ref_(out, 0) = _gnd_vector_ref_(ws, 0) * l00;
			if(n > 1){
				double l10 = _gnd_matrix_ref_(cov, 1, 0) / l00;
				double diag = _gnd_matrix_ref_(cov, 1, 1) - l10 * l10;
				double l11;

				gnd_error(diag < 0, -1, "general failure.");
				l11 = ::sqrt(diag);

				_gnd_matrix_ref_(cov, 1, 0) = l10;
				_gnd_matrix_ref_(cov, 1, 1) = l11;
				_gnd_vector_ref_(out, 1) = _gnd_vector_ref_(ws, 0) * l10
						+  _gnd_vector_ref_(ws, 1) * l11;
			}

			// cholesky decomposition
			for(j = 2; j < n; j++){
				//			double ajj = _yp_matrix_ref_(cov, j, j);
				_gnd_vector_ref_(out, j) = 0;

				for(i = 0; i < j; i++){
					double sum = 0;
					double aji = _gnd_matrix_ref_(cov, j, i);
					double aii = _gnd_matrix_ref_(cov, i, i);
					double lji;

					if(i != 0){
						gnd::matrix::flex di;
						gnd::matrix::flex dj;

						gnd::matrix::assign_to_as_vector(&di, cov, i, 0, i);
						gnd::matrix::assign_to_as_vector(&dj, cov, j, 0, i);

						gnd::matrix::inner_prod(&di, &dj, &sum);
					}
					gnd_error(aii == 0, -1, "general failure.");
					lji = (aji - sum) / aii;
					_gnd_matrix_ref_(cov, j, i) = lji;
					_gnd_vector_ref_(out, j) += lji * _gnd_vector_ref_(ws, i);
				} // for(i)

				{ // ---> diagonal
					gnd::matrix::flex dj;
					double sum;
					double diag;
					double ljj;

					gnd::matrix::assign_to_as_vector(&dj, cov, j, 0, j);
					gnd::vector::sqnorm(&dj, &sum);

					diag = _gnd_matrix_ref_(cov, j, j) - sum;
					gnd_error(diag < 0, -1, "general failure.");
					ljj = ::sqrt(diag);
					_gnd_matrix_ref_(cov, j, j) = ljj;
					_gnd_vector_ref_(out, j) += ljj * _gnd_vector_ref_(ws, j);
				} // <--- diagonal

			}// for(j)

		} // <--- operation
		return 0;
	}


	/*
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < uint32_t D >
	inline
	int random_gaussian_mult(gnd::matrix::fixed<D,D> *cov, const size_t n, gnd::matrix::fixed<D,D> *ws, gnd::matrix::fixed<1,D> *out) {
		return random_gaussian_mult<gnd::matrix::fixed<D,D>, gnd::matrix::fixed<D,D>, gnd::matrix::fixed<1,D> >(cov, n, ws, out);
	}

	/*
	 * @brief generate random value following a gaussian distribution (multi-dimension)
	 */
	template < uint32_t D >
	inline
	int random_gaussian_mult(gnd::matrix::fixed<D,D> *cov, const size_t n, gnd::matrix::fixed<D,D> *ws, gnd::matrix::fixed<D,1> *out){
		return random_gaussian_mult<gnd::matrix::fixed<D,D>, gnd::matrix::fixed<D,D>, gnd::matrix::fixed<D,1> >(cov, n, ws, out);
	}
} // <--- namespace gnd


#endif /* GND_RANDOM_HPP_ */
