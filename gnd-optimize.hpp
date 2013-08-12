/*
 * gnd-optimization.hpp
 *
 *  Created on: 2011/12/21
 *      Author: tyamada
 */

#ifndef GND_OPTIMIZE_HPP_
#define GND_OPTIMIZE_HPP_

#include <math.h>
#include <float.h>

#include "gnd-matrix-base.hpp"

/**
 * @ifnot GNDOptimize
 * @defgroup GNDOptimization optimize
 * supply optimization: newton's method
 * @endif
 */
/**
 * @namespace optimize
 * @ingroup GNDOptimize
 */

// ---> function declaration of modified newton's method for unconstrained minimization
namespace gnd { // ---> namespace gnd
	namespace optimize { // ---> namespace optimize

		template< uint32_t N >
		int newtons_method_unconstrainted( gnd::matrix::fixed<N,1> *G, gnd::matrix::fixed<N,N> *H, gnd::matrix::fixed<N,1> *d);

		template <uint32_t N>
		int _model_hessian_( gnd::matrix::fixed<N,N> *A, gnd::matrix::fixed<N,N> *L = 0);

		template< uint32_t N >
		int _perturbed_cholesky_decomposition_( gnd::matrix::fixed<N,N> *A, double ml, double *ma, gnd::matrix::fixed<N,N> *L = 0 );

	}; // <--- namespace optimize
}; // <--- namespace gnd
// <--- function declaration of modified newton's method for unconstrained minimization





// ---> constant definition
namespace gnd { // ---> namespace gnd
	namespace optimize { // ---> namespace optimize

		/**
		 * @brief machine epsilon
		 */
		const double MachineEpsilon = DBL_EPSILON;

		/**
		 * @brief sqrt machine epsilon
		 */
		const double MachineEpsilonSqrt = ::sqrt( MachineEpsilon );

		/**
		 * @brief sqrt machine epsilon
		 */
		const double MachineEpsilon4rt = ::sqrt( MachineEpsilonSqrt );

	}; // <--- namespace optimize
} // <--- namespace gnd
// <--- constant definition



// ---> function definition of modified newton's method for unconstrained minimization
namespace gnd {
	namespace optimize {
		/**
		 * @ingroup GNDoptimize
		 * @brief modified newton's method for unconstrained minimization
		 * @param[in]  *G : gradient of function
		 * @param[in]  *H : hessian of function
		 * @param[out] *d : next step
		 */
		template< uint32_t N >
		inline
		int newtons_method_unconstrainted( gnd::matrix::fixed<N,1> *G, gnd::matrix::fixed<N,N> *H, gnd::matrix::fixed<N,1> *d)
		{
			gnd_assert(!G, -1, "invalid null argument");
			gnd_assert(!H, -1, "invalid null argument");
			gnd_assert(!d, -1, "invalid null argument");

			{ // ---> operation
				gnd::matrix::fixed<N,N> HH;
				gnd::matrix::fixed<N,N> iH;

				copy(&HH, H);

				// inverse hessian
				if( gnd::matrix::inverse(&HH, &iH) < 0 ){
					return -1;
				}

				// compute d
				prod(&iH, G, d);
				scalar_prod(d, -1, d);

			} // <--- operation

			return 0;
		}


		/**
		 * @privatesection
		 * @ingroup GNDoptimize
		 * @brief formation of the model hessian
		 * @param[in,out] A : symmetrix matrix NxN (hessian matrix)
		 * @param[out]    L : cholesky factor, lower triangular
		 */
		template <uint32_t N>
		inline
		int _model_hessian_( gnd::matrix::fixed<N,N> *A, gnd::matrix::fixed<N,N> *L) {
			gnd_assert(!A, -1, "invalid null argument");

			{ // ---> operation
				double mu;
				double maxdiag;
				double mindiag;
				double maxposdiag;
				double maxoff;
				double maxoffl;
				double maxadd = 0;
				double _tsd_;	// tempolary storage, type double


				// * step 1, no necessary *

				// * step 2, no necessary *

				// step 3 and step 4
				maxdiag = mindiag = (*A)[0][0];
				for( uint32_t i = 0; i < N; i++ ) {
					maxdiag = maxdiag > (*A)[i][i] ? maxdiag : (*A)[i][i];
					mindiag = mindiag < (*A)[i][i] ? mindiag : (*A)[i][i];
				}

				// step 5
				maxposdiag = 0 < maxdiag ? 0 : maxdiag;

				// step 6
				if( mindiag <= MachineEpsilonSqrt * maxposdiag ) {
					// step 6T.1
					mu = 2 * (maxposdiag - mindiag) * MachineEpsilonSqrt - mindiag;
					maxdiag = maxdiag + mu;
				}
				else {
					// step 6E
					mu = 0;
				}

				// step 7
				maxoff = ::fabs((*A)[0][1]);
				for( uint32_t i = 0; i < N; i++)
					for(uint32_t j = i + 1; j < N; j++)	maxoff = maxoff > ( _tsd_ = ::fabs((*A)[0][1])) ? maxoff : _tsd_ ;

				// step 8
				if( (_tsd_ = maxoff * (1 + 2 * MachineEpsilonSqrt) ) > maxdiag ) {
					mu += (maxoff - maxdiag) + 2 * MachineEpsilonSqrt * maxoff;
					maxoff = _tsd_;
				}

				// step 9
				if( maxdiag == 0 ){
					mu = 1;
					maxdiag = 1;
				}

				// step 10
				if( mu > 0 ) {
					for(uint32_t i = 0; i < N; i++)	(*A)[i][i] += mu;
				}

				// step 11
				maxoffl = maxdiag > (_tsd_ = maxoff / N) ? ::sqrt(maxdiag) : ::sqrt(_tsd_);

				// step 12
				gnd::optimize::_perturbed_cholesky_decomposition_( A, maxoffl, &maxadd, L );

				// ---> step 13
				if( maxadd > 0 ) {
					double maxev;
					double minev;
					double sdd;

					// step 13.1
					maxev = (*A)[0][0];
					// step 13.2
					minev = (*A)[0][0];
					// ---> step 13.3
					for( uint32_t i = 0; i < N; i++ ){
						double offrow = 0;
						// step 13.3.1
						for( uint32_t j = 0; (signed)j < (signed)i - 1; j++ ) offrow += (*A)[j][i];
						for( uint32_t j = i + 1; j < N; j++ ) offrow += (*A)[i][j];

						// step 13.3.2
						maxev = maxev > ( _tsd_ = (*A)[i][i] + offrow ) ? maxev : _tsd_;
						minev = minev > ( _tsd_ = (*A)[i][i] - offrow ) ? minev : _tsd_;
					} // <--- step 13.3

					// step 13.4
					sdd = (maxev - minev) * ( MachineEpsilon - minev);
					// step 13.5
					sdd = sdd > 0 ? sdd : 0;
					// step 13.6
					mu = maxadd < sdd ? maxadd : sdd;

					// ---> step 13.7
					for( uint32_t i = 0; i < N; i++ ){
						(*A)[i][i] += mu;
					} // <--- step 13.7

					// step 13.8
					_perturbed_cholesky_decomposition_( A, 0, &maxadd, L );
				} // <--- step 13

			} // <--- operation

			return 0;
		}



		/**
		 * @privatesection
		 * @ingroup GNDoptimize
		 * @brief perturbed cholesky decomposition
		 * @param[in]   A : symmetric matrix
		 * @param[in]  ml : maxo f f l
		 * @param[out] ma : maximum of D diagonal element
		 * @param[out]  L : cholesky factor, lower triangular
		 *
		 * @note Find the LL^T decomposition of A + D, D a non-negative diagonal matrix
		 * that is added to A if necessary to allow the decomposition to continue,
		 * using an algorithm based on the modified Cholesky decomposition in Gill, Murray and Wright
		 */
		template< uint32_t N >
		inline
		int _perturbed_cholesky_decomposition_( gnd::matrix::fixed<N,N> *A, double ml, double *ma, gnd::matrix::fixed<N,N> *L )
		{
			gnd_assert(!A, -1, "invalid null argument");
			gnd_assert(ml < 0, -1, "invalid argument value");
			gnd_assert(!ma, -1, "invalid null argument");

			{ // ---> operation
				gnd::matrix::fixed<N,N> LL;
				double maxoffl = ml;
				double minl;
				double minl2 = 0;
				double tmp;

				// step 1
				minl = MachineEpsilon4rt * maxoffl;

				// ---> step 2
				if( maxoffl == 0 ){
					double max;
					// step 2.1
					max = (*A)[0][0];
					for( uint32_t i = 1; i < N; i++ )
						max = max > ( tmp = ::fabs( (*A)[i][i] ) ) ?
								max : tmp;
					maxoffl = ::sqrt( max );
					// step 2.2
					minl2 = MachineEpsilonSqrt * maxoffl;
				} // <--- step 2

				// step 3
				*ma = 0;

				// ---> step 4
				gnd::matrix::set_zero(&LL);
				for( uint32_t j = 0; j < N; j++ ){
					double minljj;

					// step 4.1
					LL[j][j] = (*A)[j][j];
					for( uint32_t i = 0; (signed)i < (signed)(j) - 1; i++ )
						LL[j][j] -= LL[j][i] * LL[j][i];

					// step 4.2
					minljj = 0;

					// ---> step 4.3
					for( uint32_t i = j + 1; i < N; i++ ){
						// step 4.3.1
						LL[i][j] = (*A)[j][i];
						for( uint32_t k = 0; (signed)k < (signed)(j) - 1; k++ )
							LL[i][j] -= LL[i][k] * LL[j][k];
						// step 4.3.2
						minljj = ( tmp = ::fabs(LL[i][j]) ) > minljj ?
								tmp : minljj;
					} // <--- step 4.3

					// step 4.4
					minljj = (tmp = minljj / maxoffl ) > minl ?
							tmp : minl;

					// ---> step 4.5
					if( LL[j][j] > minljj){
						// step 4.5.T
						LL[j][j] = ::sqrt(LL[j][j]);
					}
					else {
						// step 4.5.E.1
						if( minljj < minl2 ) minljj = minl2;
						// step 4.5.E.2
						*ma = *ma > (tmp = minljj * minljj - LL[j][j] ) ?
								*ma : tmp;
						// step 4.5.E.3
						LL[j][j] = minljj;
					} // <--- step 4.5

					// step 4.6
					for( uint32_t i = j + 1; i < N; i++ )
						LL[i][j] = LL[i][j] / LL[j][j];

				} // <--- step 4

				if(L) copy(L, &LL);

			} // <--- operation
			return 0;
		}
	}
};
// <--- function definition of modified newton's method for unconstrained minimization

#endif /* GND_LINEAR_ALGEBRAS_HPP_ */


