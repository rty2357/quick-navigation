/*
 * gnd_linalg.hpp
 *
 *  Created on: 2012/01/22
 *      Author: tyamada
 */


#include "gnd-vector-base.hpp"
#include "gnd-matrix-base.hpp"


/**
 * @ifnot GNDLinAlg
 * @defgroup GNDLinAlg linear-algebras
 * @endif GNDLinAlg
 */
namespace gnd {
	/**
	 * @ingroup GNDLinAlg
	 * @namespace linalg
	 * supply linear-algebras
	 */
	namespace linalg {
		/**
		 * @ingroup GNDLinAlg
		 * @brief cholesky matrix decomposition
		 */
		template < typename MTRX  >
		inline int cholesky_decomposition(MTRX *v, const size_t n)
		{
			gnd_assert(_gnd_matrix_row_(v) < n, -1, "invalid matrix property");
			gnd_assert(_gnd_matrix_column_(v) < n, -1,  "invalid matrix property");
			gnd_error(n == 0, 0, "this argument have no effect");


			{ // ---> operation
				size_t i, j;
				double l00;

				gnd_error(_gnd_matrix_ref_(v, 0, 0) <= 0, -1, "general failure.");
				l00 = ::sqrt(_gnd_matrix_ref_(v, 0, 0));
				_gnd_matrix_ref_(v, 0, 0) = l00;

				if(n > 1){
					double l10 = _gnd_matrix_ref_(v, 1, 0) / l00;
					double diag = _gnd_matrix_ref_(v, 1, 1) - l10 * l10;
					double l11;

					gnd_error(diag < 0, -1, "general failure.");
					l11 = ::sqrt(diag);

					_gnd_matrix_ref_(v, 1, 0) = l10;
					_gnd_matrix_ref_(v, 1, 1) = l11;
				}

				// cholesky decomposition
				for(j = 2; j < n; j++){
					for(i = 0; i < j; i++){
						double sum = 0;
						double aji = _gnd_matrix_ref_(v, j, i);
						double aii = _gnd_matrix_ref_(v, i, i);
						double lji;

						if(i != 0){
							gnd::vector::flex di;
							gnd::vector::flex dj;

							gnd::vector::init(&di);
							gnd::vector::init(&dj);

							gnd::matrix::assign_to_as_vector(&di, v, i, 0, i);
							gnd::matrix::assign_to_as_vector(&dj, v, j, 0, i);

							gnd::matrix::inner_prod(&di, &dj, &sum);
						}
						gnd_error(aii == 0, -1, "general failure.");
						lji = (aji - sum) / aii;

						_gnd_matrix_ref_(v, j, i) = lji;
					} // for(i)

					{ // ---> diagonal
						gnd::matrix::flex dj;
						double sum;
						double diag;
						double ljj;

						gnd::vector::init(&dj);
						gnd::matrix::assign_to_as_vector(&dj, v, j, 0, j);

						gnd::vector::sqnorm(&dj, &sum);

						diag = _gnd_matrix_ref_(v, j, j) - sum;
						gnd_error(diag < 0, -1, "general failure.");
						ljj = ::sqrt(diag);
						_gnd_matrix_ref_(v, j, j) = ljj;
					} // <--- diagonal

				}// for(j)

				for(i = 0; i < n; i++) {
					for(j = i + 1; j < n; j++) {
						_gnd_matrix_ref_(v, i, j) = 0;
					}
				}

			} // <--- operation
			return 0;
		}

	} // <--- namespace linalg
} // <--- namespace gnd

