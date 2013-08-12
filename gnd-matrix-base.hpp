/*
 * gnd_matrix_base.h
 *
 *  Created on: 2011/06/11
 *      Author: tyamada
 */

#ifndef GND_MATRIX_BASE_H_
#define GND_MATRIX_BASE_H_

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "gnd-lib-error.h"

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



// ---> type declaration
namespace gnd {
	namespace matrix {
		typedef double component_t;
		struct flex;
		template < const uint32_t R, const uint32_t C >
		struct fixed;
	}
	typedef matrix::fixed<4,4> coord_matrix;
	typedef matrix::fixed<3,1> pos_vector;
}
// <--- type declaration


// ---> type definition
namespace gnd { // ---> namespace gnd
	namespace matrix { // ---> namespace matrix
		/**
		 * @ingroup GNDMatrix
		 * @brief flexible size matrix
		 */
		struct flex {
			/// @brief data buffer
			component_t *data;
			/// @brief row size
			uint32_t row;
			/// @brief column size
			uint32_t column;
			double* operator[](int i);
			flex();
		};


		/**
		 * @ingroup GNDMatrix
		 * @brief fixed size matrix
		 * @tparam R : row size
		 * @tparam C : column size
		 */
		template < const uint32_t R, const uint32_t C >
		struct fixed {
			/// @brief data buffer
			component_t data[R*C];
			/// @brief row size
			static const uint32_t row = R;
			/// @brief column size
			static const uint32_t column = C;
			double* operator[](int i);
			fixed();
			fixed(const component_t *src, uint32_t l);
			// todo copy constructor
			// fixed( const fixed<R,C> *m );
			// fixed( const flex *m );
		};
	} // <--- namespace matrix
} // <--- namespace gnd
// <--- type definition



// ---> private interface
#define _gnd_matrix_data_(mt)				( (mt)->data )

#define _gnd_matrix_row_(mt)				( (mt)->row )
#define _gnd_matrix_column_(mt)				( (mt)->column )

#define _gnd_matrix_pointer_(mt, r, c)		( (gnd::matrix::component_t *) (mt)->data + ((r)*_gnd_matrix_column_(mt)) + (c) )
#define _gnd_matrix_ref_(mt, r, c)			( *_gnd_matrix_pointer_(mt, r, c) )

#define _gnd_matrix_msize_(mt)				( _gnd_matrix_row_(mt) * _gnd_matrix_column_(mt) * sizeof(gnd::matrix::component_t) )
#define _gnd_matrix_size_(mt)				( _gnd_matrix_row_(mt) * _gnd_matrix_column_(mt) )

#define gnd_matrix_is_avail(mt)	\
		( _gnd_matrix_data_(mt) != 0 && _gnd_matrix_row_(mt) != 0 && _gnd_matrix_column_(mt) != 0)

#define gnd_matrix_is_square(mt)	\
		( gnd_matrix_is_avail(mt) && gnd_matrix_row(mt) == gnd_matrix_column(mt))

#define gnd_matrix_is_vector(vec)	\
		( gnd_matrix_is_avail(vec) && (_gnd_matrix_row_(vec) == 1 || _gnd_matrix_column_(vec) == 1) )

#define gnd_matrix_exist(mt, r, c)	\
		(gnd_matrix_is_avail(mt) && _gnd_matrix_row_(mt) > r && _gnd_matrix_column_(mt) > c )
// <--- private interface





// ---> function definition
namespace gnd { // ---> namespace gnd
	namespace matrix { // ---> namespace matrix


		/**
		 * @ingroup GNDMatrix
		 * @brief clear
		 * @param[in,out] m : flex matrix
		 * @return 0
		 * @details this function set starting value and don't deallocate memory,
		 * @sa initialize(), release(), deallocate()
		 */
		inline
		int clear( flex *m )
		{
			gnd_assert(!m, -1, "null pointer");

			_gnd_matrix_data_(m) = 0;
			_gnd_matrix_row_(m) = 0;
			_gnd_matrix_column_(m) = 0;
			return 0;
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief initialize
		 * @param [in,out]	m : matrix
		 * @return 0
		 * @sa clear(), release(), deallocate()
		 */
		inline
		int initialize ( flex *m )
		{
			return clear(m);
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief assign buffer to flexible size matrix
		 * @param[in,out] dest : destination matrix
		 * @param[in]      src : source
		 * @param[in]      row : row size
		 * @param[in]   column : column size
		 */
		inline int assign ( flex *dest, component_t *src, const uint32_t row, const uint32_t column)
		{
			gnd_assert(!dest, 	 -1, "null pointer");
			gnd_assert((row == 0), -1, "invalid argument value");
			gnd_assert((column == 0), -1, "invalid argument value");

			{
				_gnd_matrix_data_(dest) = src;
				_gnd_matrix_row_(dest) = row;
				_gnd_matrix_column_(dest) = column;
			}
			return 0;
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief assign buffer
		 * @param[out] dest : destination
		 * @param[in]   src : buffer source matrix
		 * @param[in]   row : row index
		 * @param[in]  size : row size
		 */
		inline
		int assign ( flex *dest, flex *src, const uint32_t row, const uint32_t size)
		{
			gnd_assert(!dest || !src, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix_property");
			gnd_assert(gnd_matrix_is_avail(dest), -1, "destination matrix have been already assigned");
			gnd_assert((size == 0), -1, "invalid argument value");
			gnd_assert(_gnd_matrix_row_(src) < row + size, -1, "out of buffer");

			return assign(dest, _gnd_matrix_pointer_(src, row, 0), size, _gnd_matrix_column_(src));
		}

		/**
		 * @ingroup GNDMatrix
		 * @brief assign mt with buffer
		 * @param[out] dest : buffer assigned matrix
		 * @param[in]   src : buffer source matrix
		 * @param[in]     r : row index
		 * @param[in]    rs : row size
		 */
		template< uint32_t R, uint32_t C >
		inline
		int assign ( flex *dest, fixed<R,C> *src, const uint32_t row, const uint32_t rs)
		{
			gnd_assert(!dest || !src, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix_property");
			gnd_assert(gnd_matrix_is_avail(dest), -1, "destination matrix have been already assigned");
			gnd_assert((rs == 0), -1, "invalid argument value");
			gnd_assert(_gnd_matrix_row_(src) < row + rs, -1, "out of buffer");

			return assign(dest, _gnd_matrix_pointer_(src, row, 0), rs, _gnd_matrix_column_(src));
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief release assigned buffer
		 * @param [in,out]	mt : assigned matrix
		 */
		inline int release ( flex *mt )
		{
			gnd_assert(!mt, -1, "null pointer");

			clear(mt);
			return 0;
		}





		/**
		 * @ingroup GNDMatrix
		 * @brief allocate buffer from heap
		 * @param[in,out] mt : matrix
		 * @param[in]      r : row size
		 * @param[in]      c : column size
		 */
		inline int allocate ( flex *mt, const uint32_t r, const uint32_t c )
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(gnd_matrix_is_avail(mt), -1, "invalid matrix property");


			{
				_gnd_matrix_data_(mt) = new component_t[r * c];
				gnd_assert(_gnd_matrix_data_(mt) == 0, -1, "fail to memory allocate");
				_gnd_matrix_row_(mt) = r;
				_gnd_matrix_column_(mt) = c;
			}

			return 0;
		}






		/**
		 * @ingroup GNDMatrix
		 * @brief deallocate buffer
		 * @param[in,out] mt : matrix
		 * @param[in]      r : row size
		 * @param[in]      c : column size
		 */
		inline int deallocate ( flex  *mt )
		{
			gnd_assert(!mt, -1, "null pointer");

			if(_gnd_matrix_data_(mt) == 0) return clear(mt);

			{
				delete [] _gnd_matrix_data_(mt);
				_gnd_matrix_data_(mt) = 0;
				_gnd_matrix_row_(mt) = 0;
				_gnd_matrix_column_(mt) = 0;
			}
			return 0;
		}



		// -----------------------> ref
		/**
		 * @ingroup GNDMatrix
		 * @brief get row size
		 * @param[in] mt : matrix
		 */
		template< typename MTRX >
		inline int row ( const MTRX  *mt )
		{
			gnd_assert(!mt, -1, "null pointer");
			return _gnd_matrix_row_(mt);
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief get column size
		 * @param[in] mt : matrix
		 */
		template< typename MTRX >
		inline int column ( const MTRX  *mt )
		{
			gnd_assert(!mt, -1, "null pointer");
			return _gnd_matrix_column_(mt);
		}




		/**
		 * @ingroup GNDMatrix
		 * @brief get pointer
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @return element pointer
		 */
		template< typename MTRX >
		inline component_t* pointer( MTRX  *mt, const uint32_t r, const uint32_t c )
		{
			gnd_assert(!mt, 0, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), 0, "out of buffer");

			return static_cast<component_t*>(_gnd_matrix_pointer_(mt,r,c));
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief get const pointer
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @return element const pointer
		 */
		template< typename MTRX >
		inline const component_t* pointer_const ( const MTRX  *mt, const uint32_t r, const uint32_t c )
		{
			gnd_assert(!mt, 0, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), 0, "out of buffer");
			return static_cast<const component_t*>(_gnd_matrix_pointer_(mt,r,c));
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief get value
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @return element value
		 */
		template< typename MTRX >
		inline component_t value( const MTRX *mt, const uint32_t r, const uint32_t c )
		{
			gnd_assert(!mt, 0, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), 0, "out of buffer");

			return _gnd_matrix_ref_(mt,r,c);
		}



		// ------------------------------> getter
		/**
		 * @ingroup GNDMatrix
		 * @brief get value
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @param[out] v : destination
		 */
		template< typename MTRX >
		inline int get( const MTRX  *mt, const uint32_t r, const uint32_t c, component_t *v )
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), -1, "out of buffer");

			*v = _gnd_matrix_ref_(mt, r, c);
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief get row vector
		 * @param[in]  mt : matrix
		 * @param[in]   r : row index
		 * @param[in]   c : column index
		 * @param[out]  v : destination array
		 * @param[in]   s : destination array size
		 */
		template< typename MTRX >
		inline int get( const MTRX  *mt, const uint32_t r, const uint32_t c, component_t* v, const uint32_t s )
		{
			gnd_assert(!mt || !v, -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), -1, "out of buffer");
			gnd_assert(!s, -1, "invalid argument value");
			gnd_assert(_gnd_matrix_column_(mt) < c + s, -1, "out of buffer");

			memmove(v, _gnd_matrix_pointer_(mt, r, c), sizeof(component_t) * s);
			return 0;
		}



		// ------------------------------> setter
		/**
		 * @ingroup GNDMatrix
		 * @brief set value
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @param[out] v : src
		 */
		template< typename MTRX >
		inline int set( MTRX  *mt, const uint32_t r, const uint32_t c, const component_t v )
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), -1, "out of buffer");

			_gnd_matrix_ref_(mt, r, c) = v;
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief set row vector value
		 * @param[in] mt : matrix
		 * @param[in]  r : row index
		 * @param[in]  c : column index
		 * @param[out] v : source array
		 * @param[in]  s : source array size
		 */
		template< typename MTRX >
		inline int set( MTRX  *mt, const uint32_t r, const uint32_t c, const component_t *v, const uint32_t s )
		{
			gnd_assert(!mt || !v, -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r, c), -1, "out of buffer");
			gnd_assert(!s, -1, "invalid argument value");
			gnd_assert(_gnd_matrix_column_(mt) < c + s, -1, "out of buffer");

			::memmove(_gnd_matrix_pointer_(mt, r, c), v, sizeof(component_t) * s);
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief set uniform value in the whole element
		 * @param[in,out] mt : matrix
		 * @param[in]      v : value
		 */
		template< typename MTRX >
		inline int set_uniform ( MTRX  *mt, const component_t v )
		{
			gnd_assert(!mt, -1, "null poiner");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");

			{ // ---> operate
				const uint32_t c = _gnd_matrix_column_(mt),
						r = _gnd_matrix_row_(mt);
				uint32_t i, j;

				_gnd_matrix_ref_(mt, 0, 0) = v;
				j = c >> 1;
				for(i = 1; i <= j; i <<= 1){
					memcpy(_gnd_matrix_pointer_(mt, 0, i), _gnd_matrix_pointer_(mt, 0, 0), sizeof(component_t) * i);
				}
				if(i < c)	memcpy(_gnd_matrix_pointer_(mt, 0, i), _gnd_matrix_pointer_(mt, 0, 0), sizeof(component_t) * (c - i));


				j = r >> 1;
				for(i = 1; i <= j; i <<= 1){
					memcpy(_gnd_matrix_pointer_(mt, i, 0),
							_gnd_matrix_pointer_(mt, 0, 0), sizeof(component_t) * c * i);
				}
				if(i < r)
					memcpy(_gnd_matrix_pointer_(mt, i, 0),_gnd_matrix_pointer_(mt, 0, 0), sizeof(component_t) * c * (r - i));

			} // <--- operate
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief set 0 in the whole element
		 * @param[in,out] mt : matrix
		 */
		template< typename MTRX >
		inline int set_zero( MTRX  *mt)
		{
			gnd_assert(!mt, -1, "null poiner");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");

			::memset(_gnd_matrix_pointer_(mt, 0, 0), 0, _gnd_matrix_msize_(mt));
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief set unit matrix
		 * @param  [in] mt : matrix
		 */
		template< typename MTRX >
		inline int set_unit( MTRX  *mt )
		{
			gnd_assert(!mt, -1, "null pointer");

			{ // ---> operation
				const uint32_t n = (_gnd_matrix_row_(mt) < _gnd_matrix_column_(mt))
								? _gnd_matrix_row_(mt) : _gnd_matrix_column_(mt);
				uint32_t i;
				int rsl;

				if( (rsl = set_zero(mt)) < 0 )	return rsl;

				for(i = 0; i < n; i++)	_gnd_matrix_ref_(mt, i, i) = 1;
			} // <--- operation

			return 0;
		}



		// -----------> copy
		/**
		 * @ingroup GNDMatrix
		 * @brief sub matix copy
		 * @param[out] dest : destination
		 * @param[in]    dr : start row index of destination
		 * @param[in]    dc : start column index of destination
		 * @param[in]   src : source
		 * @param[in]    sr : start row index of source
		 * @param[in]    sc : start column index of source
		 * @param[in]    rl : copy row length
		 * @param[in]    cl : copy column length
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX1, typename MTRX2 >
		inline int submatrix_copy( MTRX1  *dest, const uint32_t dr, const uint32_t dc,
				const  MTRX2  *src, const uint32_t sr, const uint32_t sc, const uint32_t rs, const uint32_t cs)
		{
			gnd_assert((!dest || !src), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(dest), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix property");
			gnd_assert((rs <= 0 && cs <= 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(dest) < dr + rs), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(src)  < sr + rs), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(dest) < dc + cs), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(src)  < sc + cs), -1, "out of buffer");


			// ---> operate
			if( _gnd_matrix_column_(dest) == cs && _gnd_matrix_column_(src) == cs && dc == 0 && sc == 0){
				memcpy(_gnd_matrix_pointer_(dest, dr, dc), _gnd_matrix_pointer_(src, sr, sc),
						sizeof(component_t) * cs * rs);
			}
			else {
				uint32_t i;

				for(i = 0; i < rs; i++){
					memcpy(_gnd_matrix_pointer_(dest, dr + i, dc), _gnd_matrix_pointer_(src, sr + i, sc),
							sizeof(component_t) * cs);
				}
			} // <--- operate

			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief sub matix copy
		 * @param[out] dest : destination
		 * @param[in]   src : source
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX1, typename MTRX2 >
		inline int copy( MTRX1 *dest,  MTRX2  *src)
		{
			gnd_assert((!dest || !src), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(dest), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix property");

			{ // ---> operate
				const uint32_t rl = (_gnd_matrix_row_(dest) < _gnd_matrix_row_(src)) ?
						_gnd_matrix_row_(dest) : _gnd_matrix_row_(src);
				const uint32_t cl = (_gnd_matrix_column_(dest) < _gnd_matrix_column_(src)) ?
						_gnd_matrix_column_(dest) : _gnd_matrix_column_(src);

				return submatrix_copy(dest, 0, 0, src, 0, 0, rl, cl);
			} // <--- operate
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief sub matix copy
		 * @param[out] dest : destination
		 * @param[in]   src : source
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int copy( fixed<R,C> *dest, const fixed<R,C> *src)
		{
			gnd_assert((!dest || !src), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(dest), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix property");

			{ // ---> operate
				::memcpy(_gnd_matrix_pointer_(dest, 0, 0), _gnd_matrix_pointer_(src, 0, 0), sizeof(component_t) * R * C);
				return 0;
			} // <--- operate
		}





		/**
		 * @ingroup GNDMatrix
		 * @brief swap row
		 * @param [in,out] mt	: matrix
		 * @param [in]	i 		: swap row index 1
		 * @param [in]	j 		: swap row index 2
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int swap_row( MTRX * mt, const uint32_t ri, const uint32_t rj)
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(_gnd_matrix_row_(mt) < ri, -1, "out of buffer");
			gnd_assert(_gnd_matrix_row_(mt) < rj, -1, "out of buffer");

			{ // ---> operation
				uint32_t i;

				for(i = 0; i < _gnd_matrix_column_(mt); i++){
					component_t value = _gnd_matrix_ref_(mt, ri, i);
					_gnd_matrix_ref_(mt, ri, i) = _gnd_matrix_ref_(mt, rj, i);
					_gnd_matrix_ref_(mt, rj, i) = value;
				}
			} // <--- operation

			return 0;
		}




		/**
		 * @ingroup GNDMatrix
		 * @brief swap row
		 * @param [in/out] mt	: matrix
		 * @param [in]	i 		: swap row index 1
		 * @param [in]	j 		: swap row index 2
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int swap_column( MTRX * mt, const uint32_t ci, const uint32_t cj)
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(_gnd_matrix_column_(mt) < ci, -1, "out of buffer");
			gnd_assert(_gnd_matrix_column_(mt) < cj, -1, "out of buffer");

			{
				uint32_t i;

				for(i = 0; i < _gnd_matrix_row_(mt); i++){
					component_t value = _gnd_matrix_ref_(mt, i, ci);
					_gnd_matrix_ref_(mt, i, ci) = _gnd_matrix_ref_(mt, i, cj);
					_gnd_matrix_ref_(mt, i, cj) = value;
				}

			}

			return 0;
		}



		// ----> transpose
		/**
		 * @ingroup GNDMatrix
		 * @brief compute transpose matrix
		 * @param [out] trans : transpose matrix
		 * @param [in]     tr : start row index of destination
		 * @param [in]     tc : start column index of destination
		 * @param [in]    src : source matrix
		 * @param [in]     sr : start row index of destination
		 * @param [in]     sc : start column index of destination
		 * @param [in]     rl : start row index of source
		 * @param [in]     cl : start column index of source
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX1, typename MTRX2 >
		inline int submatrix_transpose( MTRX1 *trn, const uint32_t tr, const uint32_t tc,
				const MTRX2 *src, const uint32_t sr, const uint32_t sc,
				const uint32_t rl, const uint32_t cl)
		{
			gnd_assert((!src || !trn), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(trn),  -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix property");
			gnd_assert((rl == 0 && cl == 0), 0, "this argument value have no effect");

			gnd_assert((_gnd_matrix_row_(trn) < tr + cl), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(src)  < sr + rl), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(trn) < tc + rl), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(src)  < sc + cl), -1, "out of buffer");

			{ // ---> operator
				uint32_t i, j;

				for(i = 0; i < cl; i++){
					for(j = 0; j < rl; j++){
						_gnd_matrix_ref_(trn, tr + i, tc + j) = _gnd_matrix_ref_(src, sr + j, sr + i);
					}
				}
			} // <--- operator

			return 0;
		}





		/**
		 * @ingroup GNDMatrix
		 * @brief compute transpose matrix
		 * @param [out] trans : transpose matrix
		 * @param [in]    src : source matrix
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2>
		inline int transpose( MTRX1* tr, const MTRX2* src)
		{
			gnd_assert((!tr || !src), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(tr), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix property");

			{
				const uint32_t n = (_gnd_matrix_row_(tr) < _gnd_matrix_column_(src)) ?
						_gnd_matrix_row_(tr) : _gnd_matrix_column_(src);
				const uint32_t m = (_gnd_matrix_column_(tr) < _gnd_matrix_row_(src)) ?
						_gnd_matrix_column_(tr) : _gnd_matrix_row_(src);
				return submatrix_transpose(tr, 0, 0, src, 0, 0, m, n);
			}
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief compute transpose matrix
		 * @param [out] trans : transpose matrix
		 * @param [in]    src : source matrix
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		inline int gnd_matrix_transpose( flex* tr)
		{
			gnd_assert((!tr), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(tr), -1, "invalid matrix property");



			{ // ---> operation
				{ // ---> swap data
					uint32_t i, j;
					component_t swp;


					for(i = 0; i < _gnd_matrix_row_(tr); i++){
						for(j = i + 1; j < _gnd_matrix_column_(tr); j++){
							swp = _gnd_matrix_ref_(tr, i, j);
							_gnd_matrix_ref_(tr, i, j) = _gnd_matrix_ref_(tr, j, i);
							_gnd_matrix_ref_(tr, j, i) = swp;
						}
					}
				} // <--- swap data
				{ // ---> swap size
					uint32_t swp = _gnd_matrix_row_(tr);
					_gnd_matrix_row_(tr) = _gnd_matrix_column_(tr);
					_gnd_matrix_column_(tr) = swp;
				} // <--- swap size
				return 0;
			}
		}


		// ---> operation
		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int add(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t r, c;

				r = (_gnd_matrix_row_(m1) < _gnd_matrix_row_(m2)) ? _gnd_matrix_row_(m1) : _gnd_matrix_row_(m2);
				r = (r < _gnd_matrix_row_(o)) ? r : _gnd_matrix_row_(o);
				c = (_gnd_matrix_column_(m1) < _gnd_matrix_column_(m2)) ? _gnd_matrix_column_(m1) : _gnd_matrix_column_(m2);
				c = (c < _gnd_matrix_column_(o)) ? r : _gnd_matrix_column_(o);

				return submatrix_add(m1, 0, 0,
						m2, 0, 0,
						r, c,
						o, 0, 0);
			}
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief addition transposed mt1 and mt2
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int add_transpose1(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t r, c;

				r = (_gnd_matrix_column_(m1) < _gnd_matrix_row_(m2)) ? _gnd_matrix_column_(m1) : _gnd_matrix_row_(m2);
				r = (r < _gnd_matrix_row_(o)) ? r : _gnd_matrix_row_(o);
				c = (_gnd_matrix_row_(m1) < _gnd_matrix_column_(m2)) ? _gnd_matrix_row_(m1) : _gnd_matrix_column_(m2);
				c = (c < _gnd_matrix_column_(o)) ? r : _gnd_matrix_column_(o);

				return submatrix_add_transpose1(m1, 0, 0,
						m2, 0, 0,
						r, c,
						o, 0, 0);
			}
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int add_transpose2(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t r, c;

				r = (_gnd_matrix_row_(m1) < _gnd_matrix_column_(m2)) ? _gnd_matrix_row_(m1) : _gnd_matrix_column_(m2);
				r = (r < _gnd_matrix_row_(o)) ? r : _gnd_matrix_row_(o);
				c = (_gnd_matrix_column_(m1) < _gnd_matrix_row_(m2)) ? _gnd_matrix_column_(m1) : _gnd_matrix_row_(m2);
				c = (c < _gnd_matrix_column_(o)) ? r : _gnd_matrix_column_(o);

				return submatrix_add_transpose2(m1, 0, 0,
						m2, 0, 0,
						r, c,
						o, 0, 0);
			}
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int add(const fixed<R,C> *m1, const fixed<R,C> *m2, fixed<R,C> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t i;
				const component_t *p1 = _gnd_matrix_pointer_(m1, 0, 0),
						*p2 = _gnd_matrix_pointer_(m2, 0, 0);
				component_t *po = _gnd_matrix_pointer_(o, 0, 0);
				for(i = 0; i < R * C; i++){
					*po++ = *p1++ + *p2++;
				}
			}
			return 0;
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1(transpose)
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int add_transpose1(const fixed<C,R> *m1, const fixed<R,C> *m2, fixed<R,C> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t i, j;

				for(i = 0; i < R; i++){
					for(j = 0; j < C; j++){
						_gnd_matrix_ref_(o, i, j) =
								_gnd_matrix_ref_(m1, j, i) + _gnd_matrix_ref_(m2, i, j);
					}
				}
			}
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2(transpose)
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int add_transpose2(const fixed<R,C> *m1, const fixed<C,R> *m2, fixed<R,C> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t i, j;

				for(i = 0; i < R; i++){
					for(j = 0; j < C; j++){
						_gnd_matrix_ref_(o, i, j) =
								_gnd_matrix_ref_(m1, i, j) + _gnd_matrix_ref_(m2, j, i);
					}
				}
			}
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief sub matrix addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   r1 : row index matrix1
		 * @param  [in]   c1 : column index matrix1
		 * @param  [in]   m2 : matrix2
		 * @param  [in]   r2 : row index matrix2
		 * @param  [in]   c2 : column index matrix2
		 * @param  [in]    r : submatrix row size
		 * @param  [in]    c : submatrix column size
		 * @param [out]    o : output buffer
		 * @param  [in]   ro : row index output buffer
		 * @param  [in]   co : column index output buffer
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int submatrix_add(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t r, const uint32_t c,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert(( r == 0 || c == 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + c), -1, "out of buffer");

			{ // ---> operation
				unsigned int i, j;

				for(i = 0; i < r; i++){
					for(j = 0; j < c; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) =
								_gnd_matrix_ref_(m1, r1 + i, c1 + j) + _gnd_matrix_ref_(m2, r2 + i, c2 + j);
					}
				}
			} // <--- operation

			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief sub matrix addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   r1 : row index matrix1
		 * @param  [in]   c1 : column index matrix1
		 * @param  [in]   m2 : matrix2
		 * @param  [in]   r2 : row index matrix2
		 * @param  [in]   c2 : column index matrix2
		 * @param  [in]    r : submatrix row size
		 * @param  [in]    c : submatrix column size
		 * @param [out]    o : output buffer
		 * @param  [in]   ro : row index output buffer
		 * @param  [in]   co : column index output buffer
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int submatrix_add_transpose1(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t r, const uint32_t c,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + c), -1, "out of buffer");
			gnd_error(( r == 0 || c == 0), 0, "this argument value have no effect");

			{ // ---> operation
				unsigned int i, j;

				for(i = 0; i < r; i++){
					for(j = 0; j < c; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) =
								_gnd_matrix_ref_(m1, r1 + j, c1 + i) + _gnd_matrix_ref_(m2, r2 + i, c2 + j);
					}
				}
			} // <--- operation

			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief sub matrix addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   r1 : row index matrix1
		 * @param  [in]   c1 : column index matrix1
		 * @param  [in]   m2 : matrix2
		 * @param  [in]   r2 : row index matrix2
		 * @param  [in]   c2 : column index matrix2
		 * @param  [in]    r : submatrix row size
		 * @param  [in]    c : submatrix column size
		 * @param [out]    o : output buffer
		 * @param  [in]   ro : row index output buffer
		 * @param  [in]   co : column index output buffer
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int submatrix_add_transpose2(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t r, const uint32_t c,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert(( r == 0 || c == 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + c), -1, "out of buffer");

			{ // ---> operation
				unsigned int i, j;

				for(i = 0; i < r; i++){
					for(j = 0; j < c; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) =
								_gnd_matrix_ref_(m1, r1 + i, c1 + j) + _gnd_matrix_ref_(m2, r2 + j, c2 + i);
					}
				}
			} // <--- operation

			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief subtraction m1 - m2
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int sub(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t r, c;

				r = (_gnd_matrix_row_(m1) < _gnd_matrix_row_(m2)) ? _gnd_matrix_row_(m1) : _gnd_matrix_row_(m2);
				r = (r < _gnd_matrix_row_(o)) ? r : _gnd_matrix_row_(o);
				c = (_gnd_matrix_column_(m1) < _gnd_matrix_column_(m2)) ? _gnd_matrix_column_(m1) : _gnd_matrix_column_(m2);
				c = (c < _gnd_matrix_column_(o)) ? r : _gnd_matrix_column_(o);

				return submatrix_sub(m1, 0, 0,
						m2, 0, 0,
						r, c,
						o, 0, 0);
			}
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief submatrix subtraction
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   r1 : row index matrix1
		 * @param  [in]   c1 : column index matrix1
		 * @param  [in]   m2 : matrix2
		 * @param  [in]   r2 : row index matrix2
		 * @param  [in]   c2 : column index matrix2
		 * @param  [in]    r : submatrix row size
		 * @param  [in]    c : submatrix column size
		 * @param [out]    o : output buffer
		 * @param  [in]   ro : row index output buffer
		 * @param  [in]   co : column index output buffer
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int submatrix_sub(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t r, const uint32_t c,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert(( r == 0 || c == 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + c), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + r), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + c), -1, "out of buffer");

			{ // ---> operation
				unsigned int i, j;

				for(i = 0; i < r; i++){
					for(j = 0; j < c; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) =
								_gnd_matrix_ref_(m1, r1 + i, c1 + j) - _gnd_matrix_ref_(m2, r2 + i, c2 + j);
					}
				}
			} // <--- operation

			return 0;
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief subtraction
		 * @param  [in]   m1 : matrix1(transpose)
		 * @param  [in]   m2 : matrix2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int sub_transpose1(const fixed<C,R> *m1, const fixed<R,C> *m2, fixed<R,C> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t i, j;

				for(i = 0; i < R; i++){
					for(j = 0; j < C; j++){
						_gnd_matrix_ref_(o, i, j) =
								_gnd_matrix_ref_(m1, j, i) - _gnd_matrix_ref_(m2, i, j);
					}
				}
			}
			return 0;
		}



		/**
		 * @ingroup GNDMatrix
		 * @brief addition
		 * @param  [in]   m1 : matrix1
		 * @param  [in]   m2 : matrix2(transpose)
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int sub_transpose2(const fixed<R,C> *m1, const fixed<C,R> *m2, fixed<R,C> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");

			{
				uint32_t i, j;

				for(i = 0; i < R; i++){
					for(j = 0; j < C; j++){
						_gnd_matrix_ref_(o, i, j) =
								_gnd_matrix_ref_(m1, i, j) - _gnd_matrix_ref_(m2, j, i);
					}
				}
			}
			return 0;
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief Sub Matrix Product
		 * @param [in]  m1 : matrix1 : m x n
		 * @param [in]  r1 : head row index (matrix1 )
		 * @param [in]  rc1 : head column index (matrix1)
		 * @param [in]  m2 : matrix2 : n x l
		 * @param [in]  r2 : head row index (matrix2)
		 * @param [in]  rc2 : head column index (matrix2)
		 * @param [in]   m  : m
		 * @param [in]   n  : n
		 * @param [in]   l  : l
		 * @param [out] out : output  : m x l
		 * @param [in]  ro : head row index (out)
		 * @param [in]  co : head coluoindex (out)
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int submatrix_prod(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t m, const uint32_t n, const uint32_t l,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert( ( m == 0 || n == 0 || l == 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + l), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + l), -1, "out of buffer");


			{ // ---> operation
				unsigned int i, j, k;

				for(i = 0; i < m; i++){
					for(j = 0; j < l; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) = 0;
						for(k = 0; k < n; k++){
							_gnd_matrix_ref_(o, ro + i, co + j) +=
									_gnd_matrix_ref_(m1, r1 + i, c1 + k) * _gnd_matrix_ref_(m2, r2 + k, c2 + j);
						}
					}
				}
			} // <--- operation
			return 0;
		}



		/**
		 * @brief Matrix Product
		 * @param [in]  m1 : matrix1 : m x n
		 * @param [in]  m2 : matrix2 : n x l
		 * @param [out]  o : output  : m x l
		 */
		template <typename MTRX1, typename MTRX2, typename MTRX3>
		inline int prod(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");

			{ // ---> operation
				const uint32_t m = (_gnd_matrix_row_(m1) < _gnd_matrix_row_(o)) ? gnd::matrix::row(m1) : gnd::matrix::row(o);
				const uint32_t n = (_gnd_matrix_column_(m1) < _gnd_matrix_row_(m2)) ? gnd::matrix::column(m1) : gnd::matrix::row(m2);
				const uint32_t l = (_gnd_matrix_column_(m2) < _gnd_matrix_column_(o)) ? gnd::matrix::column(m2) : gnd::matrix::column(o);
				return submatrix_prod(m1, 0, 0, m2, 0, 0, m, n, l, o, 0, 0);
			} // <--- operation
		}
		template < uint32_t M, uint32_t N, uint32_t L >
		inline int prod(const fixed<M,N> *m1, const fixed<N,L> *m2, fixed<M,L> *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");

			{
				unsigned int i, j, k;

				for(i = 0; i < M; i++){
					for(j = 0; j < L; j++){
						_gnd_matrix_ref_(o, i, j) = 0;
						for(k = 0; k < N; k++){
							_gnd_matrix_ref_(o, i, j) +=
									_gnd_matrix_ref_(m1, i, k) * _gnd_matrix_ref_(m2, k, j);
						}
					}
				}
			}
			return 0;
		}



		/**
		 * @brief product transposed mt1 and mt2 (submatrix)
		 * @param [in]  m1 : matrix 1 (n x m)
		 * @param [in]  r1 : start row 2
		 * @param [in]  c1 : start column
		 * @param [in]  m2 : matrix 2 (n x l)
		 * @param [in]  r2 : start row 2
		 * @param [in]  c2 : start column 2
		 * @param [in]  m  : size of submatrix
		 * @param [in]  n  : size of submatrix
		 * @param [in]  l  : size of submatrix
		 * @param [out] o  : product result (m x l)
		 * @param [in]  ro : start row out
		 * @param [in]  co : start column out
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int submatrix_prod_transpose1(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t m, const uint32_t n, const uint32_t l,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert(( m == 0 || n == 0 || l == 0), 0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + l), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + l), -1, "out of buffer");
			gnd_assert(_gnd_matrix_pointer_(m1, 0, 0) == _gnd_matrix_pointer_(o, 0, 0)
					|| _gnd_matrix_pointer_(m2, 0, 0) == _gnd_matrix_pointer_(o, 0, 0), -1, "invalid argument");



			{
				unsigned int i, j, k;

				for(i = 0; i < m; i++){
					for(j = 0; j < l; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) = 0;
						for(k = 0; k < n; k++){
							_gnd_matrix_ref_(o, ro + i, co + j) +=
									_gnd_matrix_ref_(m1, r1 + k, c1 + i) * _gnd_matrix_ref_(m2, r2 + k, c2 + j);
						}
					}
				}
			}
			return 0;
		}



		/**
		 * @brief product transposed mt1 and mt2
		 * @param [in] m1 : matrix 1
		 * @param [in] m2 : matrix 2
		 * @param [out] o : product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template<typename MTRX1, typename MTRX2, typename MTRX3>
		inline int prod_transpose1 ( const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");

			const uint32_t m = (_gnd_matrix_column_(m1) < _gnd_matrix_row_(o)) ? _gnd_matrix_column_(m1) : _gnd_matrix_row_(o);
			const uint32_t n = (_gnd_matrix_row_(m1) < _gnd_matrix_row_(m2)) ? _gnd_matrix_row_(m1) : _gnd_matrix_row_(m2);
			const uint32_t l = (_gnd_matrix_column_(m2) < _gnd_matrix_column_(o)) ? _gnd_matrix_column_(m2) : _gnd_matrix_column_(o);
			return submatrix_prod_transpose1(m1, 0, 0, m2, 0, 0, m, n, l, o, 0, 0);
		}


		/**
		 * @brief product mt1 and transposed mt2 (submatrix)
		 * @param [in]  mt1	: matrix 1 (m x n)
		 * @param [in]  rs1 : start row 2
		 * @param [in]  cs1 : start column
		 * @param [in]  mt2	: matrix 2 (l x n)
		 * @param [in]  rs2 : start row 2
		 * @param [in]  cs2 : start column 2
		 * @param [in]  m	: size of submatrix
		 * @param [in]  n	: size of submatrix
		 * @param [in]  l	: size of submatrix
		 * @param [out] out	: product result (m x l)
		 * @param [in]  rso : start row out
		 * @param [in]  cso : start column out
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template<typename MTRX1, typename MTRX2, typename MTRX3>
		inline int submatrix_prod_transpose2(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2,const uint32_t c2,
				const uint32_t m, const uint32_t n, const uint32_t l,
				MTRX3 *o, const uint32_t ro,const uint32_t co)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(o), -1, "invalid matrix property");
			gnd_assert(( m == 0 || n == 0 || l == 0),  0, "this argument value have no effect");
			gnd_assert((_gnd_matrix_row_(m1) < r1 + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m1) < c1 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2) < r2 + l), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m2) < c2 + n), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(o) < ro + m), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(o) < co + l), -1, "out of buffer");
			gnd_assert(_gnd_matrix_pointer_(m1, 0, 0) == _gnd_matrix_pointer_(o, 0, 0), -1, "invalid argument");
			gnd_assert(_gnd_matrix_pointer_(m2, 0, 0) == _gnd_matrix_pointer_(o, 0, 0), -1, "invalid argument");


			{ // ---> operation
				unsigned int i, j, k;

				for(i = 0; i < m; i++){
					for(j = 0; j < l; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) = 0;
						for(k = 0; k < n; k++){
							_gnd_matrix_ref_(o, ro + i, co + j) +=
									_gnd_matrix_ref_(m1, r1 + i, c1 + k) * _gnd_matrix_ref_(m2, r2 + j, c2 + k);
						}
					}
				}
			} // <--- operation
			return 0;
		}


		/**
		 * @brief product mt1 and transposed mt2
		 * @param [in] mt1	: matrix 1
		 * @param [in] mt2	: matrix 2
		 * @param [out] out		: product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int prod_transpose2(const MTRX1 *m1, const MTRX2 *m2, MTRX3 *o)
		{
			gnd_assert((!m1 || !m2 || !o), -1, "null pointer");

			const uint32_t m = (_gnd_matrix_row_(m1) < _gnd_matrix_row_(o)) ? _gnd_matrix_row_(m1) : _gnd_matrix_row_(o);
			const uint32_t n = (_gnd_matrix_column_(m1) < _gnd_matrix_column_(m2)) ? _gnd_matrix_column_(m1) : _gnd_matrix_column_(m2);
			const uint32_t l = (_gnd_matrix_row_(m2) < _gnd_matrix_column_(o)) ? _gnd_matrix_row_(m2) : _gnd_matrix_column_(o);
			return submatrix_prod_transpose2(m1, 0, 0, m2, 0, 0, m, n, l, o, 0, 0);
		}



		/**
		 * @brief product
		 * @param [in] mt : matrix
		 * @param [in]  r : row index
		 * @param [in]  c : column index
		 * @param [in]  v : v
		 * @param [in] rs : sub-matrix row size
		 * @param [in] cs : sub-matrix column size
		 * @param [out] o : product result
		 * @param [in] ro : row index
		 * @param [in] co : column index
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2 >
		inline int submatrix_scalar_prod(const MTRX1 *mt, const uint32_t r, const uint32_t c,
				const uint32_t rs, const uint32_t cs,
				const double v,
				MTRX2 *o, const uint32_t ro, const uint32_t co)
		{
			gnd_assert((!mt || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r + rs - 1, c + cs -1), -1, "out of buffer");
			gnd_assert(!gnd_matrix_exist(o, ro + rs - 1, co + cs -1), -1, "out of buffer");
			gnd_warnning(rs == 0 || cs == 0, "this argument value have no effect");


			{ // ---> operation
				uint32_t i, j;

				for(i = 0; i < rs; i++){
					for(j = 0; j < cs; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) = _gnd_matrix_ref_(mt, r + i, c + j) * v;
					}
				}
			} // <--- operation
			return 0;
		}



		/**
		 * @brief product
		 * @param [in] mt : matrix 1
		 * @param [in]  v : v
		 * @param [out] o : product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2 >
		inline int scalar_prod(const MTRX1 *mt, const double v, MTRX2 *o)
		{
			gnd_assert((!mt || !o), -1, "null pointer");

			{ // ---> opeartion
				const uint32_t rs = row(mt) < row(o) ? row(mt) : row(o);
				const uint32_t cs = column(mt) < column(o) ? column(mt) : column(o);
				return submatrix_scalar_prod(mt, 0, 0, rs, cs, v, o, 0, 0);
			} // <--- opeartion
		}

		template < uint32_t R, uint32_t C >
		inline int scalar_prod(const fixed<R,C> *mt, const double v, fixed<R,C> *o)
		{
			gnd_assert((!mt || !o), -1, "null pointer");

			{ // ---> operation
				uint32_t i;
				const component_t *p = _gnd_matrix_pointer_(mt, 0, 0);
				component_t *po = _gnd_matrix_pointer_(o, 0, 0);

				for(i = 0; i < R * C; i++){
					*po++ = *p++ * v;
				}
			} // <--- operation
			return 0;
		}


		/**
		 * @brief sub-matrix scalar divide
		 * @param [in] mt : matrix
		 * @param [in]  r : row index
		 * @param [in]  c : column index
		 * @param [in]  v : v
		 * @param [in] rs : sub-matrix row size
		 * @param [in] cs : sub-matrix column size
		 * @param [out] o : product result
		 * @param [in] ro : row index
		 * @param [in] co : column index
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template<typename MTRX1, typename MTRX2>
		inline int submatrix_scalar_div(const MTRX1 *mt, const uint32_t r, const uint32_t c,
				const double v,
				const uint32_t rs, const uint32_t cs,
				MTRX2 *o, const uint32_t ro, const uint32_t co)
		{
			gnd_assert((!mt || !o), -1, "null pointer");
			gnd_assert(!gnd_matrix_exist(mt, r + rs - 1, c + cs -1), -1, "out of buffer");
			gnd_assert(!gnd_matrix_exist(mt, r + rs - 1, c + cs -1), -1, "out of buffer");
			gnd_assert(v == 0, -1, "zero divide");
			gnd_warnning(rs == 0 || cs == 0, "this argument value have no effect");


			{ // ---> operation
				uint32_t i, j;

				for(i = 0; i < rs; i++){
					for(j = 0; j < cs; j++){
						_gnd_matrix_ref_(o, ro + i, co + j) = _gnd_matrix_ref_(mt, r + i, c + j) / v;
					}
				}
			} // <--- operation
			return 0;
		}



		/**
		 * @brief scalar divide
		 * @param [in] mt : matrix 1
		 * @param [in]  v : v
		 * @param [out] o : product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template<typename MTRX1, typename MTRX2>
		inline int scalar_div(const MTRX1 *mt, const double v, MTRX2 *o)
		{
			gnd_assert((!mt || !o), -1, "null pointer");

			{ // ---> opeartion
				const uint32_t rs = _gnd_matrix_row_(mt) < _gnd_matrix_row_(o) ? _gnd_matrix_row_(mt) : _gnd_matrix_row_(o);
				const uint32_t cs = _gnd_matrix_column_(mt) < _gnd_matrix_column_(o) ? _gnd_matrix_column_(mt) : _gnd_matrix_column_(o);
				return submatrix_scalar_div(mt, 0, 0, v, rs, cs, o, 0, 0);
			} // <--- opeartion
		}



		/**
		 * @brief Compute Square of Norm (sub-vector row)
		 * @param [in]  mt : matrix
		 * @param [in]   r : row index
		 * @param [in]   c : column index
		 * @param [in]   l : subvector length
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template<typename MTRX>
		inline int submatrix_sqnorm_row(const MTRX *mt, const uint32_t r, const uint32_t c, const uint32_t l, double *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(_gnd_matrix_row_(mt) < r, -1, "out of buffer");
			gnd_assert(_gnd_matrix_column_(mt) < c + l, -1, "out of buffer");

			{ // ---> operation
				const uint32_t e = c + l;
				uint32_t i;

				*v = 0;
				for(i = c; i < e; i++){
					*v += _gnd_matrix_ref_(mt, r, i) * _gnd_matrix_ref_(mt, r, i);
				}
			} // <---- operation
			return 0;
		}


		/**
		 * @brief Compute Square of Norm (sub-vector row)
		 * @param [in]  mt : matrix
		 * @param [in]   i : row index
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int sqnorm_row(const MTRX *mt, const uint32_t i, double *v)
		{
			return submatrix_sqnorm_row(mt, i, 0, _gnd_matrix_column_(mt), v);
		}



		/**
		 * @brief Compute Norm (sub-vector row)
		 * @param [in]  mt : matrix
		 * @param [in]   r : row index
		 * @param [in]   c : column index
		 * @param [in]   l : sub-vector length
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int submatrix_norm_row(const MTRX *mt, const uint32_t r, const uint32_t c, const uint32_t l, double *v)
		{
			int fres;
			if( (fres = submatrix_sqnorm_row(mt, r, c, l, v)) < 0){
				return fres;
			}
			*v = sqrt(*v);
			return 0;
		}



		/**
		 * @brief Compute Norm (sub-vector row)
		 * @param [in]  mt : matrix
		 * @param [in]   i : row index
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int norm_row(const MTRX *mt, const uint32_t i, double *v)
		{
			int fres;
			if( (fres = sqnorm_row(mt, i, v)) < 0){
				return fres;
			}
			*v = sqrt(*v);
			return 0;
		}






		/**
		 * @brief Compute Square of Norm (sub-vector column)
		 * @param [in]  mt : matrix
		 * @param [in]   r : row index
		 * @param [in]   c : column index
		 * @param [in]   l : sub-vector length
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int submatrix_sqnorm_column(const MTRX *mt, const uint32_t r, const uint32_t c, const uint32_t l, double *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(_gnd_matrix_column_(mt) < c, -1, "out of buffer");
			gnd_assert(_gnd_matrix_row_(mt) < r + l, -1, "out of buffer");

			{ // ---> operation
				const uint32_t e = r + l;
				uint32_t i;

				*v = 0;
				for(i = r; i < e; i++){
					*v += _gnd_matrix_ref_(mt, i, c) * _gnd_matrix_ref_(mt, i, c);
				}
			} // <---- operation
			return 0;
		}

		/**
		 * @brief Compute Norm (sub-vector column)
		 * @param [out] mt : matrix
		 * @param [in]   i : column index
		 * @param [in]   v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int sqnorm_column(const MTRX *mt, const uint32_t i, double *v)
		{
			return submatrix_sqnorm_column(mt, 0, i, _gnd_matrix_row_(mt), v);
		}



		/**
		 * @brief Compute Norm (sub-vector column)
		 * @param [in]  mt : matrix
		 * @param [in]   r : row index
		 * @param [in]   c : column index
		 * @param [in]   l : sub-vector length
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int submatrix_norm_column(const MTRX *mt, const uint32_t r, const uint32_t c, const uint32_t l, double *v)
		{
			int fres;
			if( (fres = submatrix_sqnorm_column(mt, r, c, l, v)) < 0){
				return fres;
			}
			*v = sqrt(*v);
			return 0;
		}




		/**
		 * @brief Compute Norm (sub-vector column)
		 * @param [in]  mt : matrix
		 * @param [in]   i : column index
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template< typename MTRX >
		inline int norm_column(const MTRX *mt, const uint32_t i, double *v)
		{
			int fres;
			if( (fres = sqnorm_column(mt, i, v)) < 0){
				return fres;
			}
			*v = sqrt(*v);
			return 0;
		}



		/**
		 * @brief Compute Determinant (matrix 2x2)
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1 >
		inline int _det_2x2_( MTRX1 *mt, double *v)
		{
			*v = _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 1, 1)
						- _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 1, 0);

			return 0;
		}

		/**
		 * @brief Compute Determinant (matrix 3x3)
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1 >
		inline int _det_3x3_( MTRX1 *mt, double *v)
		{
			*v = _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 2)
						+ _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 0, 2)
						+ _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 1, 2)
						- _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 1, 2)
						- _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 0, 2)
						- _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 2, 2);

			return 0;
		}



		/**
		 * @brief Compute Determinant (matrix 4x4)
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1 >
		inline int _det_4x4_( MTRX1 *mt, double *v)
		{
			*v =  _gnd_matrix_ref_(mt, 0, 0) *
					(   _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) )
							+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) )
							+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) )
					)

					+ _gnd_matrix_ref_(mt, 0, 1) *
					(   _gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) )
							+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) )
							+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) )
					)

					+ _gnd_matrix_ref_(mt, 0, 2) *
					(	_gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) )
							+ _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) )
							+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) )
					)

					+ _gnd_matrix_ref_(mt, 0, 3) *
					(	_gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) )
							+ _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) )
							+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) )
					);

			return 0;
		}




		/**
		 * @brief Compute Determinant
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2 >
		inline int det(const MTRX1 *mt, MTRX2 *ws, double *v)
		{
			gnd_assert((!mt || !ws || !v), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(ws), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(mt) > _gnd_matrix_row_(ws)), -1, "lock of workspace");
			gnd_assert((_gnd_matrix_column_(mt) > _gnd_matrix_column_(ws)), -1, "lock of workspace");


			{ // ---> Initialize
				copy(ws, mt);
				*v = 1;
			} // <--- Initialize

			{ // ---> operate
				uint32_t i, j, k;
				uint32_t ipivot;
				const uint32_t n =
						(_gnd_matrix_row_(ws) < _gnd_matrix_column_(ws)) ? _gnd_matrix_row_(ws) : _gnd_matrix_column_(ws);


				// explicit case
				if(n == 2){
					return _det_2x2_(mt, v);
				}
				else if(n == 3){
					return _det_3x3_(mt, v);
				}
				else if(n == 4){
					return _det_4x4_(mt, v);
				}


				for(i = 0; i < n; i++){
					{ // ---> pivot
						component_t max = fabs(_gnd_matrix_ref_(ws, i, i));
						component_t abs_ji;

						ipivot = i;
						for(j = i + 1; j < n; j++){
							abs_ji = fabs(_gnd_matrix_ref_(ws, j, i));
							if(max < abs_ji){
								max = abs_ji;
								ipivot = j;
							}
						}
						if(ipivot != i){
							swap_row(ws, i, ipivot);
							*v = -(*v);
						}
					} // <--- pivot
					(*v) *= _gnd_matrix_ref_(ws, i, i);
					// if a diagonal element is equal 0, determinant is 0 and finish computing
					if(_gnd_matrix_ref_(ws, i, i) == 0) return 0;

					for(j = i + 1; j < n; j++){
						component_t ratio = _gnd_matrix_ref_(ws, j, i) / _gnd_matrix_ref_(ws, i, i);
						for(k = i + 1; k < n; k++){
							_gnd_matrix_ref_(ws, j, k) -= ratio * _gnd_matrix_ref_(ws, i, k);
						} // for(k)
					} // for(j)

				} // for(i)

			} // <--- operate

			return 0;
		}

		/**
		 * @brief Compute Determinant (fixed)
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C, typename MTRX >
		inline int det(const fixed<R, C> *mt, component_t *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");

			{ // ---> operation
				fixed<R, C> ws;

				return gnd_matrix_det(mt, &ws, v);
			} // <--- operation
		}

		/**
		 * @brief Compute Determinant (explicit 2x2)
		 * @param [in]  mt  : Matrix (2x2)
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int det(const fixed<2, 2> *mt, component_t *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");

			{ // ---> operation
				return _det_2x2_(mt,v);
			} // <--- operation
		}

		/**
		 * @brief Compute Determinant (explicit 3x3)
		 * @param [in]  mt  : Matrix (3x3)
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int det(const fixed<3, 3> *mt, component_t *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");

			{ // ---> operation
				return _det_3x3_(mt,v);
			} // <--- operation
		}

		/**
		 * @brief Compute Determinant (explicit 4x4)
		 * @param [in]  mt  : Matrix (4x4)
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int det(const fixed<4, 4> *mt, component_t *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");

			{ // ---> operation
				return _det_4x4_(mt,v);
			} // <--- operation
		}



		/**
		 * @brief Compute Determinant
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] v 	: Determinant
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C >
		inline int det(const fixed<R, C> *mt, double *v)
		{
			gnd_assert((!mt || !v), -1, "null pointer");

			{ // ---> operation
				fixed<R, C> ws;
				return det(mt, &ws, v);
			} // <--- operation
		}



		/**
		 * @brief Compute Invert Matrix (2x2)
		 * @param [in]  mt  : Matrix (2x2)
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2  >
		inline int _inverse_2x2_( MTRX1 *mt, MTRX2 *inv)
		{
			double det;

			_det_2x2_(mt, &det);
			if( det == 0 )	return -1;

			_gnd_matrix_ref_(inv, 0, 0) =   _gnd_matrix_ref_(mt, 1, 1) / det;
			_gnd_matrix_ref_(inv, 0, 1) = - _gnd_matrix_ref_(mt, 0, 1) / det;
			_gnd_matrix_ref_(inv, 1, 0) = - _gnd_matrix_ref_(mt, 1, 0) / det;
			_gnd_matrix_ref_(inv, 1, 1) =   _gnd_matrix_ref_(mt, 0, 0) / det;

			return 0;
		}



		/**
		 * @brief Compute Invert Matrix (3x3)
		 * @param [in]  mt  : Matrix (3x3)
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2  >
		inline int _inverse_3x3_( MTRX1 *mt, MTRX2 *inv)
		{
			double det;

			gnd::matrix::_det_3x3_(mt, &det);
			if( det == 0 )	return -1;

			_gnd_matrix_ref_(inv, 0, 0) = ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 1) ) / det;
			_gnd_matrix_ref_(inv, 0, 1) = ( _gnd_matrix_ref_(mt, 0, 2) * _gnd_matrix_ref_(mt, 2, 1) - _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 2, 2) ) / det;
			_gnd_matrix_ref_(inv, 0, 2) = ( _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 1, 2) - _gnd_matrix_ref_(mt, 0, 2) * _gnd_matrix_ref_(mt, 1, 1) ) / det;

			_gnd_matrix_ref_(inv, 1, 0) = ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 2) ) / det;
			_gnd_matrix_ref_(inv, 1, 1) = ( _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 2, 2) - _gnd_matrix_ref_(mt, 0, 2) * _gnd_matrix_ref_(mt, 2, 0) ) / det;
			_gnd_matrix_ref_(inv, 1, 2) = ( _gnd_matrix_ref_(mt, 0, 2) * _gnd_matrix_ref_(mt, 1, 0) - _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 1, 2) ) / det;

			_gnd_matrix_ref_(inv, 2, 0) = ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 0) ) / det;
			_gnd_matrix_ref_(inv, 2, 1) = ( _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 2, 0) - _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 2, 1) ) / det;
			_gnd_matrix_ref_(inv, 2, 2) = ( _gnd_matrix_ref_(mt, 0, 0) * _gnd_matrix_ref_(mt, 1, 1) - _gnd_matrix_ref_(mt, 0, 1) * _gnd_matrix_ref_(mt, 1, 0) ) / det;

			return 0;
		}




		/**
		 * @brief Compute Invert Matrix (4x4)
		 * @param [in]  mt  : Matrix (4x4)
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2  >
		inline int _inverse_4x4_( MTRX1 *mt, MTRX2 *inv)
		{
			double det;

			_det_4x4_(mt, &det);


			_gnd_matrix_ref_(inv, 0, 0) = (  _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) )
			) / det;
			_gnd_matrix_ref_(inv, 0, 1) = (  _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) )
			) / det;
			_gnd_matrix_ref_(inv, 0, 2) = (  _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 1) )
			) / det;
			_gnd_matrix_ref_(inv, 0, 3) = (  _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 3) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 1) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 2) )
			) / det;

			_gnd_matrix_ref_(inv, 1, 0) = (  _gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) )
					+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) )
			) / det;
			_gnd_matrix_ref_(inv, 1, 1) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) )
			) / det;
			_gnd_matrix_ref_(inv, 1, 2) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 0) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 2) )
			) / det;
			_gnd_matrix_ref_(inv, 1, 3) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 2) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 3) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 0) )
			) / det;

			_gnd_matrix_ref_(inv, 2, 0) = (  _gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) )
					+ _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 1, 3) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) )
			) / det;
			_gnd_matrix_ref_(inv, 2, 1) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 2, 3) * _gnd_matrix_ref_(mt, 3, 0) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) )
			) / det;
			_gnd_matrix_ref_(inv, 2, 2) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 1) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 3) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 0) )
			) / det;
			_gnd_matrix_ref_(inv, 2, 3) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 3) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 3) - _gnd_matrix_ref_(mt, 1, 3) * _gnd_matrix_ref_(mt, 2, 0) )
					+ _gnd_matrix_ref_(mt, 0, 3) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 1) )
			) / det;

			_gnd_matrix_ref_(inv, 3, 0) = (  _gnd_matrix_ref_(mt, 1, 0) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 1, 1) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) )
					+ _gnd_matrix_ref_(mt, 1, 2) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) )
			) / det;
			_gnd_matrix_ref_(inv, 3, 1) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 1) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 2, 2) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 2, 0) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 2, 1) * _gnd_matrix_ref_(mt, 3, 0) )
			) / det;
			_gnd_matrix_ref_(inv, 3, 2) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 2) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 3, 0) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 3, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 3, 1) )
			) / det;
			_gnd_matrix_ref_(inv, 3, 3) = (  _gnd_matrix_ref_(mt, 0, 0) * ( _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 2) - _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 1) )
					+ _gnd_matrix_ref_(mt, 0, 1) * ( _gnd_matrix_ref_(mt, 1, 2) * _gnd_matrix_ref_(mt, 2, 0) - _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 2) )
					+ _gnd_matrix_ref_(mt, 0, 2) * ( _gnd_matrix_ref_(mt, 1, 0) * _gnd_matrix_ref_(mt, 2, 1) - _gnd_matrix_ref_(mt, 1, 1) * _gnd_matrix_ref_(mt, 2, 0) )
			) / det;

			return 0;
		}


		/**
		 * @brief Compute Invert Matrix
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX1, typename MTRX2, typename MTRX3 >
		inline int inverse(const MTRX1 *mt, MTRX2 *ws, MTRX3 *inv)
		{
			gnd_assert((!mt || !ws || !inv), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(mt), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(ws), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(inv), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(mt) > _gnd_matrix_row_(ws)), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(mt) > _gnd_matrix_column_(ws)), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(mt) > _gnd_matrix_row_(inv)), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(mt) > _gnd_matrix_column_(inv)), -1, "out of buffer");


			{ // ---> operation
				uint32_t i, j, k;
				uint32_t ipivot;
				uint32_t n = (_gnd_matrix_row_(mt) < _gnd_matrix_column_(mt)) ? _gnd_matrix_row_(mt) : _gnd_matrix_column_(mt);

				// explicit case
				if(n == 2){
					return _inverse_2x2_(mt, inv);
				}
				else if(n == 3){
					return _inverse_3x3_(mt, inv);
				}
				else if(n == 4){
					return _inverse_4x4_(mt, inv);
				}

				{ // ---> Initialize
					gnd_assert(( copy(ws, mt) < 0 ), -1, "failed");
					gnd_assert(( set_unit(inv) < 0 ), -1, "failed");
				} // <--- Initialize

				for(i = 0; i < n; i++){
					{ // ---> pivot
						component_t max = fabs(_gnd_matrix_ref_(ws, i, i));
						component_t abs_ji;

						ipivot = i;
						for(j = i + 1; j < n; j++){
							abs_ji = fabs(_gnd_matrix_ref_(ws, j, i));
							if(max < abs_ji){
								max = abs_ji;
								ipivot = j;
							}
						}
						if(ipivot != i){
							gnd::matrix::swap_row(ws, i, ipivot);
							gnd::matrix::swap_row(inv, i, ipivot);
						}
					} // <--- pivot

					{
						component_t aii;
						aii = _gnd_matrix_ref_(ws, i, i);

						// check error; Inverse is not exist.
						if(aii == 0)	return 1;

						for(j = 0; j < n; j++){
							_gnd_matrix_ref_(ws, i, j) /= aii;
							_gnd_matrix_ref_(inv, i, j) /= aii;
						} // for(j)
						for(j = 0; j < n; j++){
							component_t aji;
							if(j == i)	continue;
							aji = _gnd_matrix_ref_(ws, j, i);
							for(k = 0; k < n; k++){
								_gnd_matrix_ref_(ws, j, k) -= aji * _gnd_matrix_ref_(ws, i, k);
								_gnd_matrix_ref_(inv, j, k) -= aji * _gnd_matrix_ref_(inv, i, k);
							} // for(k)
						} // for(j)
					}
				} // for(i)
			} // <--- operation
			return 0;
		}


		/**
		 * @brief Compute Invert Matrix
		 * @param [in]  mt  : Matrix
		 * @param [in]  ws  : Work Space
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < uint32_t R, uint32_t C, typename MTRX >
		inline int inverse(const fixed<R, C> *mt, MTRX *inv)
		{
			gnd_assert((!mt || !inv), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(inv), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(mt) > _gnd_matrix_row_(inv)), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(mt) > _gnd_matrix_column_(inv)), -1, "out of buffer");

			{ // ---> operation
				fixed<R, C> ws;

				return inverse(mt, &ws, inv);
			} // <--- operation
		}

		/**
		 * @brief Compute Invert Matrix (explicit 2x2)
		 * @param [in]  mt  : Matrix (2x2)
		 * @param [in]  ws  : Work Space
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int inverse(const fixed<2, 2> *mt, MTRX *inv)
		{
			gnd_assert((!mt || !inv), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(inv), -1, "invalid matrix property");

			return _inverse_2x2_(mt, inv);
		}

		/**
		 * @brief Compute Invert Matrix (explicit 3x3)
		 * @param [in]  mt  : Matrix (3x3)
		 * @param [in]  ws  : Work Space
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int inverse(const fixed<3, 3> *mt, MTRX *inv)
		{
			gnd_assert((!mt || !inv), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(inv), -1, "invalid matrix property");

			return gnd::matrix::_inverse_3x3_(mt, inv);
		}

		/**
		 * @brief Compute Invert Matrix (explicit 4x4)
		 * @param [in]  mt  : Matrix (4x4)
		 * @param [in]  ws  : Work Space
		 * @param [out] inv : Inverse Matrix
		 * @return ==0 : success
		 * @return ==1 :
		 * @return  <0 : fail
		 */
		template < typename MTRX >
		inline int inverse(const fixed<4, 4> *mt, MTRX *inv)
		{
			gnd_assert((!mt || !inv), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(inv), -1, "invalid matrix property");

			return gnd::matrix::_inverse_4x4_(mt, inv);
		}




		template < typename MTRX1, typename MTRX2 >
		inline int submatrix_inner_prod(const MTRX1 *m1, const uint32_t r1, const uint32_t c1,
				const MTRX2 *m2, const uint32_t r2, const uint32_t c2, const uint32_t d, double *v)
		{
			gnd_assert((!m1 || !m2 || !v), -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m1), -1, "invalid matrix property");
			gnd_assert(!gnd_matrix_is_avail(m2), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(m1) < r1), -1, "out of buffer");
			gnd_assert((_gnd_matrix_row_(m2)  < r2), -1, "out of buffer");
			gnd_assert(!d, 0, "this argument value have no effect");

			{ // ---> compute inner product
				uint32_t i;
				*v = 0;
				for(i = 0; i < d; i++)
					*v += _gnd_matrix_ref_(m1, r1, c1 + i) * _gnd_matrix_ref_(m2, r2, c2 + i);
			} // <--- compute inner product

			return 0;
		}


		template < typename MTRX1, typename MTRX2 >
		inline int inner_prod(const MTRX1 *m1, const MTRX2 *m2, double *v)
		{
			const uint32_t n = (_gnd_matrix_column_(m1) < _gnd_matrix_column_(m2)) ?
					_gnd_matrix_column_(m1) : _gnd_matrix_column_(m2);
			return gnd::matrix::submatrix_inner_prod(m1, 0, 0, m2, 0, 0, n, v);
		}



		/**
		 * @brief set rot (axis x)
		 * @param [out] m : matrix
		 * @param  [in] r : angle
		 */
		template < typename MTRX >
		inline int set_rot_x( MTRX *m, double r )
		{
			gnd_assert(!m, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(m) < 3), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m)  < 3), -1, "out of buffer");

			{ // ---> operation
				double sinv = sin(r), cosv = cos(r);
				gnd_matrix_set_unit(m);

				_gnd_matrix_ref_(m, 1, 1) =   cosv;
				_gnd_matrix_ref_(m, 1, 2) = - sinv;
				_gnd_matrix_ref_(m, 2, 1) =   sinv;
				_gnd_matrix_ref_(m, 2, 2) =   cosv;
			} // <--- operation
			return 0;
		}


		/**
		 * @brief set rot (axis y)
		 * @param [out] m : matrix
		 * @param  [in] r : angle
		 */
		template < typename MTRX >
		inline int set_rot_y(MTRX *m, double r)
		{
			gnd_assert(!m, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(m) < 3), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m)  < 3), -1, "out of buffer");

			{ // ---> operation
				double sinv = sin(r), cosv = cos(r);
				gnd_matrix_set_unit(m);

				_gnd_matrix_ref_(m, 0, 0) =   cosv;
				_gnd_matrix_ref_(m, 0, 2) =   sinv;
				_gnd_matrix_ref_(m, 2, 0) = - sinv;
				_gnd_matrix_ref_(m, 2, 2) =   cosv;
			} // <--- operation
			return 0;

		}



		/**
		 * @brief set rot (axis x)
		 * @param [out] m : matrix
		 * @param  [in] r : angle
		 */
		template < typename MTRX >
		inline int set_rot_z( MTRX *m, double r )
		{
			gnd_assert(!m, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(m), -1, "invalid matrix property");
			gnd_assert((_gnd_matrix_row_(m) < 2), -1, "out of buffer");
			gnd_assert((_gnd_matrix_column_(m)  < 2), -1, "out of buffer");

			{ // ---> operation
				double sinv = sin(r), cosv = cos(r);
				gnd_matrix_set_unit(m);

				_gnd_matrix_ref_(m, 0, 0) =   cosv;
				_gnd_matrix_ref_(m, 0, 1) = - sinv;
				_gnd_matrix_ref_(m, 1, 0) =   sinv;
				_gnd_matrix_ref_(m, 1, 1) =   cosv;
			} // <--- operation
			return 0;

		}





		/**
		 * @brief show
		 * @param [in] fp : output stream
		 * @param [in] mt : matrix
		 */
		template < typename MTRX >
		inline int show(FILE* fp, const MTRX *mt, const char *f = 0, const char *h = 0)
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!fp, -1, "null pointer");

			{
				char fmt[16];
				uint32_t i, j;

				sprintf(fmt, "%s ", f == 0 ? "%lf" : f);
				for(i = 0; i < _gnd_matrix_row_(mt); i++){
					if(h) ::fprintf(fp, "%s", h);
					for(j = 0; j < _gnd_matrix_column_(mt); j++)
					{	::fprintf(fp, fmt, _gnd_matrix_ref_(mt, i, j));	}
					::fprintf(fp, "\n");
				}
			}
			return 0;
		}

		/**
		 * @brief show
		 * @param [in] fp : output stream
		 * @param [in] mt : transposed matrix
		 */
		template < typename MTRX >
		inline int show_transpose(FILE* fp, const MTRX *mt, const char *f = 0, const char *h = 0)
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!fp, -1, "null pointer");

			{
				char fmt[16];
				uint32_t i, j;

				sprintf(fmt, "%s ", f == 0 ? "%lf" : f);
				for(i = 0; i < _gnd_matrix_column_(mt); i++){
					if(h) ::fprintf(fp, "%s", h);
					for(j = 0; j < _gnd_matrix_row_(mt); j++)
					{	::fprintf(fp, fmt, _gnd_matrix_ref_(mt, j, i));	}
					::fprintf(fp, "\n");
				}
			}
			return 0;
		}



		/**
		 * @brief show property
		 * @param [in] fp : output stream
		 * @param [in] mt : matrix
		 */
		template < typename MTRX >
		inline int show_prop(FILE* fp, const MTRX *mt)
		{
			gnd_assert(!mt, -1, "null pointer");
			gnd_assert(!fp, -1, "null pointer");

			fprintf(fp, "row %d x column %d\n", _gnd_matrix_row_(mt), _gnd_matrix_column_(mt));
			return 0;
		}




		/**
		 * @ingroup GNDMatrix
		 * @brief dummy constructor
		 */
		inline
		flex::flex()
		{
			initialize(this);
		}

		/**
		 * @ingroup GNDMatrix
		 * @brief indexer like double pointer
		 * @param[in] i : row index
		 * @return row head pointer
		 */
		inline
		double* flex::operator[](int i)
		{
			gnd_assert(i < 0 || i > (signed)_gnd_matrix_row_(this), 0, "out of buffer");
			return _gnd_matrix_pointer_(this, i, 0);
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief dummy constructor
		 */
		template<uint32_t R, uint32_t C>
		inline
		fixed<R,C>::fixed()
		{
			set_zero(this);
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief constructor
		 * @param[in] src : source data
		 * @param[in]   l : length of source data
		 */
		template<uint32_t R, uint32_t C>
		inline
		fixed<R,C>::fixed(const component_t *src, uint32_t l)
		{
			gnd_assert_void(!src,  "invalid null argument");
			gnd_assert_void(l > R * C,  "out of buffer");

			{ // ---> operation
				uint32_t n = l < R*C ? l : R*C;
				::memcpy(data, src, n * sizeof(double));
			} // <--- operation
		}

		/**
		 * @ingroup GNDMatrix
		 * @brief indexer like double pointer
		 * @param[in] i : row index
		 * @return row head pointer
		 */
		template<uint32_t R, uint32_t C>
		inline
		double* fixed<R, C>::operator[](int i)
		{
			gnd_assert(i < 0 || i > (signed)_gnd_matrix_row_(this), 0, "out of buffer");
			return _gnd_matrix_pointer_(this, i, 0);
		}

	} // <--- namespace matrix
} // <--- namespace gnd
// <--- function definition


#endif /* GND_MATRIX_BASE_H_ */


