/*
 * gnd_vector_base.hpp
 *
 *  Created on: 2011/06/12
 *      Author: tyamada
 */

#ifndef GND_VECTOR_BASE_HPP_
#define GND_VECTOR_BASE_HPP_

#include "gnd-matrix-base.hpp"


// ---> const definition
namespace gnd {
	namespace vector { // ---> namespace vector

		enum {
			INVALID = -1,
			SINGLE = 0,
			RAW,
			COLUMN,
		};

	}
} // <--- const definition


#define gnd_vector_fixed gnd::matrix::fixed

#define _gnd_vector_row_(vct)			_gnd_matrix_row_(vct)
#define _gnd_vector_column_(vct)			_gnd_matrix_column_(vct)
#define _gnd_vector_size_(vct)			( _gnd_matrix_row_(vct) * _gnd_matrix_column_(vct) )

#define gnd_vector_state(vct)	\
		( (_gnd_matrix_raw_(vct) == 0 || _gnd_matrix_column_(vct) == 0) ? gnd::vector::INVALID :	\
		(	_gnd_matrix_raw_(vct) == 1 && _gnd_matrix_column_(vct) == 1) ? gnd::vector::SINGLE :	\
			_gnd_matrix_raw_(vct) != 1 ? gnd::vector::RAW : 										\
			_gnd_matrix_column_(vct) != 1 ? gnd::vector::COLUMN : gnd::vector::INVALID				\
		)

#define gnd_vector_is_column(vct)	\
		( _gnd_matrix_column_(vct) > 1 && _gnd_matrix_row_(vct) == 1 ? true : false)
#define gnd_vector_is_row(vct)	\
		( _gnd_matrix_row_(vct) > 1 && _gnd_matrix_column_(vct) == 1 ? true : false)


#define gnd_vector_exist(vct, i)	\
		( _gnd_vector_size_(vct) > i )

#define _gnd_vector_pointer_(vct, i)		((vct)->data + i)
#define _gnd_vector_ref_(vct, i)			(*_gnd_vector_pointer_(vct, i))



// ---> type definition
namespace gnd {
	namespace vector { // ---> namespace vector

		typedef gnd::matrix::component_t component_t;

		typedef struct gnd::matrix::flex flex;


		/**
		 * @ingroup GNDMatrix
		 * @brief fixed size column vector
		 * @tparam R : row size
		 */
		template < const uint32_t R >
		struct fixed_column : matrix::fixed<R, 1> {
			double& operator[](int i);
			fixed_column();
		};


		/**
		 * @ingroup GNDMatrix
		 * @brief fixed size column vector
		 * @tparam R : row size
		 */
		template < const uint32_t C >
		struct fixed_row : matrix::fixed<1, C> {
			double& operator[](int i);
			fixed_row();
		};

}
} // <--- type definition

namespace gnd { // ---> namespace gnd

	namespace matrix { // ---> namespace matrix
		/**
		 * @brief assign mt with buffer
		 * @param [out] dest : buffer assigned matrix
		 * @param  [in]  src : buffer source matrix
		 * @param  [in]    r : row index
		 * @param  [in]    c : column index
		 * @param  [in]    s : size
		 */
		template < typename VCT >
		inline
		int assign_to_as_vector ( vector::flex *dest, VCT *src, const size_t r, const size_t c, const size_t s)
		{
			gnd_assert(!dest || !src, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix_property");
			gnd_assert((s == 0), -1, "invalid argument value");
			gnd_assert(_gnd_matrix_row_(src) < r, -1, "out of buffer");
			gnd_assert(_gnd_matrix_column_(src) < c + s, -1, "out of buffer");

			return gnd::matrix::assign(dest, _gnd_matrix_pointer_(src, r, c), 1, s);
		}
	} // <--- namespace matrix


	namespace vector { // ---> namespace vector


		/**
		 * @ingroup GNDMatrix
		 * @brief dummy constructor
		 */
		template< uint32_t R >
		inline
		fixed_column<R>::fixed_column()
		{
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief indexer like double pointer
		 * @param[in] i : row index
		 * @return row head pointer
		 */
		template< uint32_t R >
		inline
		double& fixed_column<R>::operator[](int i)
		{
			return _gnd_matrix_ref_(this, i, 0);
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief dummy constructor
		 */
		template< uint32_t C >
		inline
		fixed_row<C>::fixed_row()
		{
		}


		/**
		 * @ingroup GNDMatrix
		 * @brief indexer like double pointer
		 * @param[in] i : row index
		 * @return row head pointer
		 */
		template< uint32_t C >
		inline
		double& fixed_row<C>::operator[](int i)
		{
			return _gnd_matrix_ref_(this, 0, i);
		}



		/**
		 * @brief initialize
		 * @param [in/out]	vec : vector
		 */
		inline
		int init ( flex *vec )
		{
			return matrix::clear(vec);
		}

		/**
		 * @brief assign vec with buffer
		 * @param [in/out]	vec : assign vector
		 * @param     [in] buf : buffer
		 * @param     [in]  s : size
		 */
		inline
		int assign ( flex *vec, component_t *buf, const size_t s)
		{
			return matrix::assign(vec, buf, 1, s);
		}


		/**
		 * @brief assign mt with buffer
		 * @param [out] dest : buffer assigned matrix
		 * @param  [in]  src : buffer source matrix
		 * @param  [in]    i : index
		 * @param  [in]    s : size
		 */
		template < typename VCT >
		inline
		int assign_to ( flex *dest, VCT *src, const size_t i, const size_t s)
		{
			gnd_assert(!dest || !src, -1, "null pointer");
			gnd_assert(!gnd_matrix_is_avail(src), -1, "invalid matrix_property");
			gnd_assert(gnd_matrix_is_avail(dest), -1, "destination vector have been already assigned");
			gnd_assert((s == 0), -1, "invalid argument value");
			gnd_assert(_gnd_vector_size_(src) < i + s, -1, "out of buffer");

			return matrix::assign(dest, _gnd_vector_pointer_(src, i), 1, s);
		}



		/**
		 * @brief release assigned buffer
		 * @param [in/out]	vec : assigned vector
		 */
		inline
		int release ( flex *vec )
		{
			return matrix::release(vec);
		}




		/**
		 * @brief allocate buffer from heap
		 * @param [in/out] vec : vector
		 * @param     [in]  s : size
		 */
		inline
		int allocate ( flex *vec, const size_t s )
		{
			return matrix::allocate(vec, 1, s);
		}


		/**
		 * @brief deallocate buffer
		 * @param [in/out] vec : vector
		 * @param     [in]  s : size
		 */
		inline
		int deallocate ( flex *vec )
		{
			return matrix::deallocate(vec);
		}



		// -----------------------> ref
		/**
		 * @brief get size
		 * @param [in] vec : vector
		 */
		template< typename VCT >
		inline
		int size ( const VCT *vec )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			return _gnd_vector_size_(vec);
		}



		/**
		 * @brief get pointer
		 * @param [in] vec : vector
		 * @param [in]   i : index
		 * @return element pointer
		 */
		template < typename VCT >
		inline
		component_t* pointer( VCT  *vec, const size_t i )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_assert(!gnd_vector_exist(vec, i), 0, "out of buffer");
			return _gnd_vector_pointer_(vec, i);
		}


		/**
		 * @brief get const pointer
		 * @param [in] vec : vector
		 * @param [in]  i : index
		 * @return element const pointer
		 */
		template < typename VCT >
		inline
		const component_t* pointer_const ( const VCT *vec, const size_t i )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_assert(!gnd_vector_exist(vec, i), 0, "out of buffer");
			return static_cast<const component_t*>(_gnd_vector_pointer_(vec,i));
		}

		/**
		 * @brief get value
		 * @param [in] vec : vector
		 * @param [in]  i : index
		 * @return element value
		 */
		template < typename VCT >
		inline
		component_t value( const VCT *vec, const size_t i)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_assert(!gnd_vector_exist(vec, i), 0, "out of buffer");

			return _gnd_vector_ref_(vec,i);
		}


		// ------------------------------> getter
		/**
		 * @brief get value
		 * @param  [in] vec : vector
		 * @param  [in]  i : index
		 * @param [out]  v : destination
		 */
		template < typename VCT >
		inline
		int get( const VCT *vec, const size_t i, component_t *v )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_assert(!gnd_vector_exist(vec, i), 0, "out of buffer");

			if(gnd_vector_is_row(vec))
				return matrix::get(vec, 0, i, v);
			else
				return matrix::get(vec, i, 0, v);
		}



		// ------------------------------> setter
		/**
		 * @brief set value
		 * @param  [in] vec : vector
		 * @param  [in]  i : index
		 * @param [out]  v : src
		 */
		template < typename VCT >
		inline
		int set( const VCT *vec, const size_t i, const component_t v )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_vector_state(vec) != INVALID, "is not vector");
			gnd_assert(!gnd_vector_exist(vec, i), 0, "out of buffer");

			if(gnd_vector_is_raw(vec))
				return matrix::set(vec, 0, i, v);
			else
				return matrix::set(vec, i, 0, v);
		}


		/**
		 * @brief set uniform value in the whole element
		 * @param [in/out] vec : vector
		 * @param     [in]  v : value
		 */
		template < typename VCT >
		inline
		int set_uniform ( VCT *vec, const component_t v )
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			return matrix::set_uniform(vec, v);
		}


		/**
		 * @brief set 0 in the whole element
		 * @param [in/out] vec : vector
		 */
		template < typename VCT >
		inline
		int set_zero( VCT  *vec)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			return matrix::set_zero(vec);
		}


		// -----------> copy

		/**
		 * @brief sub matix copy
		 * @param [out] dest : destination
		 * @param  [in]  src : source
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int copy( VCT1  *dest, const VCT2  *src)
		{
			gnd_assert(!src || !dest, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(src), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(dest), "is not vector");
			return matrix::copy(dest, src);
		}

		/**
		 * @brief swap row
		 * @param [in/out] vec	: matrix
		 * @param [in]	i 		: index 1
		 * @param [in]	j 		: index 2
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT >
		inline
		int swap( VCT * vec, const size_t i, const size_t j)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");

			if(gnd_vector_is_raw(vec))
				return matrix::swap_column(vec, i, j);
			else
				return matrix::swap_row(vec, i, j);
		}



		// ---> operation

		/**
		 * @brief sub vector addition
		 * @param  [in]   v1 : vector1
		 * @param  [in]   i1 : index vector1
		 * @param  [in]   v2 : vector2
		 * @param  [in]   i2 : index vector2
		 * @param  [in]    s : sub-vector size
		 * @param [out]    o : output buffer
		 * @param  [in]   io : index output buffer
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2, typename VCT3 >
		inline
		int subvector_add(const VCT1 *v1, const size_t i1,
				const VCT2 *v2, const size_t i2,
				const size_t s,
				VCT3 *o, const size_t io)
		{
			gnd_assert(!v1 || !v2 || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(v1), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(v2), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::submatrix_add(v1, 0, i1, v2, 0, i2, o, io, 1, s);
		}


		/**
		 * @brief addition
		 * @param  [in]   v1 : vector1
		 * @param  [in]   v2 : vector2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename VCT1, typename VCT2, typename VCT3>
		inline
		int add(const VCT1 *v1, const VCT2 *v2, VCT3 *o)
		{
			gnd_assert(!v1 || !v2 || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(v1), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(v2), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::add(v1, v2, o);
		}


		/**
		 * @brief sub vector subtraction
		 * @param  [in]   v1 : vector1
		 * @param  [in]   i1 : index vector1
		 * @param  [in]   v2 : vector2
		 * @param  [in]   i2 : index vector2
		 * @param [out]    o : output buffer
		 * @param  [in]   io : index output buffer
		 * @param  [in]    s : copy size
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2, typename VCT3 >
		inline
		int subvector_sub(const VCT1 *v1, const size_t i1,
				const VCT2 *v2, const size_t i2,
				const size_t s,
				VCT3 *o, const size_t io)
		{
			gnd_assert(!v1 || !v2 || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(v1), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(v2), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return submatrix_sub(v1, 0, i1, v2, 0, i2, o, io, 1, s);
		}


		/**
		 * @brief subtraction
		 * @param  [in]   v1 : vector1
		 * @param  [in]   v2 : vector2
		 * @param [out]    o :  o
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename VCT1, typename VCT2, typename VCT3>
		inline
		int sub(const VCT1 *v1, const VCT2 *v2, VCT3 *o)
		{
			gnd_assert(!v1 || !v2 || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(v1), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(v2), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::sub(v1, v2, o);
		}



		// ---> scalar prod
		/**
		 * @brief product
		 * @param [in] vec : vector 1
		 * @param [in]   v : v
		 * @param [in]   i : index
		 * @param [in]   l : length
		 * @param [out]  o : product result
		 * @param [in]  io : output index
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int subvector_scalar_prod(const VCT1 *vec, const size_t i, const size_t l, const double v,
				VCT2 *o, const size_t io)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::submatrix_scalar_prod(vec, 0, i,
					v,
					1, l,
					o, 0, io);
		}

		/**
		 * @brief product
		 * @param [in] vec : vector 1
		 * @param [in]  v : v
		 * @param [out] o : product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int scalar_prod(const VCT1 *vec, const double v, VCT2 *o)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::scalar_prod(vec, v, o);
		}



		/**
		 * @brief sub-vector scalar divide
		 * @param [in] vec : vector 1
		 * @param [in]   v : v
		 * @param [in]   i : index
		 * @param [in]   l : length
		 * @param [out]  o : product result
		 * @param [in]  io : output index
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int subvector_scalar_div(const VCT1 *vec, const size_t i, const size_t l, const double v,
				VCT2 *o, const size_t io)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return submatrix_scalar_div(vec, 0, i,
					v,
					1, l,
					o, 0, io);
		}

		/**
		 * @brief scalar divide
		 * @param [in] vec : vector 1
		 * @param [in]  v : v
		 * @param [out] o : product result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template <typename VCT1, typename VCT2>
		inline
		int scalar_div(const VCT1 *vec,
				const double v, VCT2 *o)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");
			return matrix::scalar_div(vec, v, o);
		}



		// ---> norm
		/**
		 * @brief Compute Norm (sub-vector row)
		 * @param [in]  vec : vector
		 * @param [in]    i : sub-vector start index
		 * @param [in]    l : sub-vector length
		 * @param [out]   v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT >
		inline
		int subvector_sqnorm(const VCT *vec, const size_t i, const size_t l, double *v)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			if(gnd_vector_is_raw(vec))
				return submatrix_sqnorm_row(vec, 0, i, l, v);
			else
				return submatrix_sqnorm_column(vec, i, 0, l, v);
		}

		/**
		 * @brief Compute Norm (sub-vector row)
		 * @param [in]  vec : vector
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT >
		inline
		int sqnorm(const VCT *vec,  double *v)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			return gnd::matrix::sqnorm_row(vec, 0, v);
		}



		/**
		 * @brief Compute Norm
		 * @param [in]  vec : vector
		 * @param [in]    i : sub-vector start index
		 * @param [in]    l : sub-vector length
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT >
		inline
		int subvector_norm(const VCT *vec, const size_t i, const size_t l, double *v)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			if(gnd_vector_is_row(vec))
				return submatrix_norm_row(vec, 0, i, l, v);
			else
				return submatrix_norm_column(vec, i, 0, l, v);
		}



		/**
		 * @brief Compute Norm
		 * @param [in]  vec : vector
		 * @param [out]  v : result
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT >
		inline
		int norm(const VCT *vec,  double *v)
		{
			gnd_assert(!vec, 0, "null pointer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			if(gnd_vector_is_raw(vec))
				return norm_row(vec, v);
			else
				return norm_column(vec, v);
		}


		// ---> unit vector
		/**
		 * @brief Compute unit vector
		 * @param [in]  vec : vector
		 * @param [in]    i : sub-vector start index
		 * @param [in]    l : sub-vector length
		 * @param [out]   o : unit vector
		 * @param [in]	 io : output index
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int subvector_unit(const VCT1 *vec, const size_t i, const size_t l,
				VCT2 *o, const size_t io)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			gnd_assert(!gnd_vector_exist(vec, i + l - 1), -1, "out of buffer");
			gnd_assert(!gnd_vector_exist(o, io + l - 1), -1, "out of buffer");
			gnd_warnning(!gnd_matrix_is_vector(vec), "is not vector");
			gnd_warnning(!gnd_matrix_is_vector(o), "is not vector");


			{ // ---> operation
				int fres;
				double v;

				if( (fres = subvector_norm(vec, i, l, &v)) < 0 )	return fres;
				if(v == 0){
					gnd_warnning(v == 0, "cannot compute because of vec is zero-vector.");
					return -1;
				}
				return subvector_scalar_div(vec, i, l, v, o, io);

			} // <--- operation
		}



		/**
		 * @brief Compute unit vector
		 * @param [in]  vec : vector
		 * @param [out]   o : unit vector
		 * @return ==0 : success
		 * @return  <0 : fail
		 */
		template < typename VCT1, typename VCT2 >
		inline
		int unit(const VCT1 *vec, VCT2 *o)
		{
			gnd_assert(!vec || !o, 0, "null pointer");
			{ // ---> operation
				const size_t s = _gnd_vector_size_(vec) < _gnd_vector_size_(o) ? _gnd_vector_size_(vec) : _gnd_vector_size_(o);
				return subvector_unit(vec, 0, s, o, 0);
			} // <--- operation

		}




		/**
		 * @brief show
		 * @param [in]  fp : output stream
		 * @param [in] vec : vector
		 */
		template < typename VCT >
		inline int show(FILE* fp, const VCT  *vec, const char *f)
		{
			return matrix::show(fp, vec, f);
		}


		/**
		 * @brief show property
		 * @param [in]  fp : output stream
		 * @param [in] vec : vector
		 */
		template < typename VCT >
		inline int show_prop(FILE* fp, const VCT *vec)
		{
			return matrix::show_prop(fp, vec);
		}

	} // <--- namespace vector

} // <--- namespace gnd

#endif /* YP_VECTOR_BASE_HPP_ */


