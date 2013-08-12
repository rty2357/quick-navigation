/*
 * gnd_gridmap.hpp
 *
 *  Created on: 2011/08/09
 *      Author: tyamada
 */

#ifndef GND_GRIDMAP_HPP_
#define GND_GRIDMAP_HPP_

#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "gnd-wrap-sys.hpp"

// include debug logging function
#define GND_DEBUG_LOG_NAMESPACE1 gnd
#define GND_DEBUG_LOG_NAMESPACE2 gridmap
#include "gnd-debug-log.hpp"
#undef GND_DEBUG_LOG_NAMESPACE2
#undef GND_DEBUG_LOG_NAMESPACE1
#undef GND_DEBUG_LOG
#include "gnd-debug-log-util-def.h"

/**
 * @ifnot GNDStorage
 * @defgroup GNDStorage storage
 * @endif
 */

/**
 * @defgroup GNDGridMap grid-map
 * @ingroup GNDStorage
 */

// ---> constant value definition
namespace gnd { // ---> namespace gnd
	namespace gridmap { // ---> namespace GridMap
		/** @var __GridMapFileTag__
		 * @ingroup GNDGridMap
		 * @brief file tag
		 */
		static const char __GridMapFileTag__[] = "GGRD";
		/** @var __GridPlaneFileTag__
		 * @ingroup GNDGridMap
		 * @brief file tag
		 */
		static const char __GridPlaneFileTag__[] = "GPLN";
		/** @var __FileTagSize__
		 * @ingroup GNDGridMap
		 * @brief file tag byte size
		 */
		static const int32_t __FileTagSize__ = ::strlen(__GridMapFileTag__);
	} // <--- namespace GridMap
} // <--- namespace gnd
// <--- constant value definition


// ---> type definition
namespace gnd { // ---> namespace gnd
	namespace gridmap { // ---> namespace GridMap
		/**
		 * @ingroup GNDGridMap
		 * @brief index of pixel
		 */
		typedef struct pixelindex {
			/// @brief row index
			uint32_t row;
			/// @brief column index
			uint32_t column;
		} pixelindex;
		template < typename T>
		class basic_gridmap;
		template < typename T>
		class gridplane;
	}
}
// <--- type definition




// ---> class definition
namespace gnd { // ---> namespace gnd
	namespace gridmap { // ---> namespace GridMap

		/**
		 * @ingroup GNDGridMap
		 * @brief grid-map basic class
		 * @tparam T : stored data type in a pixel
		 */
		template < typename T>
		class basic_gridmap {
		public:
			/// @brief pointer type
			typedef T*		pt;
			/// @brief double pointer type
			typedef T**		wpt;
			/// @brief triple pointer type
			typedef T***	tpt;

			typedef T compoment;

			// ---> constructor, destructor
		public:
			basic_gridmap();
			~basic_gridmap();
			// <--- constructor, destructor

			// ---> variables
		protected:
			/// @brief data storage header pointer
			tpt _header;
			/// @brief number of memory unit
			gridmap::pixelindex _unit;
			/// @brief number of plane
			gridmap::pixelindex _plane;
			/// @brief flag of auto allocation
			bool autoreallocate;
			// <--- variables

			// ---> allocate, deallocate
		public:
			virtual int allocate(const uint32_t r, const uint32_t c);
			virtual int allocate(const uint32_t ur, const uint32_t uc, const uint32_t pr, const uint32_t pc);
			virtual int deallocate();
			virtual bool is_allocate();
			virtual int reallocate(const unsigned long sr, const unsigned long sc,
					const unsigned long hr, const unsigned long hc);
		public:
			int set_autoreallocate(bool flg);
			// <--- allocate, deallocate

			// ---> setter, getter
		public:
			/**
			 * @brief get pixel pointer
			 * @param[in] r: pixel row index
			 * @param[in] c: pixel column index
			 */
			pt pointer(const unsigned long r, const unsigned long c);
			/**
			 * @brief get plane pointer
			 * @param[in] r: plane row index
			 * @param[in] c: plane column index
			 */
			pt blockheader(const uint32_t r, const uint32_t c);
			/**
			 * @brief get pixel value
			 * @param[in] r: pixel row index
			 * @param[in] c: pixel column index
			 */
			T value(const unsigned long r, const unsigned long c);
			unsigned long row();
			unsigned long column();
			int get(const unsigned long r, const unsigned long c, pt v);
			int set(const unsigned long r, const unsigned long c, const pt v);

			int set_uniform(const T v);
			int set_uniform(const T* v);

			uint32_t _unit_row_();
			uint32_t _unit_column_();

			uint32_t _plane_row_();
			uint32_t _plane_column_();



		private:
			uint32_t memory_unit_row(unsigned long r);
			uint32_t memory_unit_column(unsigned long c);
			// <--- setter, getter


			// ---> read write
		public:
			virtual int fwrite(const char *fname);
			virtual int fread(const char *fname);

			// <--- read write

		};






		// ----------------------------------------> constructor, destructor
		/**
		 * @brief constructor
		 */
		template< typename T >
		inline
		basic_gridmap<T>::basic_gridmap() : _header(0)
		{
			_unit.row = 0;
			_unit.column = 0;

			_plane.row = 0;
			_plane.column = 0;
		}

		/**
		 * @brief destructor
		 */
		template< typename T >
		inline
		basic_gridmap<T>::~basic_gridmap()
		{
			if(is_allocate())	deallocate();
		}

		// <---------------------------------------- constructor, destructor


		// ----------------------------------------> allocate, deallocate
		/**
		 * @brief allocate memory
		 * @param[in] r : row size
		 * @param[in] c : column size
		 */
		template< typename T >
		inline
		int basic_gridmap<T>::allocate(const uint32_t r, const uint32_t c)
		{
			if(is_allocate())	return -1;

			if( !(_header = new wpt[1]) )	return -1;
			_plane.row = 1;
			if( !(_header[0] = new pt[1]) )	return -1;
			_plane.column = 1;

			if( !(_header[0][0] = new T[(unsigned long)r*c]) )	return -1;
			_unit.row = r;
			_unit.column = c;

			return 0;
		}

		/**
		 * @brief allocate memory
		 * @param[in] ur : memory unit size (row)
		 * @param[in] uc : memory unit size (column)
		 * @param[in] pr : number of unit (row)
		 * @param[in] pc : number of unit (column)
		 */
		template< typename T >
		inline int basic_gridmap<T>::allocate(const uint32_t ur, const uint32_t uc, const uint32_t pr, const uint32_t pc)
		{
			if(is_allocate())	return -1;

			if( !(_header = new wpt[pr]) )	return -1;
			for( uint32_t r = 0; r < pr; r++  ){
				if( !(_header[r] = new pt[pc]) )	return -1;
				for( uint32_t c = 0; c < pc; c++){
					if( !(_header[r][c] = new T[(unsigned long)ur*uc]) )	return -1;
				}
			}

			_plane.row = pr;
			_plane.column = pc;
			_unit.row = ur;
			_unit.column = uc;

			return 0;
		}

		/**
		 * @brief deallocate memory
		 */
		template< typename T >
		inline int basic_gridmap<T>::deallocate()
		{
			if(!is_allocate())	return -1;

			for( uint32_t pr = 0; pr < _plane.row; pr++){
				for( uint32_t pc = 0; pc < _plane.column; pc++){
					delete [] _header[pr][pc];
				}
				delete [] _header[pr];
			}
			delete[] _header;

			_unit.row = 0;
			_unit.column = 0;
			_plane.row = 0;
			_plane.column = 0;

			return 0;
		}


		/**
		 * @brief reallocate memory
		 * @param[in] sr : required size of grid (row)
		 * @param[in] sc : required size of grid (column)
		 * @param[in] hr : index of current header (row)
		 * @param[in] hc : index of current header (column)
		 */
		template< typename T >
		inline int basic_gridmap<T>::reallocate(const unsigned long sr, const unsigned long sc, const unsigned long hr, const unsigned long hc)
		{
			if( sr < row() && sc < column())	return -1;

			{
				uint32_t	npr = sr > row() ? ceil( (double)sr / _unit.row ) : _plane.row,
						npc = sc > column() ? ceil( (double)sc / _unit.column ) : _plane.column;
				uint32_t ncr =  ceil( (double)hr / _unit.row ),
						//					ncc =  ceil( (double)cc / _unit.row );
						ncc =  ceil( (double)hc / _unit.column );
				tpt header;


				header = new wpt[npr];
				for(uint32_t i = 0; i < npr; i++){
					header[i] = new pt[npc];
					for(uint32_t j = 0; j < npc; j++){
						// copy
						if( i >= ncr && i < ncr + _plane.row &&
								j >= ncc && j < ncc + _plane.column ){
							header[i][j] = _header[i - ncr][j - ncc];
						}
						// new
						else{
							header[i][j] = new T[_unit.row*_unit.column];
						}
					}
				}

				// deallocate
				for( uint32_t pr = 0; pr < _plane.row; pr++){
					delete [] _header[pr];
				}
				delete[] _header;

				// swap
				_header = header;
				_plane.row = npr;
				_plane.column = npc;
			}

			return 0;
		}

		/**
		 * @brief change auto reallocation flag
		 * @param[in] flg
		 */
		template< typename T >
		inline int basic_gridmap<T>::set_autoreallocate(bool flg)
		{
			autoreallocate = flg;
			return 0;
		}


		/**
		 * @brief return memory allocated or not
		 */
		template< typename T >
		inline bool basic_gridmap<T>::is_allocate()
		{
			return _header && _unit.row && _unit.column;
		}
		// <---------------------------------------- allocate, deallocate


		// ----------------------------------------> setter, getter
		/**
		 * @brief pointer of a pixel
		 * @param[in] r : pixel index (row)
		 * @param[in] c : pixel index (column)
		 */
		template< typename T >
		inline T* basic_gridmap<T>::pointer(const unsigned long r, const unsigned long c)
		{
			if(!is_allocate())				return 0;
			if(r > row() || c > column())	return 0;

			{
				uint32_t pr, pc, ur, uc;
				pr = memory_unit_row(r);
				pc = memory_unit_column(c);
				ur = r % _unit.row;
				uc = c % _unit.column;

				return _header[pr][pc] + (_unit.column * ur) + uc;
			}
		}

		/**
		 * @brief pointer of a memory unit
		 * @param[in] r : memory unit index (row)
		 * @param[in] c : memory unit index (column)
		 */
		template< typename T >
		inline T* basic_gridmap<T>::blockheader(const uint32_t r, const uint32_t c)
		{
			if(!is_allocate())	return 0;
			if(r > _plane_row_() || c > _plane_column_())	return 0;

			return _header[r][c];
		}


		/**
		 * @brief value of a pixel
		 * @param[in] r : pixel index (row)
		 * @param[in] c : pixel index (column)
		 */
		template< typename T >
		inline T basic_gridmap<T>::value(const unsigned long r, const unsigned long c)
		{
			return * pointer(r, c);
		}

		/**
		 * @brief number of pixel (row)
		 */
		template< typename T >
		inline unsigned long basic_gridmap<T>::row()
		{
			return _plane.row * _unit.row;
		}

		/**
		 * @brief number of pixel (column)
		 */
		template< typename T >
		inline unsigned long basic_gridmap<T>::column()
		{
			return (unsigned long)_plane.column * _unit.column;
		}

		/**
		 * @brief get value in a pixel
		 * @param[in]  r : pixel's index (row)
		 * @param[in]  c : pixel's index (column)
		 * @param[out] v : pixel's value
		 */
		template< typename T >
		inline int basic_gridmap<T>::get(const unsigned long r, const unsigned long c, pt v)
		{
			if(!v)	return -1;
			::memcpy(v, pointer(r, c), sizeof(T));
			return 0;
		}

		/**
		 * @brief set value in a pixel
		 * @param[in]  r : pixel's index (row)
		 * @param[in]  c : pixel's index (column)
		 * @param[in] v : pixel's value
		 */
		template< typename T >
		inline int basic_gridmap<T>::set(const unsigned long r, const unsigned long c, const pt v)
		{
			if(!v)	return -1;
			::memcpy(pointer(r, c), v, sizeof(T));
			return 0;
		}

		/**
		 * @brief set all pixel value uniformly
		 * @param[in] v : value
		 */
		template< typename T >
		inline int basic_gridmap<T>::set_uniform(const T v)
		{
			return set_uniform(&v);
		}

		/**
		 * @brief set all pixel value uniformly
		 * @param[in] v : value
		 */
		template< typename T >
		inline int basic_gridmap<T>::set_uniform(const T *v)
		{
			if(!v)	return -1;
			if(!is_allocate())	return -1;

			{ // ---> unit
				const uint32_t s = _unit.column * _unit.row;
				uint32_t i, j;

				::memcpy( _header[0][0] + 0, v, sizeof(T)) ;
				j = s >> 1;
				for(i = 1; i <= j; i <<= 1){
					::memcpy( _header[0][0] + i, _header[0][0] + 0, sizeof(T) * i);
				}
				if(i < s)	::memcpy( _header[0][0] + i, _header[0][0] + 0, sizeof(T) * (s-i));
			} // <--- unit

			{ // ---> plane
				const uint32_t cps = sizeof(T) * _unit.row * _unit.column;
				for(uint32_t r = 0; r < _plane.row; r++){
					for(uint32_t c = 0; c < _plane.column; c++){
						if(r != 0 || c != 0) ::memcpy(_header[r][c], _header[0][0], cps);
					}
				}
			} // <--- plane

			return 0;
		}



		/**
		 * @brief memory unit size (row)
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::_unit_row_()
		{
			return _unit.row;
		}

		/**
		 * @brief memory unit size (row)
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::_unit_column_()
		{
			return _unit.column;
		}

		/**
		 * @brief number of  memory unit (row)
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::_plane_row_()
		{
			return _plane.row;
		}

		/**
		 * @brief number of  memory unit (column)
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::_plane_column_()
		{
			return _plane.column;
		}

		/**
		 * @brief row index of memory unit
		 * @param[in] r : pixel row index
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::memory_unit_row(unsigned long r)
		{
			return r / _unit.row;
		}

		/**
		 * @brief column index of memory unit
		 * @param[in] c : pixel column index
		 */
		template< typename T >
		inline uint32_t basic_gridmap<T>::memory_unit_column(unsigned long c)
		{
			return c / _unit.column;
		}

		// <---------------------------------------- setter, getter



		// ---> read write
		/**
		 * @brief file write (binary)
		 * @param[in] fname : file path
		 */
		template< typename T >
		inline int basic_gridmap<T>::fwrite(const char *fname)
		{
			int fd;
			int ret;
			int fsize;

			{ // ---> initialize
				fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT | gnd::wO_BINARY,
						gnd::wS_IRUSR | gnd::wS_IWUSR |
						gnd::wS_IRGRP | gnd::wS_IWGRP );
				if(fd < 0) return -1;
				fsize = 0;
			} // <--- initialize


			{ // ---> operation
				int32_t unit_size = 0;

				// save file tag
				if( (ret = ::write(fd, gridmap::__GridMapFileTag__, gridmap::__FileTagSize__)) != gridmap::__FileTagSize__ )
					return -1;
				fsize += ret;

				// file header
				if( (ret = ::write(fd, &_unit, sizeof(_unit)) ) != sizeof(_unit) )
					return -1;
				fsize += ret;
				if( (ret = ::write(fd, &_plane, sizeof(_plane)) ) != sizeof(_plane) )
					return -1;
				fsize += ret;

				// ---> data
				unit_size = sizeof(T) * _unit.row * _unit.column;
				for(uint32_t r = 0; r < _plane.row; r++){
					for(uint32_t c = 0; c < _plane.column; c++){
						if( (ret = ::write(fd, _header[r][c], unit_size) ) != unit_size )
							return -1;
						fsize += ret;
					}
				} // <--- data
			} // <--- operation


			{ // ---> finalize
				close(fd);
			} // <--- finalize

			return fsize;
		}


		/**
		 * @brief file read (binary)
		 * @param[in] fname : file path
		 */
		template< typename T >
		inline int basic_gridmap<T>::fread(const char *fname)
		{
			int fd;
			int ret;
			long fsize;

			{ // ---> initialize
				fd = open(fname, O_RDWR | gnd::wO_BINARY );
				if(fd < 0) return -1;
				fsize = 0;
			} // <--- initialize


			{ // ---> operation
				unsigned char buf[gridmap::__FileTagSize__];
				gridmap::pixelindex u, p;
				int32_t unit_size = 0;

				// save file tag
				if( (ret = ::read(fd, buf, sizeof(buf))) != (signed)sizeof(buf) )
					return -1;
				fsize += ret;
				if( ::memcmp(buf, gridmap::__GridMapFileTag__, gridmap::__FileTagSize__ ))
					return -1;

				// file header
				if( (ret = ::read(fd, &u, sizeof(u)) ) != sizeof(u) )
					return -1;
				fsize += ret;
				if( (ret = ::read(fd, &p, sizeof(p)) ) != sizeof(p) )
					return -1;
				fsize += ret;

				// allocate
				if( is_allocate() ) deallocate();
				if( allocate(u.row, u.column, p.row, p.column) < 0 ) return -1;

				// ---> data
				unit_size = sizeof(T) * _unit.row * _unit.column;
				for(uint32_t r = 0; r < _plane.row; r++){
					for(uint32_t c = 0; c < _plane.column; c++){
						if( (ret = ::read(fd, _header[r][c], unit_size) ) != unit_size )
							return -1;
						fsize += ret;
					}
				} // <--- data
			} // <--- operation


			{ // ---> finalize
				close(fd);
			} // <--- finalize

			return fsize;
		}
	}
} // <--- namespace gnd
// <---- class definition basic map





// ---> class definition
namespace gnd { // namespace gnd
	namespace gridmap { // ---> namespace GridMap

		/**
		 * @ingroup GNDGridMap
		 * @brief grid-plane class
		 * @tparam T : stored data type in a pixel
		 */
		template < typename T>
		class gridplane
				: public basic_gridmap<T> {
				public:
			/// @brief storing data type pointer
			typedef T* pt;
			/// @brief storing data type const pointer
			typedef const T* const_pt;
			/// @brief storing data type double pointer
			typedef T** wpt;

				public:
			/// @brief component on 2-dimension space
			union component2d{
				struct {
					/// @brief x-component
					double x;
					/// @brief y-component
					double y;
				};
				/// @brief vector description
				double vec[2];
			};
			/// @brief typedef
			typedef union component2d component2d;

			// ---> constructor, destructor
				public:
			// constructor
			gridplane();
			// destructor
			~gridplane();
			// <--- constructor, destructor

			// ---> variables
				protected:
			/// @brief origin(row index 0, column index 0)
			component2d _orgn;
			/// @brief resolution
			component2d _rsl;
			// <--- variables
				public:
			// set core position
			int pset_core(const double x, const double y);
			// set origin position
			int pset_origin(const double x, const double y);
			// get origin position
			int pget_origin(double *x, double *y);
			// set resolution
			int pset_rsl(double x, double y);

			// ---> allocate, deallocate
				public:
			// allocate
			virtual int pallocate(const double xs, const double ys, const double xrsl, const double yrsl);
			// deallocate
			virtual int deallocate();
			// reallocate
			virtual int reallocate(const double xs, const double ys);
			// <--- allocate, deallocate


			// ---> setter, getter
				public:
			// get pixel index
			int pindex(const double x, const double y, long *r, long *c);
			// get pixel pointer
			pt ppointer(const double x, const double y);
			// get pixel value
			T pvalue(const double x, const double y);
			// x lower bounds on current plane
			double xlower();
			// x upper bounds on current plane
			double xupper();
			// y lower bounds on current plane
			double ylower();
			// y upper bounds on current plane
			double yupper();
			// get a pixel value
			int pget(const double x, const double y, pt v);
			// set a pixel value
			int pset(const double x, const double y, const_pt v);
			// get x component resolution
			double xrsl();
			// get y component resolution
			double yrsl();
			// get origin's x component value
			double xorg();
			// get origin's y component value
			double yorg();
			// get plane width
			double width();
			// get plane height
			double height();

			// get a pixel lower boundary position
			int pget_pos_lower(const unsigned long r, const unsigned long c, double *x, double *y);
			// get a pixel upper boundary position
			int pget_pos_upper(const unsigned long r, const unsigned long c, double *x, double *y);
			// get a pixel core position
			int pget_pos_core(const unsigned long r, const unsigned long c, double *x, double *y);
			// <--- setter, getter


			// ---> operator
				public:
			/// operator over-ride
			pt operator[](const int i);
			// <--- operator


			// ---> read write
				public:
			// file write(binary)
			int fwrite(const char *fname);
			// file read(binary)
			int fread(const char *fname);

			// <--- read write

		};




		// ----------------------------------------> constructor, destructor
		/**
		 * @brief constructor
		 */
		template< typename T >
		inline gridplane<T>::gridplane()
		: basic_gridmap<T>()
		  {

		  }

		/**
		 * @brief destructor
		 */
		template< typename T >
		inline gridplane<T>::~gridplane()
		{

		}
		// <---------------------------------------- constructor, destructor


		// ----------------------------------------> allocator, deallocator

		/**
		 * @brief allocate
		 * @param   xs : x component size
		 * @param   ys : y component size
		 * @param xrsl : x component resolution
		 * @param yrsl : y component resolution
		 */
		template< typename T >
		inline int gridplane<T>::pallocate(const double xs, const double ys, const double xrsl, const double yrsl)
		{
			unsigned long r, c;
			int ret;

			if(xrsl <= 0 || yrsl <= 0)	return -1;

			// compute size of row and column
			r = ::ceil(ys / yrsl);
			c = ::ceil(xs / xrsl);
			// allocate
			if( (ret = basic_gridmap<T>::allocate(r, c)) < 0 )	return ret;
			// set resolution
			_rsl.x = xrsl;
			_rsl.y = yrsl;
			return ret;
		}


		/**
		 * @brief deallocate
		 */
		template< typename T >
		inline int gridplane<T>::deallocate()
		{
			int ret;

			if( (ret = basic_gridmap<T>::deallocate()) < 0 ) return ret;

			_orgn.x = 0;
			_orgn.y = 0;
			_rsl.x = 0;
			_rsl.y = 0;

			return ret;
		}


		/**
		 * @brief deallocate
		 * @param[in] xs : requiring x component size
		 * @param[in] ys : requiring y component size
		 */
		template< typename T >
		inline int gridplane<T>::reallocate(const double xs, const double ys)
		{
			long r = 0, c = 0;
			int ret;

			// compute access index of row and column
			if( pindex(xs, ys, &r, &c) == 0 )	return 0;

			{ // ---> operation
				unsigned long cr = r < 0 ? -r : 0;
				unsigned long cc = c < 0 ? -c : 0;
				unsigned long rs = basic_gridmap<T>::row();
				unsigned long cs = basic_gridmap<T>::column();

				// compute required size of row and column
				if( r < 0 )											rs = cr + basic_gridmap<T>::row();
				else if( r >= (signed)basic_gridmap<T>::row())		rs = r + 1;
				if( c < 0 )											cs = cc + basic_gridmap<T>::column();
				else if( c >= (signed)basic_gridmap<T>::column())	cs = c + 1;

				// reallocate
				if( (ret = basic_gridmap<T>::reallocate(rs, cs, cr, cc)) < 0 ) return ret;
				// adjust origin
				if( cc != 0 ){
					_orgn.x -= _rsl.x * ::ceil( (double)cc / basic_gridmap<T>::_unit.column ) * basic_gridmap<T>::_unit.column;
				}
				if( cr != 0 ){
					_orgn.y -= _rsl.y * ::ceil( (double)cr / basic_gridmap<T>::_unit.row ) * basic_gridmap<T>::_unit.row;
				}
			} // <--- operation

			return ret;
		}
		// <---------------------------------------- allocator, deallocator




		// ----------------------------------------> setter, getter
		/**
		 * @brief set core position
		 * @param[in] x : x component value
		 * @param[in] y : y component value
		 */
		template< typename T >
		inline int gridplane<T>::pset_core(const double x, const double y)
		{
			_orgn.x = x - (basic_gridmap<T>::column() * _rsl.x) / 2.0;
			_orgn.y = y - (basic_gridmap<T>::row() * _rsl.y) / 2.0;
			return 0;
		}


		/**
		 * @brief set origin position
		 * @param[in] x : x component value
		 * @param[in] y : y component value
		 */
		template< typename T >
		inline int gridplane<T>::pset_origin(const double x, const double y)
		{
			_orgn.x = x;
			_orgn.y = y;
			return 0;
		}


		/**
		 * @brief get core position
		 * @param[out] x : x component value
		 * @param[out] y : y component value
		 */
		template< typename T >
		inline int gridplane<T>::pget_origin(double *x, double *y)
		{
			*x = _orgn.x;
			*y = _orgn.y;
			return 0;
		}


		/**
		 * @brief get pixel index
		 * @param[in]  x : x component value
		 * @param[in]  y : y component value
		 * @param[out] r : row index
		 * @param[out] c : column index
		 */
		template< typename T >
		inline int gridplane<T>::pindex(const double x, const double y, long *r, long *c)
		{
			if( !basic_gridmap<T>::is_allocate() ) return -1;

			{
				double xx = x - _orgn.x;
				double yy = y - _orgn.y;
				*r = ::floor( yy / _rsl.y );
				*c = ::floor( xx / _rsl.x );
			}

			return (*r >= 0 && *r < (signed)basic_gridmap<T>::row() &&
					*c >= 0 && *c < (signed)basic_gridmap<T>::column() ) ? 0 : -1;
		}


		/**
		 * @brief get pointer
		 * @param[in]  x : x component value
		 * @param[in]  y : y component value
		 * @return 0 : not exit
		 */
		template< typename T >
		inline T* gridplane<T>::ppointer(const double x, const double y)
		{
			long r, c;

			if( pindex(x, y, &r, &c) < 0 ) return 0;

			return basic_gridmap<T>::pointer(r, c);
		}

		/**
		 * @brief get value
		 * @param[in]  x : x component value
		 * @param[in]  y : y component value
		 * @return pixel's value
		 */
		template< typename T >
		inline T gridplane<T>::pvalue(const double x, const double y)
		{
			return * ppointer(x, y);
		}

		/**
		 * @brief x lower bound
		 */
		template< typename T >
		inline double gridplane<T>::xlower()
		{
			return _orgn.x;
		}

		/**
		 * @brief x upper bound
		 */
		template< typename T >
		inline double gridplane<T>::xupper()
		{
			return _orgn.x + basic_gridmap<T>::column() * _rsl.x;
		}

		/**
		 * @brief y lower bound
		 */
		template< typename T >
		inline double gridplane<T>::ylower()
		{
			return _orgn.y;
		}

		/**
		 * @brief y upper bound
		 */
		template< typename T >
		inline double gridplane<T>::yupper()
		{
			return _orgn.y + basic_gridmap<T>::row() * _rsl.y;
		}

		/**
		 * @brief get value
		 * @param[in]  x : x component value
		 * @param[in]  y : y component value
		 * @param[out] v : pixel's value
		 */
		template< typename T >
		inline int gridplane<T>::pget(const double x, const double y, pt v)
		{
			long r, c;

			if(!v)	return -1;
			if( pindex(x, y, &r, &c) < 0 ) return -1;
			::memcpy(v, basic_gridmap<T>::pointer(r, c), sizeof(T));
			return 0;
		}

		/**
		 * @brief set value
		 * @param[in] x : x component value
		 * @param[in] y : y component value
		 * @param[in] v : pixel's value
		 */
		template< typename T >
		inline int gridplane<T>::pset(const double x, const double y, const_pt v)
		{
			long r = 0, c = 0;
			if(!v)	return -1;
			for( int ret = pindex(x, y, &r, &c); ret < 0; ret = pindex(x, y, &r, &c)){
				int ret_realloc;
				if( (ret_realloc = reallocate(x,y)) < 0) return -1;
			}
			::memcpy(basic_gridmap<T>::pointer(r, c), v, sizeof(T));
			return 0;
		}

		/**
		 * @brief x resolution
		 */
		template< typename T >
		inline double gridplane<T>::xrsl()
		{
			return _rsl.x;
		}

		/**
		 * @brief y resolution
		 */
		template< typename T >
		inline double gridplane<T>::yrsl()
		{
			return _rsl.y;
		}


		/**
		 * @brief origin's x component vlaue
		 */
		template< typename T >
		inline double gridplane<T>::xorg()
		{
			return _orgn.x;
		}


		/**
		 * @brief origin's y component vlaue
		 */
		template< typename T >
		inline double gridplane<T>::yorg()
		{
			return _orgn.y;
		}


		/**
		 * @brief plane widith
		 */
		template< typename T >
		inline double gridplane<T>::width() {
			return _rsl.x * basic_gridmap<T>::column();
		}


		/**
		 * @brief plane height
		 */
		template< typename T >
		inline double gridplane<T>::height() {
			return _rsl.y * basic_gridmap<T>::row();
		}


		/**
		 * @brief set resolution
		 */
		template< typename T >
		inline int gridplane<T>::pset_rsl(double x, double y)
		{
			_rsl.x = x;
			_rsl.y = y;
			return 0;
		}


		/**
		 * @brief get a pixel lower bounds
		 */
		template< typename T >
		inline int gridplane<T>::pget_pos_lower(const unsigned long r, const unsigned long c, double *x, double *y)
		{
			*x = _orgn.x + c * _rsl.x;
			*y = _orgn.y + r * _rsl.y;
			return 0;
		}


		/**
		 * @brief get a pixel upper bounds
		 */
		template< typename T >
		inline int gridplane<T>::pget_pos_upper(const unsigned long r, const unsigned long c, double *x, double *y)
		{
			pget_pos_lower(r, c, x, y);
			*x += _rsl.x;
			*y += _rsl.y;
			return 0;
		}


		/**
		 * @brief get a pixel core position
		 */
		template< typename T >
		inline int gridplane<T>::pget_pos_core(const unsigned long r, const unsigned long c, double *x, double *y)
		{
			pget_pos_lower(r, c, x, y);
			*x += _rsl.x / 2.0;
			*y += _rsl.y / 2.0;
			return 0;
		}

		// <---------------------------------------- setter, getter





		// ---> read write
		/**
		 * @brief file write
		 * @param[in] fname : file name
		 */
		template< typename T >
		inline int gridplane<T>::fwrite(const char *fname)
		{
			int fd;
			int ret;
			int fsize;

			LogVerbosef("Begin - gridplane write into \"%s\"\n", fname);
			LogIndent();

			{ // ---> initialize
				fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT | gnd::wO_BINARY,
						gnd::wS_IRUSR | gnd::wS_IWUSR |
						gnd::wS_IRGRP | gnd::wS_IWGRP);
				if(fd < 0) {
					LogVerbose("fail to file open\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				fsize = 0;
			} // <--- initialize


			{ // ---> operation
				uint32_t unit_size = 0;
				gridmap::pixelindex buf;

				// save file tag
				if( (ret = ::write(fd, gridmap::__GridPlaneFileTag__, gridmap::__FileTagSize__)) != gridmap::__FileTagSize__ ) {
					LogVerbose("fail to write\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("File Tag %dbyte\n", ret);
				fsize += ret;

				// file header
				buf = basic_gridmap<T>::_unit;
				if( (ret = ::write(fd, &buf, sizeof(buf) ) ) != sizeof( buf ) ) {
					LogVerbose("fail to write\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Memory Unit Size Data %dbyte\n", ret);
				fsize += ret;

				buf = basic_gridmap<T>::_plane;
				if( (ret = ::write(fd, &buf, sizeof(buf) ) ) != sizeof( buf ) ){
					LogVerbose("fail to write\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Memory Unit Num Data %dbyte\n", ret);
				fsize += ret;

				// file header
				if( (ret = ::write(fd, &_orgn, sizeof(_orgn)) ) != sizeof(_orgn) ){
					LogVerbose("fail to write\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Origin Data %dbyte\n", ret);
				fsize += ret;
				if( (ret = ::write(fd, &_rsl, sizeof(_rsl)) ) != sizeof(_orgn) ){
					LogVerbose("fail to write\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Pixel Size Data %dbyte\n", ret);
				fsize += ret;

				// ---> data
				unit_size = sizeof(T) * basic_gridmap<T>::_unit.row * basic_gridmap<T>::_unit.column;
				for(uint32_t r = 0; r < basic_gridmap<T>::_plane.row; r++){
					for(uint32_t c = 0; c < basic_gridmap<T>::_plane.column; c++){
						uint32_t nwrite = 0;
						char *p = (char*)basic_gridmap<T>::_header[r][c];

						while( nwrite < unit_size ) {
							ret = ::write(fd, p + nwrite, unit_size - nwrite);
							if( ret <= 0 ){
								LogVerbose("fail to write\n");
								LogUnindent();
								LogVerbosef("Fail - gridplane write into \"%s\"\n", fname);
								return -1;
							}
							nwrite += ret;
						}
						LogVerbosef("Memory Unit(%d x %d) %dbyte\n", r, c, nwrite);
						fsize += nwrite;
					}
				} // <--- data
			} // <--- operation


			{ // ---> finalize
				close(fd);
			} // <--- finalize

			LogUnindent();
			LogVerbosef("End - gridplane write into \"%s\"\n", fname);
			return fsize;
		}


		/**
		 * @brief file read
		 * @param[in] fname : file name
		 */
		template< typename T >
		inline int gridplane<T>::fread(const char *fname)
		{
			int fd;
			int ret;
			int fsize;

			LogVerbosef("Begin - gridplane read from \"%s\"\n", fname);
			LogIndent();

			{ // ---> initialize
				fd = open(fname, O_RDWR | gnd::wO_BINARY );
				if(fd < 0) {
					LogVerbose("fail to file open\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				fsize = 0;
			} // <--- initialize

			{ // ---> operation
				unsigned char buf[gridmap::__FileTagSize__+1];
				gridmap::pixelindex u, p;
				int32_t unit_size = 0;
				memset(buf, 0, sizeof(buf));
				// save file tag
				if( (ret = ::read(fd, buf, gridmap::__FileTagSize__)) != (signed) gridmap::__FileTagSize__ ) {
					LogVerbose("fail to read\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("File Tag %dbyte\n", ret);
				fsize += ret;
				if( ::memcmp(buf, gridmap::__GridPlaneFileTag__, gridmap::__FileTagSize__ )) {
					LogVerbose("Invalid Tag\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}

				// file header
				if( (ret = ::read(fd, &u, sizeof(u)) ) != sizeof(u) ) {
					LogVerbose("fail to read\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Memory Unit Size Data %dbyte\n", ret);
				fsize += ret;
				if( (ret = ::read(fd, &p, sizeof(p)) ) != sizeof(p) ) {
					LogVerbose("fail to read\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Memory Unit Num Data %dbyte\n", ret);
				fsize += ret;

				// file header
				if( (ret = ::read(fd, &_orgn, sizeof(_orgn)) ) != sizeof(_orgn) ) {
					LogVerbose("fail to read\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Origin Data %dbyte\n", ret);
				fsize += ret;
				if( (ret = ::read(fd, &_rsl, sizeof(_rsl)) ) != sizeof(_orgn) ) {
					LogVerbose("fail to read\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}
				LogVerbosef("Pixel Size Data %dbyte\n", ret);
				fsize += ret;

				// allocate
				if( basic_gridmap<T>::is_allocate() ) basic_gridmap<T>::deallocate();
				if( basic_gridmap<T>::allocate(u.row, u.column, p.row, p.column) < 0 ) {
					LogVerbose("fail to allocate\n");
					LogUnindent();
					LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
					return -1;
				}

				// ---> data
				unit_size = sizeof(T) * basic_gridmap<T>::_unit.row * basic_gridmap<T>::_unit.column;
				for(uint32_t r = 0; r < basic_gridmap<T>::_plane.row; r++){
					for(uint32_t c = 0; c < basic_gridmap<T>::_plane.column; c++){
						int32_t nread = 0;
						char* p = (char*) basic_gridmap<T>::_header[r][c];

						while( nread < unit_size ) {
							ret = ::read(fd, p + nread, unit_size - nread);
							if( ret <= 0 ) {
								LogVerbose("fail to read\n");
								LogUnindent();
								LogVerbosef("Fail - gridplane read from \"%s\"\n", fname);
								return -1;
							};
							nread += ret;
						}
						LogVerbosef("Memory Unit(%d x %d) %dbyte\n", r, c, nread);
						fsize += nread;
					}
				} // <--- data

			} // <--- operation


			{ // ---> finalize
				close(fd);
			} // <--- finalize

			LogUnindent();
			LogVerbosef("End - gridplane read from \"%s\"\n", fname);
			return fsize;
		}
	}
} // <--- namespace gnd
// <--- class definition

#include "gnd-debug-log-util-undef.h"

#endif /* GND_GRIDMAP_HPP_ */
