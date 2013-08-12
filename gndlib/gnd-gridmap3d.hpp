/*
 * gnd-gridmap3d.hpp
 *
 *  Created on: 2012/09/22
 *      Author: tyamada
 */

#ifndef GND_GRIDMAP3D_HPP_
#define GND_GRIDMAP3D_HPP_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * @ifnot GNDStorage
 * @defgroup GNDStorage storage
 * @endif
 */

/**
 * @defgroup GNDGridMap3D grid-map-3d
 * @ingroup GNDStorage
 */

// ---> constant value definition
namespace gnd { // ---> namespace gnd
	namespace GridMap3D { // ---> namespace GridMap
		/** @var __GridMap3DFileTag__
		 * @ingroup GNDGridMap3D
		 * @brief file tag
		 */
		static const char __GridMap3DFileTag__[] = "GG3D";
		/** @var __GridSolidFileTag__
		 * @ingroup GNDGridMap3D
		 * @brief file tag
		 */
		static const char __GridSolidFileTag__[] = "GSLD";
		/** @var __FileTagSize__
		 * @ingroup GNDGridMap3D
		 * @brief file tag byte size
		 */
		static const ssize_t __FileTagSize__ = ::strlen(__GridMap3DFileTag__);
	} // <--- namespace GridMap
} // <--- namespace gnd
// <--- constant value definition



// ---> type definition
namespace gnd { // ---> namespace gnd
	namespace GridMap3D { // ---> namespace GridMap
		/**
		 * @ingroup GNDGridMap3D
		 * @brief index of voxel
		 */
		typedef struct voxelindex {
			/// @brief x index
			size_t x;
			/// @brief y index
			size_t y;
			/// @brief z index
			size_t z;
		} voxelindex;
	}
	template < typename T>
	class basic_gridmap3d;
	template < typename T>
	class gridsolid;
}
// <--- type definition



// ---> class definition
namespace gnd { // ---> namespace gnd
	/**
	 * @ingroup GNDGridMap3D
	 * @brief grid-map basic class
	 * @tparam T : stored data type in a pixel
	 */
	template < typename T>
	class basic_gridmap3d {
	public:
		/// @brief pointer type
		typedef T*		pt;
		/// @brief double pointer type
		typedef T**		wpt;
		/// @brief triple pointer type
		typedef T***	tpt;
		/// @brief quad pointer type
		typedef T****	qpt;

		typedef T compoment;

	private:
		unsigned long _blockindex(const size_t x, const size_t y, const size_t z);
		unsigned long _voxelindex(const size_t x, const size_t y, const size_t z);

		// ---> constructor, destructor
	public:
		basic_gridmap3d();
		~basic_gridmap3d();
		// <--- constructor, destructor

		// ---> variables
	protected:
		/// @brief data storage header pointer
		qpt _header;
		/// @brief number of memory unit
		GridMap3D::voxelindex _blocksize;
		/// @brief number of plane
		GridMap3D::voxelindex _mapsize;
		/// @brief flag of auto allocation
		bool autoreallocate;
		// <--- variables

		// ---> allocate, deallocate
	public:
		virtual int allocate(const size_t x, const size_t y, const size_t z);
		virtual int allocate(const size_t bx, const size_t by, const size_t bz, const size_t px, const size_t py, const size_t pz);
		virtual int deallocate();
		virtual bool is_allocate();
		virtual int reallocate(const unsigned long sx, const unsigned long sy, const unsigned long sz,
				const size_t hx, const size_t hy, const size_t hz);
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
		pt pointer(const unsigned long x, const unsigned long y, const unsigned long z);
		/**
		 * @brief get memory block header
		 * @param[in] r: plane row index
		 * @param[in] c: plane column index
		 */
		pt blockheader(const size_t x, const size_t y, const size_t z);
		/**
		 * @brief get pixel value
		 * @param[in] r: pixel row index
		 * @param[in] c: pixel column index
		 */
		T value(const unsigned long x, const unsigned long y, const unsigned long z);
		unsigned long xsize();
		unsigned long ysize();
		unsigned long zsize();
		int get(const unsigned long x, const unsigned long y, const unsigned long z, pt v);
		int set(const unsigned long x, const unsigned long y, const unsigned long z, const pt v);

		int set_uniform(const T v);
		int set_uniform(const T* v);


	private:
		size_t blockaddrx(unsigned long x);
		size_t blockaddry(unsigned long y);
		size_t blockaddrz(unsigned long y);
		// <--- setter, getter


		// ---> read write
	public:
		virtual int fwrite(const char *fname);
		virtual int fread(const char *fname);

		// <--- read write

	};






	template< typename T >
	inline
	unsigned long basic_gridmap3d<T>::_voxelindex(const size_t x, const size_t y, const size_t z)
	{
		return x + y * _blocksize.x + z * _blocksize.x * _blocksize.y;
	}


	// ----------------------------------------> constructor, destructor
	/**
	 * @brief constructor
	 */
	template< typename T >
	inline
	basic_gridmap3d<T>::basic_gridmap3d() : _header(0)
	{
		_blocksize.x = 0;
		_blocksize.y = 0;
		_blocksize.z = 0;

		_mapsize.x = 0;
		_mapsize.y = 0;
		_mapsize.z = 0;
	}

	/**
	 * @brief destructor
	 */
	template< typename T >
	inline
	basic_gridmap3d<T>::~basic_gridmap3d()
	{
		if(is_allocate())	deallocate();
	}

	// <---------------------------------------- constructor, destructor


	// ----------------------------------------> allocate, deallocate
	/**
	 * @brief allocate memory
	 * @param[in] x : x size
	 * @param[in] y : y size
	 * @param[in] z : z size
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::allocate(const size_t x, const size_t y, const size_t z)
	{
		if(is_allocate())	return -1;

		if( !(_header = new tpt[1]) )	return -1;
		if( !(_header[0] = new wpt[1]) )	return -1;
		if( !(_header[0][0] = new pt[1]) )	return -1;
		_mapsize.x = _mapsize.y = _mapsize.z = 1;

		if( !(_header[0][0][0] = new T[(unsigned long)x*y*z]) )	return -1;
		_blocksize.x = x;
		_blocksize.y = y;
		_blocksize.z = z;

		return 0;
	}

	/**
	 * @brief allocate memory
	 * @param[in] bx : memory block size (x)
	 * @param[in] by : memory block size (y)
	 * @param[in] bz : memory block size (z)
	 * @param[in] mx : number of block (x)
	 * @param[in] my : number of block (y)
	 * @param[in] mz : number of block (z)
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::allocate(const size_t bx, const size_t by, const size_t bz, const size_t mx, const size_t my, const size_t mz)
	{
		if(is_allocate())	return -1;
		if( !mx || !my || !mz )	return -1;
		if( !bx || !by || !bz )	return -1;

		if( !(_header       = new tpt[mz]) )	return -1;
		for( size_t z = 0; z < mz; z++  ){
			if( !(_header[z]    = new wpt[my]) )	return -1;
			for( size_t y = 0; y < my; y++){
				if( !(_header[z][y] = new  pt[mx]) )	return -1;
				for( size_t x = 0; x < mx; x++){
					if( !(_header[z][y][x] = new T[(unsigned long)bx*by*bz]) )	return -1;
				}
			}
		}

		_mapsize.x = mx;
		_mapsize.y = my;
		_mapsize.z = mz;
		_blocksize.x = bx;
		_blocksize.y = by;
		_blocksize.z = bz;

		return 0;
	}

	/**
	 * @brief deallocate memory
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::deallocate()
	{
		if(!is_allocate())	return -1;

		for( size_t z = 0; z < _mapsize.z; z++){
			for( size_t y = 0; y < _mapsize.y; y++){
				for( size_t x = 0; x < _mapsize.x; x++){
					delete [] blockheader(x,y,z);
				}
				delete [] _header[z][y];
			}
			delete [] _header[z];
		}
		delete[] _header;

		_blocksize.x = 0;
		_blocksize.y = 0;
		_blocksize.z = 0;
		_mapsize.x = 0;
		_mapsize.y = 0;
		_mapsize.z = 0;

		return 0;
	}


	/**
	 * @brief reallocate memory
	 * @param[in] sx : required size of grid (x)
	 * @param[in] sy : required size of grid (y)
	 * @param[in] sz : required size of grid (z)
	 * @param[in] hx : index of current header (x)
	 * @param[in] hy : index of current header (y)
	 * @param[in] hz : index of current header (z)
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::reallocate(const unsigned long sx, const unsigned long sy, const unsigned long sz, const size_t hx, const size_t hy, const size_t hz)
	{
		if( sx < xsize() && sy < ysize() && sz < zsize())	return -1;

		{ // ---> operation
			size_t xi, yi, zi;
			size_t mx = sx > xsize() ? ceil( (double)sx / _blocksize.x ) : _mapsize.x;
			size_t my = sy > ysize() ? ceil( (double)sy / _blocksize.y ) : _mapsize.y;
			size_t mz = sz > zsize() ? ceil( (double)sz / _blocksize.z ) : _mapsize.z;

			qpt h;

			if( !(h = new tpt[mz]) ) return -1;
			for( zi = 0; zi < mz; zi++){
				if( !(h[zi] = new wpt[my]) ) return -1;
				for( yi = 0; yi < my; yi++){
					if( !(h[zi][yi] = new pt[mx]) ) return -1;
					for( xi = 0; xi < mx; xi++){
						// copy
						if( xi >= hx && xi < hx + _mapsize.x &&
								yi >= hy && yi < hy + _mapsize.y &&
								zi >= hz && zi < hz + _mapsize.z ){
							h[zi][yi][xi] = _header[zi - hz][yi - hy][xi - hx];
						}
						// new
						else{
							if( !(h[zi][yi][xi] = new T[ _blocksize.x * _blocksize.y * _blocksize.z ]) ) return -1;
						}
					} // for(x)
				} // for(y)
			} // for(z)
			// deallocate

			// deallocate
			for( size_t zi = 0; zi < _mapsize.z; zi++){
				for( size_t yi = 0; yi < _mapsize.y; yi++){
					delete [] _header[zi][yi];
				}
				delete [] _header[zi];
			}
			delete[] _header;

			// swap
			_header = h;
			_mapsize.x = mx;
			_mapsize.y = my;
			_mapsize.z = mz;
		} // <--- operation

		return 0;
	}

	/**
	 * @brief change auto reallocation flag
	 * @param[in] flg
	 */
	template< typename T >
	inline int basic_gridmap3d<T>::set_autoreallocate(bool flg)
	{
		autoreallocate = flg;
		return 0;
	}


	/**
	 * @brief return memory allocated or not
	 */
	template< typename T >
	inline
	bool basic_gridmap3d<T>::is_allocate()
	{
		return _header && _blocksize.x && _blocksize.y && _blocksize.z;
	}
	// <---------------------------------------- allocate, deallocate


	// ----------------------------------------> setter, getter
	/**
	 * @brief pointer of a pixel
	 * @param[in] r : pixel index (row)
	 * @param[in] c : pixel index (column)
	 */
	template< typename T >
	inline
	T* basic_gridmap3d<T>::pointer(const unsigned long x, const unsigned long y, const unsigned long z)
	{
		if(!is_allocate())				return 0;
		if(x > xsize() || y > ysize() || z > zsize())	return 0;

		{
			GridMap3D::voxelindex bi, vi;
			bi.x = blockaddrx(x);
			bi.y = blockaddry(y);
			bi.z = blockaddrz(z);
			vi.x = x % _blocksize.x;
			vi.y = y % _blocksize.y;
			vi.z = z % _blocksize.z;
			return blockheader(bi.x, bi.y, bi.z) + _voxelindex( vi.x, vi.y, vi.z );
		}
	}

	/**
	 * @brief pointer of a memory unit
	 * @param[in] x : memory block index (x)
	 * @param[in] y : memory block index (y)
	 * @param[in] z : memory block index (z)
	 */
	template< typename T >
	inline
	T* basic_gridmap3d<T>::blockheader(const size_t x, const size_t y, const size_t z)
	{
		if(!is_allocate())	return 0;
		if(x >= _mapsize.x || y >= _mapsize.y || z >= _mapsize.z )	return 0;

		return _header[z][y][x];

	}


	/**
	 * @brief value of a pixel
	 * @param[in] r : pixel index (row)
	 * @param[in] c : pixel index (column)
	 */
	template< typename T >
	inline
	T basic_gridmap3d<T>::value(const unsigned long x, const unsigned long y, const unsigned long z)
	{
		return * pointer(x, y, z);
	}

	/**
	 * @brief number of voxel (x)
	 */
	template< typename T >
	inline
	unsigned long basic_gridmap3d<T>::xsize()
	{
		return _blocksize.x * _mapsize.x;
	}

	/**
	 * @brief number of voxel (y)
	 */
	template< typename T >
	inline
	unsigned long basic_gridmap3d<T>::ysize()
	{
		return _blocksize.y * _mapsize.y;
	}

	/**
	 * @brief number of voxel (z)
	 */
	template< typename T >
	inline
	unsigned long basic_gridmap3d<T>::zsize()
	{
		return _blocksize.z * _mapsize.z;
	}

	/**
	 * @brief get value in a pixel
	 * @param[in]  x : pixel's index (row)
	 * @param[in]  y : pixel's index (column)
	 * @param[in]  y : pixel's index (column)
	 * @param[out] v : pixel's value
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::get(const unsigned long x, const unsigned long y, const unsigned long z, pt v)
	{
		if(!v)	return -1;
		::memcpy(v, pointer(x, y, z), sizeof(T));
		return 0;
	}

	/**
	 * @brief set value in a pixel
	 * @param[in]  r : pixel's index (row)
	 * @param[in]  c : pixel's index (column)
	 * @param[in] v : pixel's value
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::set(const unsigned long x, const unsigned long y, const unsigned long z, pt v)
	{
		if(!v)	return -1;
		::memcpy(pointer(x, y, z), v, sizeof(T));
		return 0;
	}

	/**
	 * @brief set all pixel value uniformly
	 * @param[in] v : value
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::set_uniform(const T v)
	{
		return set_uniform(&v);
	}

	/**
	 * @brief set all pixel value uniformly
	 * @param[in] v : value
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::set_uniform(const T *v)
	{
		if(!v)	return -1;
		if(!is_allocate())	return -1;

		{ // ---> one block
			const size_t s = _blocksize.x * _blocksize.y * _blocksize.z;
			size_t i, j;

			::memcpy( blockheader(0, 0, 0) + 0, v, sizeof(T)) ;
			j = s >> 1;
			for(i = 1; i <= j; i <<= 1){
				::memcpy( blockheader(0, 0, 0) + i, blockheader(0, 0, 0) + 0, sizeof(T) * i);
			}
			if(i < s)	::memcpy( blockheader(0, 0, 0) + i, blockheader(0, 0, 0) + 0, sizeof(T) * (s-i));
		} // <--- one block

		{ // ---> map
			const size_t s = sizeof(T) * _blocksize.x * _blocksize.y * _blocksize.x;
			for(size_t z = 0; z < _mapsize.z; z++){
				for(size_t y = 0; y < _mapsize.y; y++){
					for(size_t x = 0; x < _mapsize.x; x++){
						if(x != 0 || y != 0 || z != 0) ::memcpy(blockheader(x,y,z), blockheader(0,0,0), s);
					}
				}
			}
		} // <--- map

		return 0;
	}





	/**
	 * @brief block address (x)
	 * @param[in] r : voxel x index
	 */
	template< typename T >
	inline
	size_t basic_gridmap3d<T>::blockaddrx(unsigned long i)
	{
		return i / _blocksize.x;
	}

	/**
	 * @brief block address (y)
	 * @param[in] y : voxel y index
	 */
	template< typename T >
	inline
	size_t basic_gridmap3d<T>::blockaddry(unsigned long i)
	{
		return i / _blocksize.y;
	}

	/**
	 * @brief block address (z)
	 * @param[in] r : voxel z index
	 */
	template< typename T >
	inline
	size_t basic_gridmap3d<T>::blockaddrz(unsigned long i)
	{
		return i / _blocksize.z;
	}

	// <---------------------------------------- setter, getter



	// ---> read write
	/**
	 * @brief file write (binary)
	 * @param[in] fname : file path
	 */
	template< typename T >
	inline
	int basic_gridmap3d<T>::fwrite(const char *fname)
	{
		int fd;
		int ret;
		int fsize;

		{ // ---> initialize
			fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT,
					S_IRUSR | S_IWUSR |
					S_IRGRP | S_IWGRP);
			if(fd < 0) return -1;
			fsize = 0;
		} // <--- initialize


		{ // ---> operation
			ssize_t bs = 0;	// block size

			// save file tag
			if( (ret = ::write(fd, GridMap3D::__GridMap3DFileTag__, GridMap3D::__FileTagSize__)) != GridMap3D::__FileTagSize__ )
				return -1;
			fsize += ret;

			// file header
			if( (ret = ::write(fd, &_blocksize, sizeof(_blocksize)) ) != sizeof(_blocksize) )
				return -1;
			fsize += ret;
			if( (ret = ::write(fd, &_mapsize, sizeof(_mapsize)) ) != sizeof(_mapsize) )
				return -1;
			fsize += ret;

			// ---> data
			bs = sizeof(T) * _blocksize.x * _blocksize.y * _blocksize.z;
			for(size_t z = 0; z < _mapsize.z; z++){
				for(size_t y = 0; y < _mapsize.y; y++){
					for(size_t x = 0; x < _mapsize.x; x++){
						if( (ret = ::write(fd, blockheader(x,y,z), bs) ) != bs )
							return -1;
						fsize += ret;
					}
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
	inline
	int basic_gridmap3d<T>::fread(const char *fname)
	{
		int fd;
		int ret;
		long fsize;

		{ // ---> initialize
			fd = open(fname, O_RDWR);
			if(fd < 0) return -1;
			fsize = 0;
		} // <--- initialize


		{ // ---> operation
			unsigned char buf[GridMap3D::__FileTagSize__];
			GridMap3D::voxelindex bs, ms;
			ssize_t bms = 0;

			// save file tag
			if( (ret = ::read(fd, buf, sizeof(buf))) != (signed)sizeof(buf) )
				return -1;
			fsize += ret;
			// check file tag
			if( ::memcmp(buf, GridMap3D::__GridMap3DFileTag__, GridMap3D::__FileTagSize__ ))
				return -1;

			// file header
			if( (ret = ::read(fd, &bs, sizeof(bs)) ) != sizeof(bs) )
				return -1;
			fsize += ret;
			if( (ret = ::read(fd, &ms, sizeof(ms)) ) != sizeof(ms) )
				return -1;
			fsize += ret;

			// allocate
			if( is_allocate() ) deallocate();
			if( allocate(bs.x, bs.y, bs.z, ms.x, ms.y, ms.z) < 0 ) return -1;

			// ---> data
			bms = sizeof(T) * _blocksize.x * _blocksize.y * _blocksize.z;
			for(size_t z = 0; z < _blocksize.z; z++){
				for(size_t y = 0; y < _blocksize.y; y++){
					for(size_t x = 0; x < _blocksize.x; x++){
						if( (ret = ::read(fd, blockheader(x,y,z), bms) ) != bms )
							return -1;
						fsize += ret;
					}
				}
			} // <--- data
		} // <--- operation


		{ // ---> finalize
			close(fd);
		} // <--- finalize

		return fsize;
	}
} // <--- namespace gnd
// <---- class definition basic map





// ---> class definition
namespace gnd { // namespace gnd

	/**
	 * @ingroup GNDGridMap
	 * @brief grid-plane class
	 * @param T : stored data type in a pixel
	 */
	template < typename T>
	class gridsolid
	: public basic_gridmap3d<T> {
	public:
		/// @brief storing data type pointer
		typedef T* pt;
		/// @brief storing data type const pointer
		typedef const T* const_pt;
		/// @brief storing data type double pointer
		typedef T** wpt;
		/// @brief storing data type triple pointer
		typedef T*** tpt;

	public:
		/// @brief component on 2-dimension space
		union component3d{
			struct {
				/// @brief x-component
				double x;
				/// @brief y-component
				double y;
				/// @brief z-component
				double z;
			};
			/// @brief vector description
			double vec[3];

			component3d() : x(0.0), y(0.0), z(0.0) {}
		};
		/// @brief typedef
		typedef union component3d component3d;

		// ---> constructor, destructor
	public:
		// constructor
		gridsolid();
		// destructor
		~gridsolid();
		// <--- constructor, destructor

		// ---> variables
	protected:
		/// @brief origin(row index 0, column index 0)
		component3d _orgn;
		/// @brief resolution
		component3d _rsl;
		// <--- variables
	public:
		// set core position
		int sset_core(const double x, const double y, const double z);
		// set origin position
		int sset_origin(const double x, const double y, const double z);
		// get origin position
		int sget_origin(double *x, double *y, double *z);
		// set resolution
		int sset_rsl(const double x, const double y, const double z);

		// ---> allocate, deallocate
	public:
		// allocate
		virtual int sallocate(const double sx, const double sy, const double sz, const double rslx, const double rsly, const double rslz);
		// allocate
		virtual int sallocate(const int nx, const int ny, const int nz, const double rslx, const double rsly, const double rslz);
		// deallocate
		virtual int deallocate();
		// reallocate
		virtual int reallocate(const double sx, const double sy, const double sz);
		// <--- allocate, deallocate


		// ---> setter, getter
	public:
		// get pixel index
		int sindex(const double x, const double y, const double z, long *xi, long *yi, long *zi);
		// get pixel pointer
		pt spointer(const double x, const double y, const double z);
		// get pixel value
		T svalue(const double x, const double y, const double z);
		// x lower bounds on current plane
		double xlower();
		// x upper bounds on current plane
		double xupper();
		// y lower bounds on current plane
		double ylower();
		// y upper bounds on current plane
		double yupper();
		// z lower bounds on current plane
		double zlower();
		// z upper bounds on current plane
		double zupper();
		// get a pixel value
		int sget(const double x, const double y, const double z, pt v);
		// set a pixel value
		int sset(const double x, const double y, const double z, const_pt v);
		// get x component resolution
		double xrsl();
		// get y component resolution
		double yrsl();
		// get z component resolution
		double zrsl();
		// get origin's x component value
		double xorg();
		// get origin's y component value
		double yorg();
		// get origin's z component value
		double zorg();

		// get a pixel lower boundary position
		int sget_pos_lower(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z);
		// get a pixel upper boundary position
		int sget_pos_upper(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z);
		// get a pixel core position
		int sget_pos_core(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z);
		// <--- setter, getter


		// ---> read write
	public:
		// file write(binary)
		int fwrite(const char *fname);
		// file read(binary)
		int fread(const char *fname);
		// <--- read write

		// ---> for debug
	public:
		int show_param( FILE* fp );

	};




	// ----------------------------------------> constructor, destructor
	/**
	 * @brief constructor
	 */
	template< typename T >
	inline
	gridsolid<T>::gridsolid()
	: basic_gridmap3d<T>()
	  {

	  }

	/**
	 * @brief destructor
	 */
	template< typename T >
	inline
	gridsolid<T>::~gridsolid()
	{

	}
	// <---------------------------------------- constructor, destructor


	// ----------------------------------------> allocator, deallocator

	/**
	 * @brief allocate
	 * @param   sx : x component size
	 * @param   sy : y component size
	 * @param   sz : z component size
	 * @param rslx : x component resolution
	 * @param rsly : y component resolution
	 * @param rslz : z component resolution
	 */
	template< typename T >
	inline
	int gridsolid<T>::sallocate(const double sx, const double sy, const double sz, const double rslx, const double rsly, const double rslz)
	{
		unsigned long xi, yi, zi;
		int ret;

		if( rslx <= 0 || rsly <= 0 || rslz <= 0 )	return -1;

		// compute size of row and column
		xi = ::ceil(sx / rslx);
		yi = ::ceil(sy / rsly);
		zi = ::ceil(sz / rslz);
		// allocate
		if( (ret = basic_gridmap3d<T>::allocate(xi, yi, zi)) < 0 )	return ret;
		// set resolution
		_rsl.x = rslx;
		_rsl.y = rsly;
		_rsl.z = rslz;
		return ret;
	}


	/**
	 * @brief allocate
	 * @param   nx : nunber of grid (x)
	 * @param   ny : nunber of grid (y)
	 * @param   nz : nunber of grid (z)
	 * @param rslx : x component resolution
	 * @param rsly : y component resolution
	 * @param rslz : y component resolution
	 */
	template< typename T >
	inline
	int gridsolid<T>::sallocate(const int nx, const int ny, const int nz, const double rslx, const double rsly, const double rslz)
	{
		int ret;

		if( rslx <= 0 || rsly <= 0 || rslz <= 0 )	return -1;

		// allocate
		if( (ret = basic_gridmap3d<T>::allocate(nx, ny, nz)) < 0 )	return ret;
		// set resolution
		_rsl.x = rslx;
		_rsl.y = rsly;
		_rsl.z = rslz;
		return ret;
	}


	/**
	 * @brief deallocate
	 */
	template< typename T >
	inline
	int gridsolid<T>::deallocate()
	{
		int ret;

		if( (ret = basic_gridmap3d<T>::deallocate()) < 0 ) return ret;

		_orgn.x = 0;
		_orgn.y = 0;
		_orgn.z = 0;
		_rsl.x = 0;
		_rsl.y = 0;
		_rsl.z = 0;

		return ret;
	}


	/**
	 * @brief deallocate
	 * @param[in] xs : requiring x component size
	 * @param[in] ys : requiring y component size
	 */
	template< typename T >
	inline
	int gridsolid<T>::reallocate(const double sx, const double sy, const double sz)
	{
		long xi = 0, yi = 0, zi = 0;
		int ret;

		// compute access index of row and column
		if( sindex(sx, sy, sz, &xi, &yi, &zi) == 0 )	return 0;

		{ // ---> operation
			unsigned long cxi = xi < 0 ? ::ceil( - (double) xi / basic_gridmap3d<T>::_blocksize.x ) : 0;
			unsigned long cyi = yi < 0 ? ::ceil( - (double) yi / basic_gridmap3d<T>::_blocksize.y ) : 0;
			unsigned long czi = zi < 0 ? ::ceil( - (double) zi / basic_gridmap3d<T>::_blocksize.z ) : 0;
			unsigned long sxi;
			unsigned long syi;
			unsigned long szi;

			// compute required size of row and column
			if( xi < 0 )											sxi = cxi + basic_gridmap3d<T>::xsize();
			else if( xi >= (signed)basic_gridmap3d<T>::xsize())		sxi = xi + 1;
			else 													sxi = basic_gridmap3d<T>::xsize();
			if( yi < 0 )											syi = cyi + basic_gridmap3d<T>::ysize();
			else if( yi >= (signed)basic_gridmap3d<T>::ysize())		syi = yi + 1;
			else 													syi = basic_gridmap3d<T>::ysize();
			if( zi < 0 )											szi = czi + basic_gridmap3d<T>::zsize();
			else if( zi >= (signed)basic_gridmap3d<T>::zsize())		szi = zi + 1;
			else 													szi = basic_gridmap3d<T>::zsize();

			// reallocate
			if( (ret = basic_gridmap3d<T>::reallocate(sxi, syi, szi, cxi, cyi, czi)) < 0 ) return ret;
			// adjust origin
			if( cxi != 0 ){
				_orgn.x -= _rsl.x * ::ceil( (double)cxi ) * basic_gridmap3d<T>::_blocksize.x;
			}
			if( cyi != 0 ){
				_orgn.y -= _rsl.y * ::ceil( (double)cyi ) * basic_gridmap3d<T>::_blocksize.y;
			}
			if( czi != 0 ){
				_orgn.z -= _rsl.z * ::ceil( (double)czi ) * basic_gridmap3d<T>::_blocksize.z;
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
	inline int gridsolid<T>::sset_core(const double x, const double y, const double z)
	{
		_orgn.x = x - (basic_gridmap3d<T>::xsize() * _rsl.x) / 2.0;
		_orgn.y = y - (basic_gridmap3d<T>::ysize() * _rsl.y) / 2.0;
		_orgn.z = z - (basic_gridmap3d<T>::zsize() * _rsl.z) / 2.0;
		return 0;
	}


	/**
	 * @brief set origin position
	 * @param[in] x : x component value
	 * @param[in] y : y component value
	 */
	template< typename T >
	inline
	int gridsolid<T>::sset_origin(const double x, const double y, const double z)
	{
		_orgn.x = x;
		_orgn.y = y;
		_orgn.z = z;
		return 0;
	}


	/**
	 * @brief get core position
	 * @param[out] x : x component value
	 * @param[out] y : y component value
	 * @param[out] z : z component value
	 */
	template< typename T >
	inline
	int gridsolid<T>::sget_origin(double *x, double *y, double *z)
	{
		*x = _orgn.x;
		*y = _orgn.y;
		*z = _orgn.z;
		return 0;
	}


	/**
	 * @brief get pixel index
	 * @param[in]   x : x component value
	 * @param[in]   y : y component value
	 * @param[in]   z : z component value
	 * @param[out] xi : x index
	 * @param[out] yi : y index
	 * @param[out] zi : z index
	 * @return < 0 : out of range
	 */
	template< typename T >
	inline
	int gridsolid<T>::sindex(const double x, const double y, const double z, long *xi, long *yi, long *zi)
	{
		if( !basic_gridmap3d<T>::is_allocate() ) return -1;

		{ // ---> compute index
			double xx = x - _orgn.x;
			double yy = y - _orgn.y;
			double zz = z - _orgn.z;
			*xi = ::floor( xx / _rsl.x );
			*yi = ::floor( yy / _rsl.y );
			*zi = ::floor( zz / _rsl.z );
		} // <--- compute index

		return (*xi >= 0 && *xi < (signed)basic_gridmap3d<T>::xsize() &&
				*yi >= 0 && *yi < (signed)basic_gridmap3d<T>::ysize() &&
				*zi >= 0 && *zi < (signed)basic_gridmap3d<T>::zsize()) ? 0 : -1;
	}


	/**
	 * @brief get pointer
	 * @param[in]  x : x component value
	 * @param[in]  y : y component value
	 * @return 0 : not exit
	 */
	template< typename T >
	inline
	T* gridsolid<T>::spointer(const double x, const double y, const double z)
	{
		long xi, yi, zi;

		if( sindex(x, y, z, &xi, &yi, &zi) < 0 ) return 0;

		return basic_gridmap3d<T>::pointer(xi, yi, zi);
	}

	/**
	 * @brief get value
	 * @param[in]  x : x component value
	 * @param[in]  y : y component value
	 * @param[in]  z : z component value
	 * @return voxel's value
	 */
	template< typename T >
	inline
	T gridsolid<T>::svalue(const double x, const double y, const double z)
	{
		return * spointer(x, y, z);
	}

	/**
	 * @brief x lower bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::xlower()
	{
		return _orgn.x;
	}

	/**
	 * @brief x upper bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::xupper()
	{
		return _orgn.x + basic_gridmap3d<T>::xsize() * _rsl.x;
	}

	/**
	 * @brief y lower bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::ylower()
	{
		return _orgn.y;
	}

	/**
	 * @brief y upper bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::yupper()
	{
		return _orgn.y + basic_gridmap3d<T>::ysize() * _rsl.y;
	}

	/**
	 * @brief x lower bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::zlower()
	{
		return _orgn.z;
	}

	/**
	 * @brief z upper bound
	 */
	template< typename T >
	inline
	double gridsolid<T>::zupper()
	{
		return _orgn.z + basic_gridmap3d<T>::zsize() * _rsl.z;
	}


	/**
	 * @brief get value
	 * @param[in]  x : x component value
	 * @param[in]  y : y component value
	 * @param[in]  z : z component value
	 * @param[out] v : pixel's value
	 */
	template< typename T >
	inline
	int gridsolid<T>::sget(const double x, const double y, const double z, pt v)
	{
		long xi, yi, zi;

		if(!v)	return -1;
		if( sindex(x, y, z, &xi, &yi, &zi) < 0 ) return -1;
		::memcpy(v, basic_gridmap3d<T>::pointer(xi, yi, zi), sizeof(T));
		return 0;
	}

	/**
	 * @brief set value
	 * @param[in] x : x component value
	 * @param[in] y : y component value
	 * @param[in] v : pixel's value
	 */
	template< typename T >
	inline
	int gridsolid<T>::sset(const double x, const double y, const double z, const_pt v)
	{
		long xi = 0, yi = 0, zi = 0;
		if(!v)	return -1;

		for( int ret = sindex(x, y, z, &xi, &yi, &zi); ret < 0; ret = sindex(x, y, z, &xi, &yi, &zi)){
			int ret_realloc;
			if( (ret_realloc = reallocate(x,y,z)) < 0) return -1;
		}
		::memcpy(basic_gridmap3d<T>::pointer(xi, yi, zi), v, sizeof(T));
		return 0;
	}

	/**
	 * @brief x resolution
	 */
	template< typename T >
	inline
	double gridsolid<T>::xrsl()
	{
		return _rsl.x;
	}

	/**
	 * @brief y resolution
	 */
	template< typename T >
	inline
	double gridsolid<T>::yrsl()
	{
		return _rsl.y;
	}

	/**
	 * @brief z resolution
	 */
	template< typename T >
	inline
	double gridsolid<T>::zrsl()
	{
		return _rsl.z;
	}

	/**
	 * @brief origin's x component value
	 */
	template< typename T >
	inline
	double gridsolid<T>::xorg()
	{
		return _orgn.x;
	}

	/**
	 * @brief origin's y component value
	 */
	template< typename T >
	inline
	double gridsolid<T>::yorg()
	{
		return _orgn.y;
	}

	/**
	 * @brief origin's z component value
	 */
	template< typename T >
	inline
	double gridsolid<T>::zorg()
	{
		return _orgn.z;
	}


	/**
	 * @brief set resolution
	 */
	template< typename T >
	inline
	int gridsolid<T>::sset_rsl(double x, double y, double z)
	{
		_rsl.x = x;
		_rsl.y = y;
		_rsl.z = z;
		return 0;
	}


	/**
	 * @brief get a pixel lower bounds
	 */
	template< typename T >
	inline
	int gridsolid<T>::sget_pos_lower(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z)
	{
		if(x)	*x = _orgn.x + xi * _rsl.x;
		if(y)	*y = _orgn.y + yi * _rsl.y;
		if(z)	*z = _orgn.z + zi * _rsl.z;
		return 0;
	}


	/**
	 * @brief get a pixel upper bounds
	 */
	template< typename T >
	inline
	int gridsolid<T>::sget_pos_upper(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z)
	{
		sget_pos_lower(xi, yi, zi, x, y, z);
		if(x)	*x += _rsl.x;
		if(y)	*y += _rsl.y;
		if(z)	*z += _rsl.z;
		return 0;
	}


	/**
	 * @brief get a pixel core position
	 */
	template< typename T >
	inline
	int gridsolid<T>::sget_pos_core(const unsigned long xi, const unsigned long yi, const unsigned long zi, double *x, double *y, double *z)
	{
		sget_pos_lower(xi, yi, zi, x, y, z);
		if(x)	*x += _rsl.x / 2.0;
		if(y)	*y += _rsl.y / 2.0;
		if(z)	*z += _rsl.z / 2.0;
		return 0;
	}

	// <---------------------------------------- setter, getter





	// ---> read write
	/**
	 * @brief file write
	 * @param[in] fname : file name
	 */
	template< typename T >
	inline
	int gridsolid<T>::fwrite(const char *fname)
	{
		int fd;
		int ret;
		int fsize;

		{ // ---> initialize
			fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT,
					S_IRUSR | S_IWUSR |
					S_IRGRP | S_IWGRP);
			if(fd < 0) return -1;
			fsize = 0;
		} // <--- initialize


		{ // ---> operation
			ssize_t bsize = 0;
			GridMap3D::voxelindex buf;

			// save file tag
			if( (ret = ::write(fd, GridMap3D::__GridSolidFileTag__, GridMap3D::__FileTagSize__)) != GridMap3D::__FileTagSize__ )
				return -1;
			fsize += ret;

			// file header
			buf = basic_gridmap3d<T>::_blocksize;
			if( (ret = ::write(fd, &buf, sizeof(buf) ) ) != sizeof( buf ) )
				return -1;
			fsize += ret;
			buf = basic_gridmap3d<T>::_mapsize;
			if( (ret = ::write(fd, &buf, sizeof(buf) ) ) != sizeof( buf ) )
				return -1;
			fsize += ret;

			// file header
			if( (ret = ::write(fd, &_orgn, sizeof(_orgn)) ) != sizeof(_orgn) )
				return -1;
			fsize += ret;
			if( (ret = ::write(fd, &_rsl, sizeof(_rsl)) ) != sizeof(_orgn) )
				return -1;
			fsize += ret;

			// ---> data
			bsize = sizeof(T) * basic_gridmap3d<T>::_blocksize.x * basic_gridmap3d<T>::_blocksize.y * basic_gridmap3d<T>::_blocksize.z;
			for(size_t z = 0; z < basic_gridmap3d<T>::_mapsize.z; z++){
				for(size_t y = 0; y < basic_gridmap3d<T>::_mapsize.y; y++){
					for(size_t x = 0; x < basic_gridmap3d<T>::_mapsize.x; x++){
						if( (ret = ::write(fd, basic_gridmap3d<T>::_header[z][y][x], bsize) ) != bsize ) {
							return -1;
						}
						fsize += ret;
					} // <--- for(x)
				} // <--- for(y)
			} // <--- for(z)
			// <--- data
		} // <--- operation


		{ // ---> finalize
			close(fd);
		} // <--- finalize

		return fsize;
	}


	/**
	 * @brief file read
	 * @param[in] fname : file name
	 */
	template< typename T >
	inline
	int gridsolid<T>::fread(const char *fname)
	{
		int fd;
		int ret;
		int fsize;

		{ // ---> initialize
			fd = open(fname, O_RDWR);
			if(fd < 0) return -1;
			fsize = 0;
		} // <--- initialize

		{ // ---> operation
			unsigned char buf[GridMap3D::__FileTagSize__+1];	// buffer to read file tag
			GridMap3D::voxelindex	b,							// block's size
			m;							// map's size
			ssize_t bsize = 0;									// block's memory size
			::memset(buf, 0, sizeof(buf));
			// save file tag
			if( (ret = ::read(fd, buf, GridMap3D::__FileTagSize__)) != (signed) GridMap3D::__FileTagSize__ )
				return -1;
			fsize += ret;
			if( ::memcmp(buf, GridMap3D::__GridSolidFileTag__, GridMap3D::__FileTagSize__ ))
				return -1;

			// file header
			if( (ret = ::read(fd, &b, sizeof(b)) ) != sizeof(b) )
				return -1;
			fsize += ret;
			if( (ret = ::read(fd, &m, sizeof(m)) ) != sizeof(m) )
				return -1;
			fsize += ret;

			// file header
			if( (ret = ::read(fd, &_orgn, sizeof(_orgn)) ) != sizeof(_orgn) )
				return -1;
			fsize += ret;
			if( (ret = ::read(fd, &_rsl, sizeof(_rsl)) ) != sizeof(_orgn) )
				return -1;
			fsize += ret;

			// allocate
			if( basic_gridmap3d<T>::is_allocate() ) basic_gridmap3d<T>::deallocate();
			if( basic_gridmap3d<T>::allocate(b.x, b.y, b.z, m.x, m.y, m.z) < 0 ) return -1;

			// ---> data
			bsize = sizeof(T) * basic_gridmap3d<T>::_blocksize.x * basic_gridmap3d<T>::_blocksize.y * basic_gridmap3d<T>::_blocksize.z;
			for(size_t z = 0; z < basic_gridmap3d<T>::_mapsize.z; z++){
				for(size_t y = 0; y < basic_gridmap3d<T>::_mapsize.y; y++){
					for(size_t x = 0; x < basic_gridmap3d<T>::_mapsize.x; x++){
						if( (ret = ::read(fd, basic_gridmap3d<T>::_header[z][y][x], bsize) ) != bsize ) {
							return -1;
						}
						fsize += ret;
					} // <--- for(x)
				} // <--- for(y)
			} // <--- for(z)
			// <--- data

		} // <--- operation


		{ // ---> finalize
			close(fd);
		} // <--- finalize

		return fsize;
	}

	template< typename T >
	inline
	int gridsolid<T>::show_param( FILE* fp ) {
		::fprintf(fp, "          origin : %lf %lf %lf\n", _orgn.x, _orgn.y, _orgn.z);
		::fprintf(fp, "       grid size : %lf %lf %lf\n", _rsl.x, _rsl.y, _rsl.z);
		::fprintf(fp, "         x range : %lf : %lf\n", xlower(), xupper());
		::fprintf(fp, "         y range : %lf : %lf\n", ylower(), yupper());
		::fprintf(fp, "         z range : %lf : %lf\n", zlower(), zupper());
		::fprintf(fp, "      total size : %ld x %ld x %ld\n", basic_gridmap3d<T>::xsize(), basic_gridmap3d<T>::ysize(), basic_gridmap3d<T>::zsize());
		::fprintf(fp, "   size of block : %d x %d x %d\n", basic_gridmap3d<T>::_blocksize.x, basic_gridmap3d<T>::_blocksize.y, basic_gridmap3d<T>::_blocksize.z);
		::fprintf(fp, "number  of block : %d x %d x %d\n", basic_gridmap3d<T>::_mapsize.x,   basic_gridmap3d<T>::_mapsize.y,   basic_gridmap3d<T>::_mapsize.z);
		return 0;
	}


} // <--- namespace gnd
// <--- class definition



#endif /* GND_GRIDMAP3D_HPP_ */
