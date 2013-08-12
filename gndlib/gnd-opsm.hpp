/*
 * gnd-opsm.hpp
 * observed probability scan matching
 *
 *  Created on: 2011/12/02
 *      Author: tyamada
 */

#ifndef GND_OPSM_HPP_
#define GND_OPSM_HPP_

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdint.h>

#include "gnd-optimize.hpp"
#include "gnd-gridmap.hpp"
#include "gnd-random.hpp"
#include "gnd-bmp.hpp"
#include "gnd-util.h"
#include "gnd-queue.hpp"
#include "gnd-linalg.hpp"
#include "gnd-matrix-base.hpp"
#include "gnd-matrix-coordinate.hpp"
#include "gnd-lib-error.h"



// include debug logging function
#define GND_DEBUG_LOG_NAMESPACE1 gnd
#define GND_DEBUG_LOG_NAMESPACE2 opsm
#include "gnd-debug-log.hpp"
#undef GND_DEBUG_LOG_NAMESPACE2
#undef GND_DEBUG_LOG_NAMESPACE1
#undef GND_DEBUG_LOG

#include "gnd-debug-log-util-def.h"

/**
 * @ifnot GNDPSM
 * @defgroup GNDPSM probabilistic scan matching
 * @endif
 */



// ---> class declaration
namespace gnd  {
	namespace opsm {
		class optimize_basic;
		class optimize_newton;
		typedef optimize_newton newton;
		class optimize_monte_calro_method;
		typedef optimize_monte_calro_method mcl;
		class optimize_quasi_monte_calro;
		typedef optimize_quasi_monte_calro qmc;
		class optimize_hybrid_qmc2newton;
		typedef optimize_hybrid_qmc2newton hybrid_q2n;
	}

}
// <--- class declaration



// ---> constant and structure definition
namespace gnd {
	namespace opsm {

		/**
		 * @brief number of map data plane
		 */
		static const size_t PlaneNum = 4;

		static const double SqrtDBL_EPSILON = ::sqrt(DBL_EPSILON);

		/*
		 * @brief number of map data plane
		 */
		static const double ErrorMargin = SqrtDBL_EPSILON;

		/**
		 * @brief default file out directory
		 */
		static const char CMapDirectoryDefault[] = ".";

		/**
		 * @brief counting map file name format
		 */
		static const char CMapFileNameFormat[] = "%s/%s.%02d.%s";

		/**
		 * @brief default counting map file name
		 */
		static const char CMapFileNameDefault[] = "opsm";

		/**
		 * @brief default counting map file extension
		 */
		static const char CMapFileExtension[] = "cmap";

		/**
		 * @brief default map data cell size
		 */
		static const double DefaultMapCellSize = 5;

		/**
		 * @brief reflection point position index
		 */
		enum {
			PosX = 0,
			PosY = 1,
			PosDim = 2,
		};


		/**
		 * @brief statistics counting map for probabilistic scan matching
		 */
		struct counting_map_pixel {
			/// @brief sum of reflection point position value
			matrix::fixed< PosDim,1 > pos_sum;
			/// @brief sum of covariance
			matrix::fixed< PosDim,PosDim > cov_sum;
			/// @brief number of reflection
			uint64_t cnt;
			/// @brief constructor
			counting_map_pixel() : cnt(0) {}
		};
		/**
		 * @typedef counting_pixel_t
		 * @see counting_map_pixel
		 */
		typedef struct counting_map_pixel cmap_pixel_t;

		/**
		 * @ingroup GNDPSM
		 * @brief counting map
		 */
		struct counting_map {
			/// @brief four planes
			gridmap::gridplane<cmap_pixel_t> plane[PlaneNum];
		};
		/**
		 * @ingroup GNDPSM
		 * @typedef cmap_t
		 * @see counting_map
		 */
		typedef struct counting_map cmap_t;



		/**
		 * @brief probabilistic scam matching pixel of map
		 */
		struct map_pixel {
			/// mean
			matrix::fixed< PosDim,1 > mean;
			/// inverse matrix of covariance
			matrix::fixed< PosDim,PosDim > inv_cov;
			/// weight ( N / sqrt(|Sigma|)  )
			double K;
			/// number of points
			uint64_t N;
			/// @brief constructor
			map_pixel() : K(0.0), N(0) {}
		};
		/**
		 * @typedef pixel_t
		 * @see map_pixel
		 */
		typedef struct map_pixel pixel_t;

		/**
		 * @ingroup GNDPSM
		 * @brief map
		 */
		struct scanmatching_map {
			/// @brief four planes
			gridmap::gridplane<pixel_t> plane[PlaneNum];
		};
		/**
		 * @typedef map_t
		 * @see scanmatching_map
		 */
		typedef struct scanmatching_map map_t;

		/**
		 * @ingroup GNDPSM
		 * @brief position_gain
		 * @note sum of laser points on map in the range of robot sensor
		 */
		struct pos_gain {
			uint64_t N[PlaneNum];
		};
		/**
		 * @typedef gain_t
		 * @see pos_gain
		 */
		typedef struct pos_gain pgain_t;
	}
};
// <--- constant and structure definition




// ---> function declaration
namespace gnd {
	namespace opsm {

		int init_counting_map(cmap_t *m, double p, double u = DefaultMapCellSize);
		int clear_counting_map(cmap_t *m);
		int destroy_counting_map(cmap_t *m);

		int counting_map(cmap_t *m, double x, double y);

		int update_map(cmap_t *cnt, map_t *map, double x, double y, double err = ErrorMargin );
		int update_ndt_map(cmap_t *c, map_t *m, double x, double y);
		int build_map(map_t *map, cmap_t *cnt, double err = ErrorMargin, double *max = 0);
		int build_ndt_map(map_t *map, cmap_t *cnt, double err = ErrorMargin);
		int destroy_map(map_t *m);

		int position_gain(map_t *map, double x, double y, double r, pgain_t *pg);
		int likelihood(map_t *m, double x, double y, double *l);
		int likelihood(map_t *m, double x, double y, pgain_t *pg, double *l);
		int gradient(map_t *m, double x, double y, matrix::fixed<4,4> *c, matrix::fixed<3,1> *g, double *l = 0);

		int read_counting_map(cmap_t *c,  const char* d = CMapDirectoryDefault, const char* f = CMapFileNameDefault, const char* e = CMapFileExtension);
		int write_counting_map(cmap_t *c,  const char* d = CMapDirectoryDefault, const char* f = CMapFileNameDefault, const char* e = CMapFileExtension);

		int build_bmp(bmp8_t *b, map_t *m, double p = 0.1, double sr = 0.0, double cp = 4.0);
		int build_bmp(bmp32_t *b, map_t *m, double p = 0.1, double sr = 0.0, double cp = 4.0);
		int build_bmp8(bmp8_t *b, map_t *m, double p = 0.1, double sr = 0.0, double cp = 4.0);
		int build_bmp32(bmp32_t *b, map_t *m, double p = 0.1, double sr = 0.0, double cp = 4.0);
	}
};
// <--- function declaration



// ---> function definition
namespace gnd {
	namespace opsm {

		/**
		 * @ingroup GNDPSM
		 * @brief initialize counting map
		 * @param[out] m : counting map
		 * @param[in]  p : pixel size
		 * @param[in]  u : grid map allocate unit size
		 */
		inline
		int init_counting_map(cmap_t *m, double p, double u)
		{
			gnd_assert(!m, -1, "invalid null pointer");
			gnd_assert(p <= 0, -1, "invalid argument. grid size must be greater than 0.");
			gnd_assert(u < p, -1, "invalid argument. cell size must be greater than 0.");

			LogDebugf("Begin - init_counting_map(%p, %lf, %lf)\n", m, p, u);
			LogIndent();

			// ---> plane scan loop
			for( size_t i = 0; i < PlaneNum; i++){
				// check error
				if( m->plane[i].is_allocate() < 0) {
					LogDebug("this map is buzy\n");
					LogUnindent();
					LogDebugf(" Fail - init_counting_map(%p, %lf, %lf)\n", m, p, u);
					gnd_exit(-1, "invalid argument, this map is buzy");
				}

				// allocate memory cell
				if( m->plane[i].pallocate(u, u, p, p) < 0) {
					LogDebug("fail to allocate\n");
					LogUnindent();
					LogDebugf("Fail  - init_counting_map(%p, %lf, %lf)\n", m, p, u);
					gnd_exit(-1, "fail to allocate");
				}
				else {
					LogVerbosef("map[%d] allocated\n", i);
				}
				// set origin
				m->plane[i].pset_core( (i % 2) * (p / 2.0), ((i / 2) % 2) * (p / 2.0) );
			} // <--- plane scan loop

			LogUnindent();
			LogDebugf("End   - init_counting_map(%p, %lf, %lf)\n", m, p, u);
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief clear counting map
		 * @param[out] m :
		 */
		int clear_counting_map(cmap_t *m) {
			gnd_assert(!m, -1, "invalid null pointer");
			gnd_assert(!(m->plane+0) || !(m->plane + 1) || !(m->plane + 2) || !(m->plane + 3), -1, "invalid null pointer");

			{ // ---> operation
				opsm::counting_map_pixel ini;

				for( size_t i = 0; i < opsm::PlaneNum; i++){
					m->plane[i].set_uniform(&ini);
				}
			} // <--- operation
			return 0;
		}

		/**
		 * @ingroup GNDPSM
		 * @brief release counting map
		 * @param[out] m : counting map
		 */
		inline
		int destroy_counting_map(cmap_t *m)
		{
			gnd_assert(!m, -1, "invalid null pointer");

			LogDebugf("Begin - destroy_counting_map(%p)\n", m);
			LogIndent();

			for( size_t i = 0; i < PlaneNum; i++){
				m->plane[i].deallocate();
			}

			LogUnindent();
			LogDebugf("  End - destroy_counting_map(%p)\n", m);
			return 0;
		}

		/**
		 * @ingroup GNDPSM
		 * @brief map counting function
		 * @param[out] m : counting map
		 * @param[in] x : laser scanner reflection point x
		 * @param[in] y : laser scanner reflection point y
		 */
		inline
		int counting_map(cmap_t *m, double x, double y)
		{
			gnd_assert(!m, -1, "invalid null argument");

			{ // ---> operate
				matrix::fixed<PosDim, 1> xx;

				set(&xx, PosX, 0, x);
				set(&xx, PosY, 0, y);

				// ---> plene scanning loop
				for(size_t i = 0; i < PlaneNum; i++){
					cmap_pixel_t *pp = 0;				// reference of pixel data
					matrix::fixed<PosDim, 1>	core;		// pixel core position
					matrix::fixed<PosDim, 1>	pos;		// reflection point on pixel
					matrix::fixed<PosDim, PosDim>	cov;	// reflection point covariance in pixel
					long r = 0, c = 0;						//

					// if memory is lacking, map data reallocate
					for( pp = m->plane[i].ppointer( x, y );
							pp == 0;
							pp = m->plane[i].ppointer( x, y ) ){
						m->plane[i].reallocate(x, y);
					}

					// get row and column number of reflection point pixel
					m->plane[i].pindex(x, y, &r, &c);
					// get core position of reflection point pixel
					m->plane[i].pget_pos_core(r, c, pointer(&core, PosX, 0), pointer(&core, PosY, 0));

					// compute reflection point on pixel
					sub(&xx, &core, &pos);

					// compute covariance
					prod_transpose2(&pos, &pos, &cov);

					// add date
					add(&pp->pos_sum, &pos, &pp->pos_sum);
					add(&pp->cov_sum, &cov, &pp->cov_sum);
					pp->cnt++;
				} // <--- plene scanning loop
			} // <--- operate
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief map update
		 * @param[out] cnt : counting map
		 * @param[out]    map : map
		 * @param[in]       x : laser scanner reflection point x
		 * @param[in]       y : laser scanner reflection point y
		 * @param[in]     err : margin of some kinds of error ( such as rounding error sensor resolution  )
		 */
		inline
		int update_map(cmap_t *cnt, map_t *map, double x, double y, double err )
		{
			gnd_assert(!map, -1, "invalid null argument");

			{ // ---> operation
				int ret;
				matrix::fixed<PosDim,PosDim> ws2x2;	// workspace 2x2 matrix
				matrix::fixed<PosDim,PosDim> cov;		// covariance matrix
				long r = 0, c = 0;

				if( (ret = counting_map(cnt, x, y)) < 0){
					return ret;
				}
				// ---> for each plane
				for(size_t i = 0; i < PlaneNum; i++){
					cmap_pixel_t *cpp;
					pixel_t *pp;

					if( !map->plane[i].is_allocate() ) {
						map->plane[i].allocate( cnt->plane[i]._unit_row_(), cnt->plane[i]._unit_column_(),
								cnt->plane[i]._plane_row_(), cnt->plane[i]._plane_column_());
						map->plane[i].pset_origin( cnt->plane[i].xlower(), cnt->plane[i].ylower());
						map->plane[i].pset_rsl( cnt->plane[i].xrsl(), cnt->plane[i].yrsl());
					}

					// if memory is lacking, map data reallocate
					for( ret = map->plane[i].pindex(x, y, &r, &c);
							ret < 0;
							ret = map->plane[i].pindex(x, y, &r, &c) ){
						map->plane[i].reallocate(x, y);
						LogDebug("reallocate\n");
					}

					// get pointer
					if( !(pp = map->plane[i].ppointer(x,y)) ) continue;
					if( !(cpp = cnt->plane[i].ppointer(x,y)) ) continue;

					{ // ---> obtain the number of points
						pp->N = cpp->cnt;
					} // <--- obtain the number of points

					// ---> obtain mean and inverse matrix of co-variance
					if(cpp->cnt <= 3){
						pp->K = 0;
						continue;
					}
					else {
						// compute mean
						scalar_div(&cpp->pos_sum, (double)cpp->cnt, &pp->mean );

						// compute covariance
						prod_transpose2(&pp->mean, &cpp->pos_sum, &ws2x2);
						sub(&cpp->cov_sum, &ws2x2, &ws2x2);
						scalar_div(&ws2x2, (double)cpp->cnt, &cov);

						// add minimal diagonal matrix
						set_unit(&ws2x2);
						scalar_prod(&ws2x2, gnd_square(err) , &ws2x2);
						add(&cov, &ws2x2, &cov);

						// compute inverse covariance
						inverse(&cov, &pp->inv_cov);
					}
					// --->obtain mean and inverse matrix of co-variance


					{ // ---> compute evaluation gain
						double det;

						// obtain determinant of co-variance matrix
						if( matrix::det(&cov, &det) < 0) {
							pp->K = 0;
							continue;
						}

						pp->K = (double) (cpp->cnt) / ( ::sqrt(det) );
					} // <--- compute evaluation gain

				} // <--- for each plane

			} // <--- operation

			return 0;
		}


		/**
		 * @privatesection
		 * @ingroup GNDPSM
		 * @brief map counting function (ndt)
		 * @param[in,out] cnt : counting map
		 * @param[out]    map : map
		 * @param[in]       x : laser scanner reflection point x
		 * @param[in]       y : laser scanner reflection point y
		 * @param[in]     err : minimum error
		 */
		inline
		int update_ndt_map(cmap_t *cnt, map_t *map, double x, double y, double err)
		{
			gnd_assert(!map, -1, "invalid null argument");

			{ // ---> operation
				int ret;
				matrix::fixed<PosDim,PosDim> ws2x2;	// workspace 2x2 matrix
				matrix::fixed<PosDim,PosDim> cov;		// covariance matrix
				long r, c;

				if( (ret = counting_map(cnt, x, y)) < 0) return ret;
				// ---> for each plane
				for(size_t i = 0; i < PlaneNum; i++){
					cmap_pixel_t *cpp;
					pixel_t *pp;

					if( !map->plane[i].is_allocate() ) {
						map->plane[i].allocate( cnt->plane[i]._unit_row_(), cnt->plane[i]._unit_column_(),
								cnt->plane[i]._plane_row_(), cnt->plane[i]._plane_column_());
						map->plane[i].pset_origin( cnt->plane[i].xlower(), cnt->plane[i].ylower());
						map->plane[i].pset_rsl( cnt->plane[i].xrsl(), cnt->plane[i].yrsl());
					}

					// if memory is lacking, map data reallocate
					for( ret = map->plane[i].pindex(x, y, &r, &c);
							ret < 0;
							ret = map->plane[i].pindex(x, y, &r, &c) ){
						map->plane[i].reallocate(x, y);
					}

					// get pointer
					if( !(pp = map->plane[i].ppointer(x,y)) ) continue;
					if( !(cpp = cnt->plane[i].ppointer(x,y)) ) continue;

					{ // ---> obtain the number of points
						pp->N = 1;
					} // <--- obtain the number of points

					// if 0
					if(cpp->cnt <= 3){
						pp->K = 0;
						continue;
					}
					else {
						// compute mean
						scalar_div(&cpp->pos_sum, (double)cpp->cnt, &pp->mean );

						// compute covariance
						prod_transpose2(&pp->mean, &cpp->pos_sum, &ws2x2);
						sub(&cpp->cov_sum, &ws2x2, &ws2x2);
						scalar_div(&ws2x2, (double)cpp->cnt, &cov);

						// add minimal diagonal matrix
						set_unit(&ws2x2);
						scalar_prod(&ws2x2, gnd_square(err) , &ws2x2);
						add(&cov, &ws2x2, &cov);

						// compute inverse covariance
						inverse(&cov, &pp->inv_cov);
					}

					{ // ---> compute evaluation gain
						pp->K = 1.0;
					} // <--- compute evaluation gain

				} // <--- for each plane

			} // <--- operation

			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief map building function
		 * @param[out]    map : builded map
		 * @param[in]     cnt : laser scanner reflection point counting data
		 * @param[in]     err : minimum error
		 * @param[out]   maxk : maximum gain
		 */
		inline
		int build_map(map_t *map, cmap_t *cnt, double err, double *maxk) {
			gnd_assert(!cnt, -1, "invalid null pointer");
			gnd_assert(!map, -1, "map is null");

			LogDebugf("Begin - int build_map(%p, %p, %lf, %lf)\n", map, cnt, err, maxk);
			LogIndent();

			{ // ---> operation
				matrix::fixed<PosDim,PosDim> ws2x2;	// workspace 2x2 matrix
				matrix::fixed<PosDim,PosDim> cov;	// covariance matrix

				if( maxk ) *maxk = 0;

				// ---> for each plane
				for(size_t i = 0; i < PlaneNum; i++){
					LogVerbosef("plane %d ...\n", i);

					if( map->plane[i].is_allocate() ){
						if( map->plane[i].xrsl() != cnt->plane[i].xrsl() || map->plane[i].yrsl() != cnt->plane[i].yrsl() ){
							LogDebug("fail to memeory allocate\n");
							LogUnindent();
							LogDebugf("Fail  - int build_map(%p, %p, %lf, %lf)\n", map, cnt, err, maxk);
							return -1;
						}
					}
					else {
						map->plane[i].allocate( cnt->plane[i]._unit_row_(), cnt->plane[i]._unit_column_(),
								cnt->plane[i]._plane_row_(), cnt->plane[i]._plane_column_());

						map->plane[i].pset_origin( cnt->plane[i].xlower(), cnt->plane[i].ylower());
						map->plane[i].pset_rsl( cnt->plane[i].xrsl(), cnt->plane[i].yrsl());
					}

					// ---> for each row
					for( unsigned long r = 0; r < cnt->plane[i].row(); r++){
						// ---> for each column
						for( unsigned long c = 0; c < cnt->plane[i].column(); c++){
							cmap_pixel_t *cpp;
							pixel_t *pp;
							double x, y;

							// get ndt data
							cpp = cnt->plane[i].pointer( r, c );
							cnt->plane[i].pget_pos_core(r, c, &x, &y);
							for( pp = map->plane[i].ppointer( x, y );
									pp == 0;
									pp = map->plane[i].ppointer( x, y ) ){
								map->plane[i].reallocate(x, y);
							}

							{ // ---> obtain the number of points
								pp->N = cpp->cnt;
							} // <--- obtain the number of points

							// ---> obtain mean and inverse matrix of co-variance
							if(cpp->cnt <= 3){
								pp->K = 0;
								continue;
							}
							else {
								// compute mean
								scalar_div(&cpp->pos_sum, (double)cpp->cnt, &pp->mean );

								// compute covariance
								prod_transpose2(&pp->mean, &cpp->pos_sum, &ws2x2);
								sub(&cpp->cov_sum, &ws2x2, &ws2x2);
								scalar_div(&ws2x2, (double)cpp->cnt, &cov);

								// add minimal diagonal matrix
								set_unit(&ws2x2);
								scalar_prod(&ws2x2, gnd_square(err) , &ws2x2);
								add(&cov, &ws2x2, &cov);

								// compute inverse covariance
								inverse(&cov, &pp->inv_cov);
							}
							// --->obtain mean and inverse matrix of co-variance


							{ // ---> compute evaluation gain
								double det = 0;

								// obtain determinant of co-variance matrix
								if( matrix::det(&cov, &det) < 0) {
									pp->K = 0;
									continue;
								}

								pp->K = (double) (cpp->cnt) / ( ::sqrt(det) );

								if( maxk && *maxk < pp->K ){
									*maxk = pp->K;
								}

							} // <--- compute evaluation gain

						} // <-- for each column
					} // <--- for each row

				} // <--- for each plane
			}  // <--- operation

			LogUnindent();
			LogDebugf("End   - int build_map(%p, %p, %lf, %lf)\n", map, cnt, err, maxk);
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief map building function
		 * @param[out] map : builded map
		 * @param[in]  cnt : laser scanner reflection point counting data
		 * @param[in]  err : laser scanner field range
		 */
		inline
		int build_ndt_map( map_t *map, cmap_t *cnt, double err )
		{
			gnd_assert(!cnt, -1, "invalid null pointer");
			gnd_assert(!map, -1, "map is null");

			LogDebugf("Begin - build_ndt_map(%p, %p, %lf)\n", map, cnt, err);
			LogIndent();

			{ // ---> operation
				matrix::fixed<PosDim,PosDim> ws2x2;	// workspace 2x2 matrix
				matrix::fixed<PosDim,PosDim> cov;		// covariance matrix

				// ---> for each plane
				for(size_t i = 0; i < PlaneNum; i++){
					LogVerbosef("plane %d ...\n", i);

					if( map->plane[i].is_allocate() ){
						if( map->plane[i].xrsl() != cnt->plane[i].xrsl() || map->plane[i].yrsl() != cnt->plane[i].yrsl() ){
							LogDebug("fail to allocate\n");
							LogUnindent();
							LogDebugf("Fail - build_ndt_map(%p, %p, %lf)\n", map, cnt, err);
							return -1;
						}
					}
					else {
						map->plane[i].allocate( cnt->plane[i]._unit_row_(), cnt->plane[i]._unit_column_(),
								cnt->plane[i]._plane_row_(), cnt->plane[i]._plane_column_());

						map->plane[i].pset_origin( cnt->plane[i].xlower(), cnt->plane[i].ylower());
						map->plane[i].pset_rsl( cnt->plane[i].xrsl(), cnt->plane[i].yrsl());
					}

					// ---> for each row
					for( unsigned long r = 0; r < cnt->plane[i].row(); r++){
						// ---> for each column
						for( unsigned long c = 0; c < cnt->plane[i].column(); c++){
							cmap_pixel_t *cpp;
							pixel_t *pp;
							double x, y;


							// get ndt data
							cpp = cnt->plane[i].pointer( r, c );
							cnt->plane[i].pget_pos_core(r, c, &x, &y);
							for( pp = map->plane[i].ppointer( x, y );
									pp == 0;
									pp = map->plane[i].ppointer( x, y ) ){
								map->plane[i].reallocate(x, y);
							}

							{ // ---> obtain the number of points
								pp->N = 1;
							} // <--- obtain the number of points

							// if 0
							if(cpp->cnt <= 3){
								pp->K = 0;
								continue;
							}
							else {
								// compute mean
								scalar_div(&cpp->pos_sum, (double)cpp->cnt, &pp->mean );

								// compute covariance
								prod_transpose2(&pp->mean, &cpp->pos_sum, &ws2x2);
								sub(&cpp->cov_sum, &ws2x2, &ws2x2);
								scalar_div(&ws2x2, (double)cpp->cnt, &cov);

								// add minimal diagonal matrix
								set_unit(&ws2x2);
								scalar_prod(&ws2x2, gnd_square(err) , &ws2x2);
								add(&cov, &ws2x2, &cov);

								// compute inverse covariance
								inverse(&cov, &pp->inv_cov);
							}

							pp->K = 1.0;

						} // <-- for each column
					} // <--- for each row

				} // <--- for each plane
			}  // <--- operation

			LogUnindent();
			LogDebugf("End - build_ndt_map(%p, %p, %lf)\n", map, cnt, err);
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief destory map
		 * @param[out] m :  map
		 */
		inline
		int destroy_map(map_t *m)
		{
			gnd_assert(!m, -1, "invalid null pointer");

			LogDebugf("Begin - destroy_map(%p)\n", m);
			LogIndent();

			for( size_t i = 0; i < PlaneNum; i++)
				m->plane[i].deallocate();

			LogUnindent();
			LogDebugf("END   - destroy_map(%p)\n", m);
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief count laser points in the range of robot sensor
		 * @param[in] map : map data
		 * @param[in]   x : robot position x
		 * @param[in]   y : robot position x
		 * @param[in]  sr : sensor range
		 * @param[out] pg : position gain
		 * @return <0 : error
		 */
		inline
		int position_gain(map_t *map, double x, double y, double sr, pgain_t *pg) {
			gnd_assert(!map, -1, "map is null");
			gnd_assert(!pg, -1, "output storage is null");

			{ // ---> operate
				long r = 0, c = 0;
				size_t f = ::floor( sr / map->plane[0].xrsl());

				pixel_t *tmp_cpp;
				unsigned long lowerr;	// search range lower row
				unsigned long upperr;	// search range upper row
				unsigned long lowerc;	// search range lower column
				unsigned long upperc;	// search range upper column

				// ---> obtain the sum of laser points in the sensor range for each plane
				for(unsigned int i = 0; i < PlaneNum; i++){
					// clear
					pg->N[i] = 0;

					// get row and column index at robot position
					map->plane[i].pindex(x, y, &r, &c);

					//
					lowerr = r < (signed)f ? 0 : r - f;
					upperr = r + f >= map->plane[i].row() ? map->plane[i].row() : r + f;
					lowerc = c < (signed)f ? 0 : c - f;
					upperc = c + f >= map->plane[i].column() ? map->plane[i].column() : c + f;

					// compute average of local area number of observed point
					for( unsigned long rr = lowerr; rr < upperr; rr++){
						for( unsigned long cc = lowerc; cc < upperc; cc++){
							tmp_cpp = map->plane[i].pointer( rr, cc );
							pg->N[i] += tmp_cpp->N;
						}
					}
				}
				// <--- obtain the sum of laser points in the sensor range for each plane

			} // <--- operate
			return 0;
		}


		/**
		 * @ingroup GNDPSM
		 * @brief compute likelihood
		 * @param[in] map : map data
		 * @param[in]   x : laser scanner reflection point x
		 * @param[in]   y : laser scanner reflection point y
		 * @param[out]  l : likelihood
		 */
		inline
		int likelihood(map_t *map, double x, double y, double *l){
			return likelihood(map, x, y, 0, l);
		}

		/**
		 * @ingroup GNDPSM
		 * @brief compute likelihood
		 * @param[in] map : map data
		 * @param[in]   x : laser scanner reflection point x
		 * @param[in]   y : laser scanner reflection point y
		 * @param[in]  pg : position gain
		 * @param[out]  l : likelihood
		 */
		inline
		int likelihood(map_t *map, double x, double y, pgain_t *pg, double *l){
			gnd_assert(!l, -1, "invalid null pointer");
			gnd_assert(!map, -1, "map is null");

			{ // ---> operate
				matrix::fixed<PosDim,1> pos;
				matrix::fixed<PosDim,1> q;
				matrix::fixed<1,PosDim> ws1x2;	// workspace 2x2 matrix
				matrix::fixed<1,1> ws1x1;		// workspace 1x1 matrix

				{ // ---> initialize
					set(&pos, PosX, 0, x);
					set(&pos, PosY, 0, y);
					*l = 0;
				} // <--- initialize


				// ---> for
				for( size_t i = 0; i < PlaneNum; i++){
					pixel_t *pp;
					int ret;
					long pr, pc;

					// get index of pixel
					ret = map->plane[i].pindex( value(&pos, PosX, 0), value(&pos, PosY, 0), &pr, &pc );
					// no data
					if( ret < 0 )			continue;

					// get pixel data
					pp = map->plane[i].pointer( pr, pc );
					// zero weight
					if(pp->K <= 0.0)	continue;

					// get pixel core pos on ndt data pixel
					map->plane[i].pget_pos_core(pr, pc, pointer(&q, PosX, 0), pointer(&q, PosY, 0));
					sub(&pos, &q, &q);

					// difference from mean
					sub(&q, &pp->mean, &q);
					// compute likelihood
					prod_transpose1(&q, &pp->inv_cov, &ws1x2);
					prod(&ws1x2, &q, &ws1x1);
					if( pg ){
						*l += ::exp( - value(&ws1x1, 0, 0) / 2.0) * (pp->K) / (pg->N[i] + 1);
					}
					else {
						*l += ::exp( - value(&ws1x1, 0, 0) / 2.0) * (pp->K);
					}
				} // ---> for

				// normalization
				*l /= PlaneNum;
				return 0;
			} // <--- operate
		}


		/**
		 * @brief optimization iterate
		 * @param[in]  x : laser scanner reflection point x
		 * @param[in]  y : laser scanner reflection point y
		 * @param[in]  c : coordinate convert matrix
		 * @param[in]  m : likelihood map
		 * @param[out] g : gradient   ( derivation of f(x,y) because of minimization problem)
		 * @param[out] l : likelihood ( f(x,y) )
		 * @return    0 :
		 * @details this function compute following value for newton's method:
		 * likelihood, gradient, hessian
		 */
		inline
		int gradient( map_t *m, double x, double y, matrix::fixed<4,4> *c, matrix::fixed<3,1> *g, double *l ) {
			matrix::fixed<4,1> X;				// reflection point on global coordinate
			matrix::fixed<PosDim,3> J;			// jacobi
			matrix::fixed<PosDim,1> q_quad_33;

			{ // ---> initialize
				matrix::set_zero(g);
				if(l) *l = 0;
			} // <--- initialize


			{ // ---> compute reflection points on global coordinate
				matrix::fixed<4,1> xx;				// reflection point on global coordinate
				xx[0][0] = x;
				xx[1][0] = y;
				xx[2][0] = 0;
				xx[3][0] = 1;
				matrix::prod(c, &xx, &X);
			} // <--- compute reflection points on global coordinate

			{ // ---> jacobi matrix
				set_zero(&J);
				J[PosX][0] = 1;
				J[PosX][2] = - x * (*c)[1][0] - y * (*c)[0][0];
				J[PosY][1] = 1;
				J[PosY][2] =   x * (*c)[0][0] - y * (*c)[1][0];
			} // <--- jacobi matrix

			{ // ---> q quad_33
				q_quad_33[PosX][0] = - x * (*c)[0][0] + y * (*c)[1][0];
				q_quad_33[PosY][0] = - x * (*c)[1][0] - y * (*c)[0][0];
			} // <--- q quad_33;

			// ---> scanning loop of map plane
			for( size_t mi = 0; mi < PlaneNum; mi++){
				pixel_t *px;
				matrix::fixed<PosDim,1> q;
				matrix::fixed<1,PosDim> qT_invS;		// q^t * Sigma^-1
				matrix::fixed<1,3> qT_invS_J;			// q^t * Sigma^-1 * J
				double chi_sq_2;						// q^T * Sigma^-1 * q / 2 / 2value
				double lkh = 0;
				matrix::fixed<1,3> gg;					// gradient
				matrix::fixed<1,1> ws1x1;				// work space

				{ // ---> get pixel data and compute q(difference from mean)
					matrix::fixed<2,1> ws2x1;
					long pr, pc;
					// get index of pixel
					if( m->plane[mi].pindex( X[PosX][0], X[PosY][0], &pr, &pc ) < 0)
						continue; // no data
					px = m->plane[mi].pointer( pr, pc );
					// zero weight
					if(px->K <= 0.0)	continue;
					// get pixel core position
					m->plane[mi].pget_pos_core(pr, pc, pointer(&ws2x1, PosX, 0), pointer(&ws2x1, PosY, 0));
					// compute sensor reading position on a focus pixel
					sub(&X, &ws2x1, &ws2x1);
					// compute difference from mean
					sub(&ws2x1, &px->mean, &q);
				} // <--- get pixel data and compute q(difference from mean)


				{ // ---> likelihood
					// q^T * Sigma^-1
					prod_transpose1(&q, &px->inv_cov, &qT_invS);
					// chi-square / 2.0
					prod(&qT_invS, &q, &ws1x1);
					chi_sq_2 = ws1x1[0][0] / 2.0;
					// likelihood exp( -q^T * Sigma^-1 * q / 2 )
					lkh = ::exp( -chi_sq_2 ) * px->K;
					if( l ) (*l) += lkh;
				} // <--- likelihood

				{ // ---> gradient
					// q^T * Sigma^-1 * q' * exp( - q^T * Sigma^-1 * q / 2)
					prod(&qT_invS, &J, &qT_invS_J);
					// gradient
					matrix::scalar_prod(&qT_invS_J, lkh, &gg);
					matrix::sub_transpose2(g, &gg, g);
				} // <--- gradient
			}
			return 0;
		}

		/**
		 * @ingroup GNDPSM
		 * @brief counting map file read
		 * @param[out] c : counting map
		 * @param[in] d : directory path
		 * @param[in] f : file name template
		 * @param[in] e : extention
		 */
		inline
		int read_counting_map(cmap_t *c,  const char* d, const char* f, const char* e)
		{
			gnd_assert(!c, -1, "invalid null argument");
			gnd_assert(!d, -1, "invalid null argument");
			gnd_assert(!f, -1, "invalid null argument");
			gnd_assert(!e, -1, "invalid null argument");

			LogDebugf("Begin - read_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
			LogIndent();

			{ // ---> operation
				char path[1024];
				// ---> map plane data scanning loop
				for( size_t i = 0; i < PlaneNum; i++){
					::sprintf(path, CMapFileNameFormat, d, f, i, e);
					LogDebugf("file #%d path \"%s\"\n", i, path);
					if( c->plane[i].fread(path) < 0 ) {
						LogUnindent();
						LogDebugf("Fail  - read_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
						gnd_exit(-1, "fail to file-open");
					}
				} // <--- map plane data scanning loop
			} // <--- operation
			LogUnindent();
			LogDebugf("End  - read_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
			return 0;
		}

		/**
		 * @ingroup GNDPSM
		 * @brief counting map file out
		 * @param[in] c : counting map
		 * @param[in] d : directory path
		 * @param[in] f : file name template
		 * @param[in] e : extention
		 */
		inline
		int write_counting_map(cmap_t *c,  const char* d, const char* f, const char* e)
		{
			gnd_assert(!c, -1, "invalid null argument");
			gnd_assert(!d, -1, "invalid null argument");
			gnd_assert(!f, -1, "invalid null argument");
			gnd_assert(!e, -1, "invalid null argument");

			LogDebugf("Begin - write_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
			LogIndent();

			{ // ---> operation
				char path[1024];
				// ---> map plane data scanning loop
				for( size_t i = 0; i < PlaneNum; i++){
					::sprintf(path, CMapFileNameFormat, d, f, i, e);
					LogVerbosef("file #%d path \"%s\"\n", i, path);
					if( c->plane[i].fwrite(path) < 0 ) {
						LogUnindent();
						LogDebugf("Fail - write_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
						gnd_exit(-1, "fail to file-open");
					}
				} // <--- map plane data scanning loop
			} // <--- operation
			LogUnindent();
			LogDebugf("End  - write_counting_map(%p, %p, %lf, %lf, %lf)\n", c, d, f, e);
			return 0;
		}


		/**
		 * @brief build bitmap data (gray)
		 * @param[out] bmp : gray scale bit map
		 * @param[in] map : map data
		 * @param[in] ps : pixel size
		 * @param[in] sr : smoothing parameter ( sensor range[m])
		 * @param[in] cp : contranst parameter
		 */
		inline
		int build_bmp(bmp8_t *bmp, map_t *map, double ps, double sr, double cp) {
			return build_bmp8(bmp, map, ps, sr, cp);
		}

		/**
		 * @brief build bitmap data (gray)
		 * @param[out] bmp : gray scale bit map
		 * @param[in] map : map data
		 * @param[in] ps : pixel size
		 * @param[in] sr : smoothing parameter ( sensor range[m])
		 * @param[in] cp : contranst parameter
		 */
		inline
		int build_bmp(bmp32_t *bmp, map_t *map, double ps, double sr, double cp) {
			return build_bmp32(bmp, map, ps, sr, cp);
		}


		/**
		 * @brief build bitmap data (gray)
		 * @param[out] bmp : gray scale bit map
		 * @param[in] map  : map data
		 * @param[in] ps   : pixel size
		 * @param[in] sr   : sensor range (for smoothing)
		 * @param[in] cp   : contrast parameter
		 */
		inline
		int build_bmp8(bmp8_t *bmp, map_t *map, double ps, double sr, double cp)
		{
			gridmap::gridplane<double> ws;	// workspace

			gnd_assert(!bmp, -1, "invalid null argument");
			gnd_assert(!map, -1, "invalid null argument");
			gnd_assert(ps <= 0, -1, "invalid argument. pixel size must be greater than 0.");

			{ // ---> initialize
				// allocate bmp data
				if(bmp->is_allocate())	bmp->deallocate();
				bmp->pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
				bmp->pset_origin(map->plane[3].xlower(), map->plane[3].ylower());

				ws.pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
				ws.pset_origin(map->plane[3].xlower(), map->plane[3].ylower());
			} // <--- initialize

			{ // ---> operation
				double lkh = 0;		// likelihood
				double x, y;		// bitmap pixel core position
				double mu;
				double sigma;
				unsigned long cnt = 0;

				LogVerbose("compute likelihood");

				{ // ---> compute likelihood and average
					mu = 0;

					for( unsigned long r = 0; r < ws.row(); r++){
						for( unsigned long c = 0; c < ws.column(); c++){

							// get pixel core position
							ws.pget_pos_core(r, c, &x, &y);

							// compute likelihood
							if( sr > 0){
								pgain_t pg;
								position_gain(map, x, y, sr, &pg);
								likelihood(map, x, y, &pg, &lkh);
							}
							else {
								likelihood(map, x, y, &lkh);
							}

							ws.set(r, c, &lkh);

							if( lkh != 0 ) {
								mu += lkh;
								cnt++;
							}
						}
					}
					if( cnt == 0 ) return -1;
					mu /= cnt;
				} // ---> compute likelihood and average

				{ // ---> compute likelihood and average
					sigma = 0;
					for( unsigned long r = 0; r < ws.row(); r++){
						for( unsigned long c = 0; c < ws.column(); c++){
							ws.get(r, c, &lkh);
							if( lkh == 0 ) continue;
							sigma += gnd_square(lkh - mu);
						}
					}
					sigma /= cnt;
					sigma = ::sqrt(sigma);
				} // ---> compute likelihood and average

				{ // ---> set bitmap
					unsigned char bpv;	// bitmap pixel value

					double min = mu - (cp * sigma);
					double max = mu + (cp * sigma);

					min = min < 0 ? 0 : min;

					for( unsigned long r = 0; r < bmp->row(); r++){
						// ---> grid map scanning loop column
						for( unsigned long c = 0; c < bmp->column(); c++){
							ws.get(r, c, &lkh);

							// normalize for bitmap
							if( lkh <= min ){
								bpv = 0;
							}
							else {
								lkh = (lkh - min) / (max - min);
								lkh *= 0xff;
								bpv = lkh > 0xff ? 0xff : (unsigned char)lkh;
							}

							bmp->set( r, c, &bpv);
						} // <--- grid map scanning loop column
					} // <--- grid map scanning loop
				} // <--- set bitmap

			} // <--- operation

			{ // ---> finalize
				ws.deallocate();
			} // <--- finalize

			return 0;
		}

		/**
		 * @brief build bitmap data (gray)
		 * @param[out] bmp : gray scale bit map
		 * @param[in] map  : map data
		 * @param[in] ps   : pixel size
		 * @param[in] sr   : sensor range (for smoothing)
		 * @param[in] cp   : contrast parameter
		 */
		inline
		int build_bmp32(bmp32_t *bmp, map_t *map, double ps, double sr, double cp)
		{
			gridmap::gridplane<double> ws;	// workspace

			gnd_assert(!bmp, -1, "invalid null argument");
			gnd_assert(!map, -1, "invalid null argument");
			gnd_assert(ps <= 0, -1, "invalid argument. pixel size must be greater than 0.");


			{ // ---> initialize
				// allocate bmp data
				if(bmp->is_allocate())	bmp->deallocate();
				bmp->pallocate(map->plane[3].xupper() - map->plane[0].xlower(), map->plane[3].yupper() - map->plane[0].ylower(), ps, ps);
				bmp->pset_origin(map->plane[0].xlower(), map->plane[0].ylower());

				ws.pallocate(map->plane[3].xupper() - map->plane[0].xlower(), map->plane[3].yupper() - map->plane[0].ylower(), ps, ps);
				ws.pset_origin(map->plane[0].xlower(), map->plane[0].ylower());
			} // <--- initialize

			{ // ---> operation
				double lkh = 0;		// likelihood
				double x, y;		// bitmap pixel core position
				double mu;
				double sigma;
				double lkh_max;
				unsigned long cnt = 0;

				LogVerbose("compute likelihood");

				{ // ---> compute likelihood and average
					mu = 0;
					lkh_max = 0;

					for( unsigned long r = 0; r < ws.row(); r++){
						for( unsigned long c = 0; c < ws.column(); c++){

							// get pixel core position
							ws.pget_pos_core(r, c, &x, &y);

							// compute likelihood
							if( sr > 0){
								pgain_t pg;
								position_gain(map, x, y, sr, &pg);
								likelihood(map, x, y, &pg, &lkh);
							}
							else {
								likelihood(map, x, y, &lkh);
							}

							ws.set(r, c, &lkh);

							if( lkh != 0 ) {
								mu += lkh;
								cnt++;
							}
							lkh_max = lkh_max < lkh ? lkh : lkh_max;
						}
					}
					if( cnt == 0 ) return -1;
					mu /= cnt;
				} // ---> compute likelihood and average

				{ // ---> compute likelihood and average
					sigma = 0;
					for( unsigned long r = 0; r < ws.row(); r++){
						for( unsigned long c = 0; c < ws.column(); c++){
							ws.get(r, c, &lkh);
							if( lkh == 0 ) continue;
							sigma += gnd_square(lkh - mu);
						}
					}
					sigma /= cnt;
					sigma = ::sqrt(sigma);
				} // ---> compute likelihood and average

				{ // ---> set bitmap
					unsigned int bpv;	// bitmap pixel value

					double min = mu - (cp * sigma);
					double max = mu + (cp * sigma);

					min = min < 0 ? 0 : min;
					max = max < lkh_max ? max : lkh_max;

					for( unsigned long r = 0; r < bmp->row(); r++){
						// ---> grid map scanning loop column
						for( unsigned long c = 0; c < bmp->column(); c++){
							ws.get(r, c, &lkh);

							// normalize for bitmap
							if( lkh <= min ){
								bpv = 0;
							}
							else {
								lkh = (lkh - min) / (max - min);
								lkh *= 0x8000;
								bpv = lkh > 0xffff ? 0xffff : (unsigned char)lkh;
							}

							bmp->set( r, c, &bpv);
						} // <--- grid map scanning loop column
					} // <--- grid map scanning loop
				} // <--- set bitmap

			} // <--- operation

			{ // ---> finalize
				ws.deallocate();
			} // <--- finalize

			return 0;
		}
	}
};
// <--- function definition



// ---> class definition
namespace gnd {
	namespace opsm {

		/**
		 * @ingroup GNDPSM
		 * @brief probabilistic scan matching optimizer
		 */
		class optimize_basic {
		public:
			/// @brief position type
			typedef matrix::fixed<3,1> 			pos_t;

			// ---> declaration
		public:
			/// map type
			typedef opsm::map_t					map_t;
			/// map type pointer
			typedef map_t*						map_pt;
			/// pixel type
			typedef opsm::pixel_t 				pixel_t;
			/// pixel type pointer
			typedef pixel_t*					pixel_pt;
			/// reflection point
			typedef matrix::fixed<2,1> 			point_t;
			/// reflection points
			typedef queue< matrix::fixed<2,1> > points_t;
			// <--- declaration


			// ---> constructor, destructor
		public:
			optimize_basic();
			optimize_basic(map_pt m);
			~optimize_basic();
			// <--- constructor, destructor


			// ---> map
		protected:
			map_pt _map;	///< @brief map reference
		public:
			virtual int set_map(map_pt m);
			virtual int release_map();
			// <--- map

			// ---> reflection point
		protected:
			/// @brief laser scanner reflection point
			points_t _points;
		public:
			virtual int set_scan_point(double x, double y);
			virtual int nscan_point() const;
			// <--- reflection point


			// ---> starting vlaue of optimization
		public:
			/**
			 * @brief create initial paramete
			 */
			virtual int initial_parameter_create(void** p) = 0;
			/**
			 * @brief set starting value
			 */
			virtual int initial_parameter_set_position(void* p, double x, double y, double theta) = 0;
			/**
			 * @brief delete starting value
			 */
			virtual int initial_parameter_delete(void** p) = 0;
			// <--- starting value of optimization


			// ---> optimization
		public:
			/**
			 * @brief begin
			 * @note clear all (scan data, previous optimization result) and set initial parameter
			 * @param[in] v : initial parameter
			 * @return    0 :
			 */
			virtual int begin(void *v) = 0;
			/**
			 * @brief iterate
			 * @param[out] d : delta
			 * @param[out] p : pos
			 * @param[out] l : likelihood
			 * @return    0 :
			 */
			virtual int iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l) = 0;
			// <--- optimization


			// ---> converge criteria
		public:
			/// @brief default convergenct threshold (distance)
			static const double DefConvergeDist = gnd_m2dist(0.01);
			/// @brief default convergenct threshold (orient)
			static const double DefConvergeAngle = gnd_deg2ang(0.5);
			/// @brief variables ofconvergence test
			struct converge_var {
				double sqdist;	///< distance threshold
				double orient;	///< orient threshold
				pos_t delta;	///< delta
				converge_var();
			};
		protected:
			/// converge parameter
			struct converge_var _converge;
		public:
			int set_converge_threshold(double d, double a);
			int converge_test();
			static int converge_test(double sqd, double o, double sqdt, double ot);
			// <--- converge criteria
		};

		/**
		 * @brief constructor
		 */
		inline
		optimize_basic::optimize_basic()
		{
			_map = 0;
		}

		/**
		 * @brief constructor
		 * @param[in] m: map
		 */
		inline
		optimize_basic::optimize_basic(map_pt m) {
			set_map(m);
		}

		/**
		 * @brief destructor
		 */
		inline
		optimize_basic::~optimize_basic()
		{
			release_map();
		}


		/**
		 * @brief set map
		 * @param[in] m : map pointer
		 * @return  0 : map is null
		 * 		   >0 : map is exist
		 */
		inline
		int optimize_basic::set_map(map_pt m)
		{
			if(m)	_map = m;
			return (_map != 0);
		}

		/**
		 * @brief release map
		 */
		inline
		int optimize_basic::release_map()
		{
			_map = 0;
			return 0;
		}


		/**
		 * @brief set scan point
		 * @param[in]  x : reflect point x
		 * @param[in]  y : reflect point y
		 * @return    number of points
		 */
		inline
		int optimize_basic::set_scan_point(double x, double y) {
			point_t p;
			p[0][0] = x;
			p[1][0] = y;
			return _points.push_back(&p);
		}

		/**
		 * @brief get number of scan point
		 * @return    number of points
		 */
		inline
		int optimize_basic::nscan_point() const{
			return _points.size();
		}


		/**
		 * @brief constructor of optimizer_basic::converge_var
		 */
		optimize_basic::converge_var::converge_var() : sqdist( gnd_square( DefConvergeDist )), orient(DefConvergeAngle)
		{
		}

		/**
		 * @brief set convergence criteria threshold
		 * @param[in] d: distance
		 * @param[in] o: orient
		 */
		int optimize_basic::set_converge_threshold(double d, double o) {
			_converge.sqdist = gnd_square(d);
			_converge.orient = ::fabs(o);
			return 0;
		}

		/**
		 * @brief converge test
		 */
		int optimize_basic::converge_test() {
			return converge_test( gnd_square( _converge.delta[0][0] ) + gnd_square( _converge.delta[1][0] ) , _converge.delta[2][0],
					_converge.sqdist, _converge.orient);
		}


		/**
		 * @brief converge test
		 * @param[in]  sqd : square distance
		 * @param[in]    o : orient
		 * @param[in] sqdt : square distance threshold
		 * @param[in]   ot : orient threshold
		 * @return !0 : converge
		 * @return  0 : not converge
		 */
		int optimize_basic::converge_test(double sqd, double o, double sqdt, double ot) {
			return sqd < sqdt && ::fabs(o) < ot;
		}

	}
}
// optimizer_basic
// ---> class definition



// ---> class definition
// optimizer_newton
namespace gnd {
	namespace opsm {

		/**
		 * @ingroup GNDPSM
		 * @brief probabilistic scan matching optimizer
		 */
		class optimize_newton
				: public optimize_basic {

			// ---> type declaration
				public:
			/// map type
			typedef opsm::map_t					map_t;
			/// map type pointer
			typedef map_t*						map_pt;
			/// pixel type
			typedef opsm::pixel_t 				pixel_t;
			/// pixel type pointer
			typedef pixel_t*					pixel_pt;

			/// input variance
			typedef matrix::fixed<3,1>			initial_parameter;
			/// coordinate matrix type
			typedef matrix::fixed<4,4>			coordm_t;

			/// @brief variables
			struct variables {
				matrix::fixed<3,1> pos;		///< position
				double likelihood;			///< likelihood
			};
			/// variables
			typedef struct variables variables;
			// <--- type declaration


			// ---> constructor, destructor
				public:
			optimize_newton();
			optimize_newton(map_pt m);
			~optimize_newton();
			// <--- constructor, destructor

			// ---> variance
				protected:
			/// @brief variables
			variables	_var;
			/// @brief coordinate convert matrix
			coordm_t	_coordm;
			// <--- variance


			// ---> starting value of optimization
				public:
			virtual int initial_parameter_create(void** p);
			virtual int initial_parameter_delete(void** p);
			virtual int initial_parameter_set_position(void* p, double x, double y, double theta);
			// <--- starting value of optimization



			// ---> optimization
				public:
			virtual int begin(void *v);
			virtual int iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l) ;
			// <--- optimization

			static int _newton_method_variables_( double x, double y, matrix::fixed<4,4> *c, map_pt m,  double *l, matrix::fixed<3,1> *g, matrix::fixed<3,3> *h );
		};


		/**
		 * @brief constructor
		 */
		inline
		optimize_newton::optimize_newton()
		{
		}

		/**
		 * @brief constructor
		 */
		inline
		optimize_newton::optimize_newton(map_pt m) : optimize_basic(m)
		{
		}

		/**
		 * @brief destructor
		 */
		inline
		optimize_newton::~optimize_newton()
		{
		}


		/**
		 * @brief create starting value
		 * @param[in,out] p : starting value buffer pointer
		 */
		inline
		int optimize_newton::initial_parameter_create(void** p) {
			*p = static_cast<void*>( new initial_parameter );
			return 0;
		}


		/**
		 * @brief delete starting value
		 * @param[in,out] p : starting value
		 */
		inline
		int optimize_newton::initial_parameter_delete(void** p) {
			initial_parameter* pp = static_cast<initial_parameter*>( *p );
			delete pp;
			*p = 0;
			return 0;
		}


		/**
		 * @brief set starting value
		 * @param[in]     p : starting value
		 * @param[in]     x : robot position x
		 * @param[in]     y : robot position y
		 * @param[in] theta : robot position theta
		 */
		inline
		int optimize_newton::initial_parameter_set_position(void* p, double x, double y, double theta) {
			initial_parameter *pp = static_cast<initial_parameter*>(p);
			(*pp)[0][0] = x;
			(*pp)[1][0] = y;
			(*pp)[2][0] = theta;
			return 0;
		}

		/**
		 * @brief input starting value
		 * @param[in] v : starting value
		 */
		inline
		int optimize_newton::begin(void *v) {
			gnd_assert(!v, -1, "invalid null argument");
			initial_parameter *p = static_cast<initial_parameter*>(v);

			// set position
			copy(&_var.pos, p);

			matrix::coordinate_converter(&_coordm,
					_var.pos[0][0], _var.pos[1][0], 0,
					::cos(_var.pos[2][0]), ::sin(_var.pos[2][0]), 0,
					 0, 0, 1);

			// set zero
			_var.likelihood = 0;
			_points.clear();
			return 0;
		}

		/**
		 * @brief optimization iterate
		 * @param[out] d : difference
		 * @param[out] p : pos
		 * @param[out] v : variance
		 * @param[out] l : likelihood
		 * @return    0 :
		 */
		inline
		int optimize_newton::iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l) {
			double likelihood = 0;
			uint64_t pi;
			matrix::fixed<3,1> grad;
			matrix::fixed<3,3> hess;

			// ---> scanning loop of reflection points
			for( pi = 0; pi < _points.size(); pi++ ){
				double l;
				matrix::fixed<3,1> g;
				matrix::fixed<3,3> h;

				// compute likelihood, gradient, hessian
				_newton_method_variables_( _points[pi][0][0], _points[pi][1][0], &_coordm, _map, &l, &g, &h );
				// summation
				likelihood += l;
				add(&grad, &g, &grad);
				add(&hess, &h, &hess);
			} // <--- scanning loop of reflection points


			{ // ---> optimization
				int ret;
				matrix::fixed<3,1> delta;

				// modified newton's method for unconstrained minimizer
				if( (ret = optimize::newtons_method_unconstrainted(&grad, &hess, &delta)) < 0)
					return ret;

				// store delta for convergence test
				copy(&_converge.delta, &delta);
				// position optimization
				add(&delta, &_var.pos, &_var.pos);

				// set coordinate convert matrix
				matrix::coordinate_converter(&_coordm,
						_var.pos[0][0], _var.pos[1][0], 0,
						::cos(_var.pos[2][0]), ::sin(_var.pos[2][0]), 0,
						 0, 0, 1);

				LogDebugf("     : newton - delta = (%lf, %lf, %lf):\n", delta[0][0], delta[1][0], delta[2][0]);
				// set output data
				if( d ) {
					copy(d, &delta);
				}
				if( p ) {
					copy(p, &_var.pos);
				}
				if( l ) {
					*l = _var.likelihood;
				}
			} // <--- set output

			{ // ---> set zero
				_var.likelihood = 0;
			} // <--- set zero

			return 0;
		}



		/**
		 * @brief optimization iterate
		 * @param[in]  x : laser scanner reflection point x
		 * @param[in]  y : laser scanner reflection point y
		 * @param[in]  c : coordinate convert matrix
		 * @param[in]  m : likelihood map
		 * @param[out] l : likelihood ( f(x,y) )
		 * @param[out] g : gradient   ( derivation of -f(x,y) because of minimization problem)
		 * @param[out] h : hessian    ( quadratic of -f(x,y) because of minimization problem)
		 * @return    0 :
		 * @details this function compute following value for newton's method:
		 * likelihood, gradient, hessian
		 */
		inline
		int optimize_newton::_newton_method_variables_( double x, double y, matrix::fixed<4,4> *c, map_pt m,  double *l, matrix::fixed<3,1> *g, matrix::fixed<3,3> *h ) {
			matrix::fixed<4,1> X;					// reflection point on global coordinate
			matrix::fixed<PosDim,3> J;			// jacobi
			matrix::fixed<PosDim,1> q_quad_33;

			{ // ---> compute reflection points on global coordinate
				matrix::fixed<4,1> xx;
				xx[0][0] = x;
				xx[1][0] = y;
				xx[2][0] = 0;
				xx[3][0] = 1;
				prod( c, &xx, &X );
			} // <--- compute reflection points on global coordinate

			{ // ---> jacobi matrix
				set_zero(&J);
				J[PosX][0] = 1;
				J[PosX][2] = - x * (*c)[1][0] - y * (*c)[0][0];
				J[PosY][1] = 1;
				J[PosY][2] =   x * (*c)[0][0] - y * (*c)[1][0];
			} // <--- jacobi matrix

			{ // ---> q quad_33
				q_quad_33[PosX][0] = - x * (*c)[0][0] + y * (*c)[1][0];
				q_quad_33[PosY][0] = - x * (*c)[1][0] - y * (*c)[0][0];
			} // <--- q quad_33;

			{
				*l = 0;
				matrix::set_zero(g);
				matrix::set_zero(h);
			}

			// ---> scanning loop of map plane
			for( size_t mi = 0; mi < PlaneNum; mi++){
				pixel_t *px;
				matrix::fixed<PosDim,1> q;
				matrix::fixed<1,PosDim> qT_invS;		// q^t * Sigma^-1
				matrix::fixed<1,3> qT_invS_J;			// q^t * Sigma^-1 * J
				double chi_sq_2;						// q^T * Sigma^-1 * q / 2 / 2value
				double lkh = 0;
				matrix::fixed<1,3> gg;					// gradient
				matrix::fixed<3,3> hh;					// gradient
				matrix::fixed<1,1> ws1x1;				// work space

				{ // ---> get pixel data and compute q(difference from mean)
					matrix::fixed<2,1> ws2x1;
					long pr, pc;
					// get index of pixel
					if( m->plane[mi].pindex( X[PosX][0], X[PosY][0], &pr, &pc ) < 0)
						continue; // no data
					px = m->plane[mi].pointer( pr, pc );
					// zero weight
					if(px->K <= 0.0)	continue;
					// get pixel core position
					m->plane[mi].pget_pos_core(pr, pc, pointer(&ws2x1, PosX, 0), pointer(&ws2x1, PosY, 0));
					// compute sensor reading position on a focus pixel
					sub(&X, &ws2x1, &ws2x1);
					// compute difference from mean
					sub(&ws2x1, &px->mean, &q);
				} // <--- get pixed data and compute q(difference from mean)


				{ // ---> likelihood
					// q^T * Sigma^-1
					prod_transpose1(&q, &px->inv_cov, &qT_invS);
					// chi-square / 2.0
					prod(&qT_invS, &q, &ws1x1);
					chi_sq_2 = ws1x1[0][0] / 2.0;
					// likelihood exp( -q^T * Sigma^-1 * q / 2 )
					lkh = ::exp( -chi_sq_2 ) * px->K;
					*l += lkh;
				} // <--- likelihood

				{ // ---> gradient
					// q^T * Sigma^-1 * q' * exp( - q^T * Sigma^-1 * q / 2)
					prod(&qT_invS, &J, &qT_invS_J);
					// gradient
					scalar_prod(&qT_invS_J, lkh, &gg);
					matrix::add(g, &gg, g);
				} // <--- gradient

				{ // ---> hessian
					matrix::fixed<3,3> ws3x3;
					matrix::fixed<3,2> ws3x2;

					// (-q*Sigma^-1*q')^T * (-q*Sigma^-1*q')
					prod_transpose1( &qT_invS_J, &qT_invS_J, &hh );

					// -q*Sigma^-1*q''
					prod( &qT_invS, &q_quad_33, &ws1x1 );
					hh[2][2] -= ws1x1[0][0];

					// -q'*Sigma^-1*q'
					prod_transpose1( &J, &px->inv_cov, &ws3x2 );
					prod( &ws3x2, &J, &ws3x3 );
					sub( &hh, &ws3x3, &hh );
					// hessian
					scalar_prod( &hh, -(lkh), &hh );

					optimize::_model_hessian_(&hh);
					matrix::add(h, &hh, h);
				} // <--- hessian
			}
			return 0;
		}

	}

};
// <--- class function definition











// ---> class definition
namespace gnd {
	namespace opsm {

		/**
		 * @brief probabilistic scan matching optimizer (monte-calro)
		 */
		class optimize_monte_calro_method : public optimize_basic {
			// ---> type declaration
		public:
			/// map type
			typedef opsm::map_t		map_t;
			/// map type pointer
			typedef map_t*			map_pt;
			/// pixel type
			typedef opsm::pixel_t	pixel_t;
			/// pixel type pointer
			typedef pixel_t*		pixel_pt;
			// <--- type declaration

			// ---> constructor, destructor
		public:
			optimize_monte_calro_method();
			optimize_monte_calro_method(map_pt m);
			~optimize_monte_calro_method();
			// <--- constructor, destructor

			// ---> particle
		protected:
			/// @brief particle
			struct particle {
				vector::fixed_column<3> pos;	///< position
				matrix::fixed<4,4> coordm;		///< coordinate convert matrix
				double likelihood;				///< likelihood
			};
			/// @brief particles
			queue< struct particle > particles;
			/// @brief particles
			queue< struct particle > _ws_resmpl;
			// <--- particle

			// ---> starting value
		public:
			/// @brief starting value of optimization
			typedef struct initial_parameter {
				vector::fixed_column<3> pos;		///<! position (column vector)
				matrix::fixed<3,3>	var_ini;		///<! initial variance
				matrix::fixed<3,3>	var_rsmp;		///<! variance for resampling random
				uint32_t			n;				///<! number of particle
				double				alpha;			///<! parameter for seeking around max likelihood particle
				initial_parameter();
			} initial_parameter;

		protected:
			/// @brief optimize
			initial_parameter _v;
			// <--- starting value


			// ---> starting value of optimization
		public:
			virtual int initial_parameter_create(void** p);
			virtual int initial_parameter_delete(void** p);
			virtual int initial_parameter_set_position(void* p, double x, double y, double theta);
			// <--- starting of optimization


			// ---> optimization
		public:
			virtual int begin(void *v);
			virtual int iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l);
			// <--- optimization
		protected:
			int init_particle( initial_parameter *v );
		};


		/**
		 * @brief constructor
		 */
		inline
		optimize_monte_calro_method::optimize_monte_calro_method() {
			return;
		}

		/**
		 * @brief constructor
		 * @param[in] m : map
		 */
		inline
		optimize_monte_calro_method::optimize_monte_calro_method(map_pt m)
		: optimize_basic(m) {
			return;
		}

		/**
		 * @brief destructor
		 */
		inline
		optimize_monte_calro_method::~optimize_monte_calro_method(){
		}


		/**
		 * @brief create starting value
		 * @param[in,out] p : starting value buffer pointer
		 */
		inline
		int optimize_monte_calro_method::initial_parameter_create(void** p) {
			*p = static_cast<void*>( new initial_parameter );
			return 0;
		}


		/**
		 * @brief delete starting value
		 * @param[in,out] p : starting value
		 */
		inline
		int optimize_monte_calro_method::initial_parameter_delete(void** p) {
			initial_parameter* pp = static_cast<initial_parameter*>( *p );
			delete pp;
			*p = 0;
			return 0;
		}


		/**
		 * @brief set starting value
		 * @param[in]     p : starting value
		 * @param[in]     x : robot position x
		 * @param[in]     y : robot position y
		 * @param[in] theta : robot position theta
		 */
		inline
		int optimize_monte_calro_method::initial_parameter_set_position(void* p, double x, double y, double theta) {
			initial_parameter *pp = static_cast<initial_parameter*>(p);
			pp->pos[0] = x;
			pp->pos[1] = y;
			pp->pos[2] = theta;
			return 0;
		}



		/**
		 * @brief input starting value
		 * @param[in] v : input variance
		 * @return 0
		 */
		inline
		int optimize_monte_calro_method::begin(void *v) {
			LogDebug("Begin - mcl begin\n");
			LogIndent();

			::memcpy(&_v, v, sizeof(_v));

			// ckolesky
			linalg::cholesky_decomposition(&_v.var_rsmp, 3);

			// reflesh particle
			particles.clear();
			init_particle(&_v);

			// clear laser scanner reflection point
			_points.clear();

			LogUnindent();
			LogDebug("End   - mcl begin\n");

			return 0;
		}


		/**
		 * @brief create particle
		 * @param[in] v : input variance
		 * @return 0
		 */
		inline
		int optimize_monte_calro_method::init_particle( initial_parameter *v ) {
			matrix::fixed<3,1> rnd;
			matrix::fixed<3,3> L;
			particle p;

			// cholesky decomposition
			matrix::copy(&L, &v->var_ini);
			linalg::cholesky_decomposition(&L, 3);

			// set 0
			p.likelihood = 0;

			// set position
			matrix::copy(&p.pos, &v->pos);

			// get coordinate convert matrix
			matrix::coordinate_converter(&p.coordm,
					p.pos[0], p.pos[1], 0,
					::cos(p.pos[2]), ::sin(p.pos[2]), 0,
					 0, 0, 1);
			particles.push_back(&p);

			while ( particles.size() < v->n ){
				// create particle according to gaussian
				rnd[0][0] = random_gaussian(1.0);
				rnd[1][0] = random_gaussian(1.0);
				rnd[2][0] = random_gaussian(1.0);
				matrix::prod(&L, &rnd, &p.pos);
				// add average
				matrix::add(&p.pos, &v->pos, &p.pos);

				// get coordinate convert matrix
				matrix::coordinate_converter(&p.coordm,
						p.pos[0], p.pos[1], 0,
						::cos(p.pos[2]), ::sin(p.pos[2]), 0,
						 0, 0, 1);

				particles.push_back(&p);
			}

			return 0;
		}



		/**
		 * @brief optimization iterate
		 * @param[out] d : difference
		 * @param[out] p : pos
		 * @param[out] v : variance ()
		 * @param[out] l : likelihood
		 * @return    0 :
		 */
		inline
		int optimize_monte_calro_method::iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l){
			gnd_error(particles.size() == 0, -1, "no data" );
			gnd_error(_points.size() == 0, -1, "no scan data" );

			LogDebug("Begin - mcl iterate\n");
			LogIndent();

			{ // ---> operate
				uint32_t i, j;
				double lk = 0;
				double sum;
				double max;
				size_t imax;
				vector::fixed_column<3>	delta;	// delta
				vector::fixed_column<3>	ws3x1;	// workspace
				matrix::fixed<3,3>		ws3x3;	// workspace

				matrix::set_zero(&delta);
				sum = 0;
				for( j = 0; j < _points.size(); j++ ){
					matrix::fixed<4,1> p;

					p[0][0] = _points[j][0][0];
					p[1][0] = _points[j][1][0];
					p[2][0] = 0;
					p[3][0] = 1;

					for(size_t i = 0; i < (unsigned)particles.size(); i++){
						matrix::fixed<4,1> x;

						matrix::prod( &particles[i].coordm,  &p, &x );

						opsm::likelihood(_map, x[0][0], x[1][0], &lk);
						particles[i].likelihood += lk;
					}
				} // ---> loop for compute likelihood


				// ---> likelihood sum and get max
				max = 0.0;
				imax = 0;
				for( i = 0; i < (unsigned)particles.size(); i++){
					particles[i].likelihood /= _points.size();

					// get max
					if( max < particles[i].likelihood )	{
						max = particles[i].likelihood;
						imax = i;
					}

					// sum
					sum += particles[i].likelihood;

					LogVerbosef("      : particle (%lf, %lf, %lf) lkh = %lf\n",
							particles[i].pos[0], particles[i].pos[1], particles[i].pos[2], particles[i].likelihood);
				} // ---> likelihood sum and get max

				// if sum == zero, error
				gnd_error( sum == 0, -1, "all likelihood is 0" );

				{ // ---> save maximum likelihood particle
					matrix::sub(&particles[imax].pos, &_v.pos, &delta);

					matrix::copy(&_v.pos, &particles[imax].pos);
				} // <--- save maximum likelihood particle

				LogDebugf("      : delta = (%lf, %lf, %lf):\n", delta[0], delta[1], delta[2]);

				// store delta for converge test
				matrix::copy(&_converge.delta, &delta);

				{ // ---> output
					if( p ) { // ---> set position
						matrix::copy(p, &particles[imax].pos);
					} // <--- set position

					if( d ) { // ---> set delta
						matrix::copy(d, &delta);
					} // <--- set delta

					if( l ) {// ---> set likelihood
						*l = particles[imax].likelihood;
					} // <--- set likelihood
				} // <--- output (nearest neighbor)



				{ // ---> resampling
					particle p;

					// clear
					_ws_resmpl.clear();

					// save max likelihood particle
					_ws_resmpl.push_back(&particles[imax]);

					// save max particle
					// and seek around max one
					while(_ws_resmpl.size() < _v.n * _v.alpha )
						_ws_resmpl.push_back(&particles[imax]);

					// ---> select particle considering its likelihood
					while( _ws_resmpl.size() < _v.n ) {
						double lrand = sum * random_uniform();

						// select sample
						for( i = 0; i < particles.size() - 1 && lrand > 0; i++ )
							lrand -= particles[i].likelihood;

						// save
						_ws_resmpl.push_back(&particles[i]);
					} // <--- select particle considering its likelihood


					// cholesky decomposition
					LogVerbosef("L    : %.4lf, %.4lf, %.4lf\n", _v.var_rsmp[0][0], _v.var_rsmp[0][1], _v.var_rsmp[0][2] );
					LogVerbosef("     : %.4lf, %.4lf, %.4lf\n", _v.var_rsmp[1][0], _v.var_rsmp[1][1], _v.var_rsmp[1][2] );
					LogVerbosef("     : %.4lf, %.4lf, %.4lf\n", _v.var_rsmp[2][0], _v.var_rsmp[2][1], _v.var_rsmp[2][2] );

					// clear
					particles.clear();
					p.likelihood = 0;

					// set max
					particles.push_back(&_ws_resmpl[0]);
					// resample
					for( i = particles.size(); i < _ws_resmpl.size(); i++ ) {
						// random
						ws3x1[0] = random_gaussian(1.0);
						ws3x1[1] = random_gaussian(1.0);
						ws3x1[2] = random_gaussian(1.0);

						matrix::prod( &_v.var_rsmp, &ws3x1, &p.pos );
						LogVerbosef("rand : %.4lf, %.4lf, %.4lf\n", p.pos[0], p.pos[1], p.pos[2] );
						matrix::add( &p.pos, &_ws_resmpl[i].pos, &p.pos );

						// get coordinate convert matrix
						matrix::coordinate_converter(&p.coordm,
								p.pos[0], p.pos[1], 0,
								::cos(p.pos[2]), ::sin(p.pos[2]), 0,
								 0, 0, 1);

						// set
						particles.push_back(&p);
					}


				} // <--- resampling
			} // <--- operate

			LogUnindent();
			LogDebug("End  - mcl iterate\n");
			return 0;
		}


		inline
		optimize_monte_calro_method::initial_parameter::initial_parameter(){
			matrix::set_zero(&pos);

			n = 250;
			var_ini[0][0] = gnd_square( gnd_m2dist( 0.1 ) );
			var_ini[1][1] = gnd_square( gnd_m2dist( 0.1 ) );
			var_ini[2][2] = gnd_square( gnd_deg2ang( 10.0 ) );

			var_rsmp[0][0] = gnd_square( gnd_m2dist( 0.005 ) );
			var_rsmp[1][1] = gnd_square( gnd_m2dist( 0.005 ) );
			var_rsmp[2][2] = gnd_square( gnd_deg2ang( 0.25 ) );

			alpha = 0.0;
		}
	}
};
// <--- class definition





// ---> class definition
namespace gnd {
	namespace opsm {

		/**
		 * @brief probabilistic scan matching optimizer (quasi-monte-calro)
		 */
		class optimize_quasi_monte_calro : public optimize_basic {
			// ---> type declaration
		public:
			/// map type
			typedef opsm::map_t		map_t;
			/// map type pointer
			typedef map_t*			map_pt;
			/// pixel type
			typedef opsm::pixel_t	pixel_t;
			/// pixel type pointer
			typedef pixel_t*		pixel_pt;
			// <--- type declaration

			// ---> constructor, destructor
		public:
			optimize_quasi_monte_calro();
			optimize_quasi_monte_calro(map_pt m);
			~optimize_quasi_monte_calro();
			// <--- constructor, destructor

		private:
			/// @brief particle table
			queue< vector::fixed_column<3> > table;
			/// @brief create sphere-table
			int create_particletable(uint32_t n);

			// ---> particle
		protected:
			/// @brief particle
			struct particle {
				vector::fixed_column<3> pos;		///< position
				matrix::fixed<4,4> coordm;	///< coordinate convert matrix
				double likelihood;			///< likelihood
			};
			/// @brief particles
			queue< struct particle > particles;
			// <--- particle

			// ---> starting value
		public:
			/// @brief starting value of optimization
			typedef struct initial_parameter {
				vector::fixed_column<3>	pos;			///<! position
				matrix::fixed<3,3>	var;		///<! variance
				uint32_t			n;				///<! resolution
				initial_parameter();
			} initial_parameter;
		protected:
			/// @brief optimize starting value
			initial_parameter _v;
			// <--- starting value


			// ---> starting value of optimization
		public:
			virtual int initial_parameter_create(void** p);
			virtual int initial_parameter_delete(void** p);
			virtual int initial_parameter_set_position(void* p, double x, double y, double theta);
			// <--- starting of optimization


			// ---> optimization
		public:
			virtual int begin(void *v);
			virtual int iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l);
			// <--- optimization
		protected:
			int create_particle(initial_parameter *v);
		};


		/**
		 * @brief constructor
		 */
		inline
		optimize_quasi_monte_calro::optimize_quasi_monte_calro() {
			return;
		}

		/**
		 * @brief constructor
		 * @param[in] m : map
		 */
		inline
		optimize_quasi_monte_calro::optimize_quasi_monte_calro(map_pt m)
		: optimize_basic(m) {
			return;
		}

		/**
		 * @brief destructor
		 */
		inline
		optimize_quasi_monte_calro::~optimize_quasi_monte_calro(){
		}

		inline
		int optimize_quasi_monte_calro::create_particletable(uint32_t n) {
			uint32_t x, y, t;
			vector::fixed_column<3> tmp;

			LogVerbose("Begin : qmc create_particletable\n");
			LogIndent();

			table.clear();
			matrix::set_zero(&tmp);
			table.push_back(&tmp);

			for(x = 0; x < n * 2 + 3; x++) {
				tmp[0] = ((double)x / (n+1)) - 1;
				for(y = 0; y < n * 2 + 3; y++) {
					tmp[1] = ((double)y / (n+1)) - 1;
					for(t = 0; t < n * 2 + 3; t++) {
						tmp[2] = ((double)t / (n+1)) - 1;
						LogVerbosef("%.04lf %.04lf %.04lf\n", tmp[0], tmp[1], tmp[2]);
						table.push_back(&tmp);
					} // for (t)
				} // for (y)
			} // for (x)

			LogUnindent();
			LogVerbose(" End   : qmc create_particletable\n");
			return 0;
		}



		/**
		 * @brief create starting value
		 * @param[in,out] p : starting value buffer pointer
		 */
		inline
		int optimize_quasi_monte_calro::initial_parameter_create(void** p) {
			*p = static_cast<void*>( new initial_parameter );
			return 0;
		}


		/**
		 * @brief delete starting value
		 * @param[in,out] p : starting value
		 */
		inline
		int optimize_quasi_monte_calro::initial_parameter_delete(void** p) {
			initial_parameter* pp = static_cast<initial_parameter*>( *p );
			delete pp;
			p = 0;
			return 0;
		}


		/**
		 * @brief set starting value
		 * @param[in]     p : starting value
		 * @param[in]     x : robot position x
		 * @param[in]     y : robot position y
		 * @param[in] theta : robot position theta
		 */
		inline
		int optimize_quasi_monte_calro::initial_parameter_set_position(void* p, double x, double y, double theta) {
			initial_parameter *pp = static_cast<initial_parameter*>(p);
			pp->pos[0] = x;
			pp->pos[1] = y;
			pp->pos[2] = theta;
			return 0;
		}

		/**
		 * @brief input starting value
		 * @param[in] v : input variance
		 * @return 0
		 */
		inline
		int optimize_quasi_monte_calro::begin(void *v) {
			initial_parameter *varp = static_cast<initial_parameter*>(v);

			create_particletable(varp->n);

			// reflesh particle
			particles.clear();
			create_particle(varp);

			// clear laser scanner reflection point
			_points.clear();
			::memcpy( &_v, varp, sizeof(_v) );

			return 0;
		}


		/**
		 * @brief create particle
		 * @param[in] v : input variance
		 * @return 0
		 */
		inline
		int optimize_quasi_monte_calro::create_particle(initial_parameter *v) {
			matrix::fixed<3,3> L;
			particle p;
			uint32_t i;

			LogVerbose("Begin : qmc create_particle\n");
			LogIndent();

			// cholesky decomposition
			matrix::copy(&L, &v->var);
			linalg::cholesky_decomposition(&L, 3);
			LogVerbosef("L : %.04lf %.04lf %.04lf\n", L[0][0], L[0][1], L[0][2]);
			LogVerbosef("  : %.04lf %.04lf %.04lf\n", L[1][0], L[1][1], L[1][2]);
			LogVerbosef("  : %.04lf %.04lf %.04lf\n", L[2][0], L[2][1], L[2][2]);

			// set 0
			p.likelihood = 0;
			matrix::set_zero(&p.pos);
			matrix::coordinate_converter(&p.coordm,
					p.pos[0] + v->pos[0], p.pos[1] + v->pos[1], 0,
					::cos(p.pos[2] + v->pos[2]), ::sin(p.pos[2] + v->pos[2]), 0,
					 0, 0, 1);
			particles.push_back(&p);

			// ---> scanning loop for Table
			for( i = 0; i < table.size(); i++ ){
				prod(&L, table + i, &p.pos);
				matrix::coordinate_converter(&p.coordm,
						p.pos[0] + v->pos[0], p.pos[1] + v->pos[1], 0,
						::cos(p.pos[2] + v->pos[2]), ::sin(p.pos[2] + v->pos[2]), 0,
						 0, 0, 1);
				LogVerbosef("%.04lf %.04lf %.04lf\n", p.pos[0], p.pos[1], p.pos[2]);
				particles.push_back(&p);
			} // <--- scanning loop for SphereTable

			LogUnindent();
			LogVerbose(" End   : qmc create_particle\n");
			return 0;
		}



		/**
		 * @brief optimization iterate
		 * @param[out] d : difference
		 * @param[out] p : pos
		 * @param[out] v : variance
		 * @param[out] l : likelihood
		 * @return    0 :
		 */
		inline
		int optimize_quasi_monte_calro::iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l){
			gnd_error(particles.size() == 0, -1, "no data" );

			{ // ---> operate
				double lk = 0;
				double sum;
				matrix::fixed<3,1> delta;
				matrix::fixed<3,1> ws3x1;

				set_zero(&delta);
				sum = 0;
				// ---> loop for compute likelihood
				for(size_t i = 0; i < (unsigned)particles.size(); i++){
					particles[i].likelihood = 0;
				}
				for(size_t j = 0; j < (unsigned)_points.size(); j++ ){
					matrix::fixed<4,1> p;

					p[0][0] = _points[j][0][0];
					p[1][0] = _points[j][1][0];
					p[2][0] = 0;
					p[3][0] = 1;

					for(size_t i = 0; i < (unsigned)particles.size(); i++){
						matrix::fixed<4,1> x;

						matrix::prod( &particles[i].coordm,  &p, &x );

						opsm::likelihood(_map, x[0][0], x[1][0], &lk);
						particles[i].likelihood += lk;
					}
				} // ---> loop for compute likelihood
				for(size_t i = 0; i < (unsigned)particles.size(); i++){
					sum += particles[i].likelihood;
					matrix::scalar_prod( &particles[i].pos, particles[i].likelihood, &ws3x1 );
					matrix::add(&delta, &ws3x1, &delta);
				}

				if(sum == 0)	return -1;
				// weighted average
				matrix::scalar_div(&delta, sum, &delta);

				LogDebugf("     : qmc - delta = (%lf, %lf, %lf):\n", delta[0][0], delta[1][0], delta[2][0]);

				// store delta
				matrix::copy(&_converge.delta, &delta);

				// output delta
				if( d )	matrix::copy(d, &delta);
				// compute
				matrix::add(&delta, &_v.pos, &_v.pos);

				// output position
				if( p ) matrix::copy(p, &_v.pos);


				{ // ---> compute distribution
					matrix::fixed<3,1> ws3x1;
					matrix::fixed<3,3> ws3x3;

					// ---> compute weighted summation
					set_zero(&_v.var);
					for(size_t i = 0; i < (unsigned)particles.size(); i++) {
						matrix::sub(&particles[i].pos, &delta, &ws3x1);
						matrix::prod_transpose2(&ws3x1, &ws3x1, &ws3x3);
						matrix::scalar_prod(&ws3x3, particles[i].likelihood, &ws3x3);
						matrix::add(&_v.var, &ws3x3, &_v.var);
					} // <--- compute weighted summation

					matrix::scalar_div(&_v.var, sum, &_v.var);
				} // <--- compute distribution

				if( l ) {// ---> set likelihood
					*l = particles[0].likelihood;
				} // <--- set likelihood


				{ // ---> refresh particle
					// store starting value
					particles.clear();
					create_particle(&_v);
				} /// <--- refresh particle
			} // <--- operate
			return 0;
		}


		inline
		optimize_quasi_monte_calro::initial_parameter::initial_parameter(){
			n = 1;
			var[0][0] = gnd_square( gnd_m2dist( 0.05 ) );
			var[1][1] = gnd_square( gnd_m2dist( 0.05 ) );
			var[2][2] = gnd_square( gnd_deg2ang( 3 ) );
		}
	}
};
// <--- class definition





// ---> class definition
namespace gnd {
	namespace opsm {

		/**
		 * @brief probabilistic scan matching optimizer (quasi-monte-calro and newton's method)
		 */
		class optimize_hybrid_qmc2newton : public optimize_quasi_monte_calro {

			// ---> type declaration
		public:
			/// map type
			typedef opsm::map_t		map_t;
			/// map type pointer
			typedef map_t*			map_pt;
			/// pixel type
			typedef opsm::pixel_t	pixel_t;
			/// pixel type pointer
			typedef pixel_t*		pixel_pt;
			// <--- type declaration

			// ---> constructor, destructor
		public:
			optimize_hybrid_qmc2newton();
			optimize_hybrid_qmc2newton(map_pt m);
			~optimize_hybrid_qmc2newton();
			// <--- constructor, destructor

			// ---> optimization
		public:
			virtual int iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l);
			// <--- optimization
		};


		/**
		 * @brief constructor
		 */
		inline
		optimize_hybrid_qmc2newton::optimize_hybrid_qmc2newton() {
			return;
		}

		/**
		 * @brief constructor
		 * @param[in] m : map
		 */
		inline
		optimize_hybrid_qmc2newton::optimize_hybrid_qmc2newton(map_pt m)
		: optimize_quasi_monte_calro(m) {
			return;
		}

		/**
		 * @brief destructor
		 */
		inline
		optimize_hybrid_qmc2newton::~optimize_hybrid_qmc2newton(){
		}





		/**
		 * @brief optimization iterate
		 * @param[out] d : difference
		 * @param[out] p : pos
		 * @param[out] v : variance
		 * @param[out] l : likelihood
		 * @return    0 :
		 */
		inline
		int optimize_hybrid_qmc2newton::iterate(matrix::fixed<3,1> *d, matrix::fixed<3,1> *p, double *l){

			{ // ---> operate
				double lk, likelihood = 0;
				double sum;
				matrix::fixed<3,1> delta;
				matrix::fixed<3,1> ws3x1;

				// ---> quasi monte-calro method
				if( particles.size() > 0) {
					set_zero(&delta);
					sum = 0;

					LogVerbose("     : qmc2newton - quasi-monte-calro:\n");
					// ---> loop for compute likelihood
					for(size_t i = 0; i < (unsigned)particles.size(); i++){
						particles[i].likelihood = 0;
						for(size_t j = 0; j < (unsigned)_points.size(); j++ ){
							matrix::fixed<4,1> x;
							matrix::fixed<4,1> X;

							x[0][0] = _points[j][0][0];
							x[1][0] = _points[j][1][0];
							x[2][0] = 0;
							x[3][0] = 1;

							prod( &particles[i].coordm,  &x, &X );

							opsm::likelihood(_map, X[0][0], X[1][0], &lk);
							particles[i].likelihood += lk;
						}
						sum += particles[i].likelihood;
						matrix::scalar_prod( &particles[i].pos, particles[i].likelihood, &ws3x1 );
						add(&delta, &ws3x1, &delta);
					} // ---> loop for compute likelihood

					likelihood = particles[0].likelihood;

					// exception likelihood = 0
					if(sum == 0)	return -1;
					// weighted average
					scalar_div( &delta, sum, &delta );

					{ // ---> compute distribution
						matrix::fixed<3,1> ws3x1;
						matrix::fixed<3,3> ws3x3;

						// ---> compute weighted summation
						matrix::set_zero(&_v.var);
						for(size_t i = 0; i < (unsigned)particles.size(); i++) {
							matrix::sub(&particles[i].pos, &delta, &ws3x1);
							matrix::prod_transpose2(&ws3x1, &ws3x1, &ws3x3);
							matrix::scalar_prod(&ws3x3, particles[i].likelihood, &ws3x3);
							matrix::add(&_v.var, &ws3x3, &_v.var);
						} // <--- compute weighted summation

						matrix::scalar_div(&_v.var, sum, &_v.var);
					} // <--- compute distribution

				} // <--- quasi monte-calro method
				// ---> newton's method
				else {
					int ret;
					uint64_t pi;
					matrix::fixed<3,1> grad;
					matrix::fixed<3,3> hess;
					matrix::fixed<4,4> coordm;

					LogVerbose("     : qmc2newton - newton:\n");
					// coordinate convert matrix
					matrix::coordinate_converter(&coordm,
							_v.pos[0], _v.pos[1], 0,
							::cos(_v.pos[2]), ::sin(_v.pos[2]), 0,
							 0, 0, 1);

					// ---> scanning loop of reflection points
					for( pi = 0; pi < _points.size(); pi++ ){
						matrix::fixed<3,1> g;
						matrix::fixed<3,3> h;

						// compute likelihood, gradient, hessian
						optimize_newton::_newton_method_variables_( _points[pi][0][0], _points[pi][1][0], &coordm, _map, &lk, &g, &h );

						// summation
						likelihood += lk;
						add( &grad, &g, &grad );
						add( &hess, &h, &hess );
					} // <--- scanning loop of reflection points

					// modified newton's method for unconstrained minimizer
					if( (ret = optimize::newtons_method_unconstrainted( &grad, &hess, &delta )) < 0)
						return ret;
				} // <--- newton's method

				LogDebugf("     : qmc2newton - delta = (%lf, %lf, %lf):\n", delta[0][0], delta[1][0], delta[2][0]);
				// merge delta
				matrix::add( &delta, &_v.pos, &_v.pos );
				// store delta
				matrix::copy( &_converge.delta, &delta );

				// output position
				if( p ) matrix::copy( p, &_v.pos );
				// set likelihood
				if( l ) *l = likelihood;
				// output delta
				if( d )	matrix::copy( d, &delta );

				// select qmc or neton's method
				if( !converge_test( gnd_square( _converge.delta[0][0] ) + gnd_square( _converge.delta[1][0] ) , _converge.delta[2][0],
						gnd_square( gnd_m2dist(0.001) ), gnd_deg2ang(1)) ) {
					// quasi monte calro method
					particles.clear();
					create_particle(&_v);
				}
				else {
					// newton's method
					particles.clear();
				}

			} // <--- operate
			return 0;
		}

	}
};

#include "gnd-debug-log-util-undef.h"

// <--- class function definition
#endif /* GND_OPSM_HPP_ */
