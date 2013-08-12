/*
 * gnd-odometry-correction-map.hpp
 *
 *  Created on: 2012/10/12
 *      Author: tyamada
 */

#ifndef GND_ODOMETRY_CORRECTION_MAP_HPP_
#define GND_ODOMETRY_CORRECTION_MAP_HPP_


#include "gnd-gridmap3d.hpp"
#include "gnd-util.h"

// ---> constant definition
// ---> namespace gnd
namespace gnd {
	// ---> namespace odometry
	namespace odometry {
		// ---> namespace correction:
		namespace correction {
			static const int InitMapSize = 11;
		} // <--- namespace correction

	} // <--- namespace odometry
}// <--- namespace gnd
// <--- constant definition


// ---> type declaration
// ---> namespace gnd
namespace gnd {
	// ---> namespace odometry
	namespace odometry {
		// ---> namespace rsemap: Road Surface Environmental MAP
		namespace correction {


			/**
			 * @brief difference between localized position with external sensor and odometry position
			 */
			struct vxl {
				double dist;
				double dx;
				double dy;
				double dtheta;
				vxl();
			};


			/**
			 * @brief constructor
			 */
			vxl::vxl() : dist(0), dx(0), dy(0), dtheta(0) {
			}


		} // <--- namespace rsemap: Road Surface Environmental MAP
		/**
		 * @brief road surface environment plane
		 */
		typedef gridsolid<correction::vxl> cmap;
	} // <--- namespace odometry
} // <--- namespace gnd
// <--- type declaration



// ---> function declaration
// ---> namespace gnd
namespace gnd {
	// ---> namespace odometry
	namespace odometry {
		// ---> namespace correction
		namespace correction {
			int create( cmap* map, double d, int a );
			int release( cmap* m );
			int counting(cmap* m, double x, double y, double t, double r, double ex, double ey, double et);
			int get( cmap *m, double x, double y, double t, double *dx, double *dy, double *dt );
			double _orient_offset_(cmap *m);
			double _orient_normalize_(cmap *m, double ori);
		} // <--- namespace correction
	} // <--- namespace odometry
}// <--- namespace gnd




// ---> function definition
// ---> namespace gnd
namespace gnd {
	// ---> namespace odometry
	namespace odometry {
		// ---> namespace correction
		namespace correction {

			/**
			 * @brief create road surface environmental map for proofing odometry
			 * @param [out] m : map
			 * @param [in]	d : grid size
			 * @param [in]	a : anglur resolution
			 */
			inline
			int create( cmap* m, double d, int a ) {
				int ret;
				gnd_assert(!m, -1, "invalid null argument");
				gnd_assert(a < 0, -1, "invalid argument");
				gnd_assert(d <= 0, -1, "invalid argument");


				ret = m->sallocate(InitMapSize, InitMapSize, a, d, d, 2 * M_PI / a);
				gnd_error(ret < 0, -1, "Fail to allocate");
				if( a % 2 )
					m->sset_core(0, 0, 0);
				else
					m->sset_core(0, 0,_orient_offset_(m) );

				return 0;
			}


			/**
			 * @brief create road surface environmental map for proofing odometry
			 * @param [out] m : map
			 * @param [in]	x : grid size
			 * @param [in]	y : grid size
			 * @param [in]	a : anglur resolution
			 */
			inline
			int create( cmap* m, double x, double y, int a ) {
				int ret;
				gnd_assert(!m, -1, "invalid null argument");
				gnd_assert(a < 0, -1, "invalid argument");
				gnd_assert(x <= 0, -1, "invalid argument");
				gnd_assert(y <= 0, -1, "invalid argument");


				ret = m->sallocate(InitMapSize, InitMapSize, a, x, y, 2 * M_PI / a);
				gnd_error(ret < 0, -1, "Fail to allocate");
				if( a % 2 )
					m->sset_core(0, 0, 0);
				else
					m->sset_core(0, 0,_orient_offset_(m) );

				return 0;
			}

			/**
			 * @brief release road surface environmental map for proofing odometry
			 * @param [in/out] m : map
			 */
			inline
			int release( cmap* m ) {
				gnd_assert(!m, -1, "invalid null argument");

				m->deallocate();

				return 0;
			}


			/**
			 * @brief create road surface environmental map for proofing odometry
			 * @param [in/out] m : map
			 * @param [in]	x : position x
			 * @param [in]	y : position y
			 * @param [in]	t : orientation angle
			 * @param [in]  r : running distance
			 * @param [in] ex : error x
			 * @param [in] ey : error y
			 * @param [in] eth: error theta
			 */
			inline
			int counting(cmap* m, double x, double y, double t, double r, double ex, double ey, double et) {
				gnd_assert(!m, -1,  "invalid null argument");
				gnd_assert(!m->is_allocate(), -1, "invalid map data");

				{ // ---> operation
					struct vxl *v = 0;
					double theta = _orient_normalize_(m, t);
					// if memory is lacking, map data reallocate
					for( v = m->spointer( x, y, theta );
						v == 0;
						v = m->spointer( x, y, theta ) ){
						m->reallocate( x, y, theta );
					}

					v->dist += r;
					v->dx += ex;
					v->dy += ey;
					v->dtheta += et;

				} // <--- operation
				return 0;
			}


			inline
			int get( cmap *m, double x, double y, double t, double *dx, double *dy, double *dt ) {
				gnd_assert(!m, -1, "inavlid null argument");
				gnd_assert(!m->is_allocate(), -1, "no map data");

				{ // ---> operation
					struct vxl *v;
					double theta = _orient_normalize_(m, t);

					// zero set
					if( dx ) *dx = 0;
					if( dy ) *dy = 0;
					if( dt ) *dt = 0;

					// if memory is lacking, map data reallocate
					if( !(v = m->spointer( x, y, theta )) ) {
						return -1;
					}

					if( v->dist >= 0.5 ) {
						if( dx ) *dx = v->dx / v->dist;
						if( dy ) *dy = v->dy / v->dist;
						if( dt ) *dt = v->dtheta / v->dist;
					}
					else {
						return -1;
					}

				} // <--- operation
				return 0;
			}


			inline
			double _orient_offset_(cmap *m){
				return M_PI / m->zsize();
			}

			inline
			double _orient_normalize_(cmap *m, double ori) {
				double offset = -M_PI + _orient_offset_(m);
				return gnd_rad_normalize2(ori, offset);
			}

		} // <--- namespace correction
	} // <--- namespace odometry
}// <--- namespace gnd



#endif /* GND_ODOMETRY_CORRECTION_MAP_HPP_ */
