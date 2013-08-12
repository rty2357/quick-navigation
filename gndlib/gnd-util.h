/*
 * gnd_util.h
 *
 *  Created on: 2011/05/18
 *      Author: tyamada
 */

#ifndef GND_UTIL_H_
#define GND_UTIL_H_


/**
 * @ifnot GNDUtil
 * @defgroup GNDUtil utility
 * @endif
 */

#include <math.h>

#define gnd_sign(x)				( ((x) < 0) ? -1 : 1 )


// ---> distance unit
#define gnd_unit_mm2cm_scale	(10.0)
#define gnd_unit_cm2mm_scale	(0.1)
#define gnd_mm2cm(x)			((x) / gnd_unit_mm2cm_scale)
#define gnd_cm2mm(x)			((x) * gnd_unit_mm2cm_scale)

#define gnd_unit_mm2m_scale		(0.001)
#define gnd_unit_m2mm_scale		(1000.0)
#define gnd_mm2m(x)				((x) / gnd_unit_m2mm_scale)
#define gnd_m2mm(x)				((x) * gnd_unit_m2mm_scale)

#define gnd_unit_cm2m_scale		(0.01)
#define gnd_unit_m2cm_scale		(100.0)
#define gnd_cm2m(x)				((x) * gnd_unit_cm2m_scale)
#define gnd_m2cm(x)				((x) * gnd_unit_m2cm_scale)

#define gnd_unit_dist2mm_scale	gnd_unit_m2mm_scale
#define gnd_unit_mm2dist_scale	gnd_unit_mm2m_scale
#define gnd_unit_dist2cm_scale	gnd_unit_m2cm_scale
#define gnd_unit_cm2dist_scale	gnd_unit_cm2m_scale
#define gnd_unit_dist2m_scale	(1.0)
#define gnd_unit_m2dist_scale	(1.0)

#define gnd_mm2dist(x)			( (x) * gnd_unit_mm2dist_scale )
#define gnd_dist2mm(x)			( (x) * gnd_unit_dist2mm_scale )
#define gnd_cm2dist(x)			( (x) * gnd_unit_cm2dist_scale )
#define gnd_dist2cm(x)			( (x) * gnd_unit_dist2cm_scale )
#define gnd_m2dist(x)			( (x) * gnd_unit_m2dist_scale )
#define gnd_dist2m(x)			( (x) * gnd_unit_dist2m_scale )
// <--- distance unit




// ---> angle unit
#define gnd_unit_rad2ang_scale	( 1.0 )
#define gnd_unit_ang2rad_scale	( 1.0 )
#define gnd_unit_deg2ang_scale	( M_PI / 180.0 )
#define gnd_unit_ang2deg_scale	( 180.0 / M_PI )
#define gnd_rad2ang(x)			( (x) * gnd_unit_rad2ang_scale )
#define gnd_ang2rad(x)			( (x) * gnd_unit_ang2rad_scale )
#define gnd_deg2ang(x)			( (x) * gnd_unit_deg2ang_scale )
#define gnd_ang2deg(x)			( (x) * gnd_unit_ang2deg_scale )


#define gnd_unit_rad2deg_scale	( 180.0 / M_PI )
#define gnd_unit_deg2rad_scale	( M_PI / 180.0 )
#define gnd_rad2deg(x)			( (x) * gnd_unit_rad2deg_scale )
#define gnd_deg2rad(x)			( (x) * gnd_unit_deg2rad_scale )
// <--- angle unit


// ---> time unit
#define gnd_unit_sec2time_scale			(1.0)
#define gnd_unit_time2sec_scale			(1.0)
#define gnd_unit_msec2time_scale		(1.0e-3)
#define gnd_unit_time2msec_scale		(1.0e+3)
#define gnd_unit_usec2time_scale		(1.0e-6)
#define gnd_unit_time2usec_scale		(1.0e+6)
#define gnd_unit_nsec2time_scale		(1.0e-9)
#define gnd_unit_time2nsec_scale		(1.0e+9)

#define gnd_sec2time(x)			((x) * gnd_unit_sec2time_scale )
#define gnd_time2sec(x)			((x) * gnd_unit_time2sec_scale )
#define gnd_msec2time(x)		((x) * gnd_unit_msec2time_scale )
#define gnd_time2msec(x)		((x) * gnd_unit_time2msec_scale )
#define gnd_usec2time(x)		((x) * gnd_unit_usec2time_scale )
#define gnd_time2usec(x)		((x) * gnd_unit_time2usec_scale )
#define gnd_nsec2time(x)		((x) * gnd_unit_nsec2time_scale )
#define gnd_time2nsec(x)		((x) * gnd_unit_time2nsec_scale )

#define gnd_unit_sec2msec_scale		(1.0e+3)
#define gnd_unit_sec2usec_scale		(1.0e+6)
#define gnd_unit_sec2nsec_scale		(1.0e+9)
#define gnd_unit_msec2sec_scale		(1.0e-3)
#define gnd_unit_msec2usec_scale	(1.0e+3)
#define gnd_unit_msec2nsec_scale	(1.0e+6)
#define gnd_unit_usec2sec_scale		(1.0e-6)
#define gnd_unit_usec2msec_scale	(1.0e-3)
#define gnd_unit_usec2nsec_scale	(1.0e+3)
#define gnd_unit_nsec2sec_scale		(1.0e-9)
#define gnd_unit_nsec2msec_scale	(1.0e-6)
#define gnd_unit_nsec2usec_scale	(1.0e-3)

#define gnd_sec2msec(x)			((x)*gnd_unit_sec2msec_scale)
#define gnd_sec2usec(x)			((x)*gnd_unit_sec2usec_scale)
#define gnd_sec2nsec(x)			((x)*gnd_unit_sec2nsec_scale)
#define gnd_msec2sec(x)			((x)*gnd_unit_msec2sec_scale)
#define gnd_msec2usec(x)		((x)*gnd_unit_msec2usec_scale)
#define gnd_msec2nsec(x)		((x)*gnd_unit_msec2nsec_scale)
#define gnd_usec2sec(x)			((x)*gnd_unit_usec2sec_scale)
#define gnd_usec2msec(x)		((x)*gnd_unit_usec2msec_scale)
#define gnd_usec2nsec(x)		((x)*gnd_unit_usec2nsec_scale)
#define gnd_nsec2sec(x)			((x)*gnd_unit_nsec2sec_scale)
#define gnd_nsec2msec(x)		((x)*gnd_unit_nsec2msec_scale)
#define gnd_nsec2usec(x)		((x)*gnd_unit_nsec2usec_scale)
// <--- time unit



/**
 * @ingroup GNDUtil
 * @def
 * @brief normalization of angle
 * @return -pi <= ret < p
 */
#define gnd_rad_normalize(x)	( (x) - ( ( floor(((x) + M_PI ) / (2 * M_PI) ) ) * (2*M_PI)))
/**
 * @ingroup GNDUtil
 * @def
 * @brief normalization of angle
 * @return -180 <= ret < 180
 */
#define gnd_deg_normalize(x)	( (x) - ( ( floor(((x) + 180.0  ) / (360.0)) ) * (360.0)))

#define gnd_rad_normalize_sign(x)	(gnd_rad_normalize(x) < 0 ? gnd_rad_normalize(x) + 2*M_PI : gnd_rad_normalize(x))
#define gnd_deg_normalize_sign(x)	(gnd_deg_normalize(x) < 0 ? gnd_deg_normalize(x) + 360 : gnd_deg_normalize(x))



/**
 * @ingroup GNDUtil
 * @def
 * @brief normalization of angle
 * @param [in] x : angle value
 * @param [in] o : offset ( - pi <= offset < pi)
 * @return offset <= ret < 2* pi + offset
 */
#define gnd_rad_normalize2(x, o)	( (x) - ( ( floor(((x) - (o)) / (2 * M_PI) ) ) * (2 * M_PI)))
/**
 * @ingroup GNDUtil
 * @def
 * @brief normalization of angle
 * @param [in] x : angle value
 * @param [in] o : offset ( -180 <= offset < 180)
 * @return offset <= ret < offset + 360
 */
#define gnd_deg_normalize2(x, o)	( (x) - ( ( floor(( (x) - (o) ) / (360.0)) ) * (360.0)))



#include <time.h>
/**
 * @def
 * @brief compare variables that is type struct timespec
 * @param [i] (struct timespec *) a :
 * @param [i] (struct timespec *) b :
 * @return if <0 , a is less than b
 * @return if =0 , a equal b
 * @return if >0 , a greater than b
 */
#define gnd_timespec_comp(a, b)					\
	(											\
		(  (a)->tv_sec < (b)->tv_sec  ) ? -1 :	\
		(  (a)->tv_sec > (b)->tv_sec  ) ?  1 :	\
		( (a)->tv_nsec < (b)->tv_nsec ) ? -1 :	\
		( (a)->tv_nsec > (b)->tv_nsec ) ?  1 : 	\
		0	/*  a == b*/ 						\
	)

#define gnd_timespec2time(x)	( gnd_sec2time( (x)->tv_sec ) + gnd_nsec2time( (x)->tv_nsec ) )
#define gnd_time2timespec(a, x)											\
	do{																	\
		(a)->tv_sec = (long int)  gnd_time2sec( x );					\
		(a)->tv_nsec = (long int) gnd_time2nsec( (x) - (a)->tv_sec );	\
	}while(0)


#define gnd_timespec_zero(a)				 							\
		do{																\
			(a)->tv_sec = 0;											\
			(a)->tv_nsec = 0;											\
		} while(0)

#define gnd_timespec_add(a,b,c)				 							\
		do{																\
			(c)->tv_sec = (a)->tv_sec + (b)->tv_sec;					\
			for( (c)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec;			\
				 (c)->tv_nsec >= gnd_sec2nsec(1u);						\
				 (c)->tv_sec++, (c)->tv_nsec -= gnd_sec2nsec(1));		\
		} while(0)

#define gnd_timespec_sub(a,b,c)				 							\
		do{																\
			(c)->tv_sec = (a)->tv_sec + (b)->tv_sec;					\
			for( (c)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec;			\
				 (c)->tv_nsec < 0;										\
				 (c)->tv_sec--, (c)->tv_nsec += gnd_sec2nsec(1));		\
		} while(0)


#define gnd_timeval_comp(a, b)					\
	(											\
		(  (a)->tv_sec < (b)->tv_sec  ) ? -1 :	\
		(  (a)->tv_sec > (b)->tv_sec  ) ?  1 :	\
		( (a)->tv_usec < (b)->tv_usec ) ? -1 :	\
		( (a)->tv_usec > (b)->tv_usec ) ?  1 : 	\
		0	/*  a == b*/ 						\
	)

#define gnd_timeval2time(x)	( gnd_sec2time( (x)->tv_sec ) + gnd_usec2time( (x)->tv_usec ) )
#define gnd_time2timeval(a, x)											\
	do{																	\
		(a)->tv_sec = (long int)  gnd_time2sec( x );					\
		(a)->tv_usec = (long int) gnd_time2usec( (x) - (a)->tv_sec );	\
	}while(0)


#define gnd_square(x)				((x) * (x))


#include <string.h>
/**
 * @def
 * @brief get working directory path
 * @param [i] (char * []) e : environmental variables ( the 3rd argument of main function)
 * @param [o] (char * []) c : working directory
 * @param [i]    (size_t) n : length of buffer c
 */
#define gnd_get_working_directory(e, c, n)													\
	do{																		\
		static const char _pwd_[] = "PWD";									\
		static const int _pwd_len_ = strlen(_pwd_);							\
		static const char _sep_ = '=';										\
		char **_dp_;														\
		char *_p_;															\
		for( _dp_ = e; *_dp_ != 0; _dp_++) {								\
			if( strncmp(_pwd_, *_dp_, _pwd_len_ ) ) 			continue;	\
			if( !(_p_ = strchr( (*_dp_) + _pwd_len_, _sep_ )) )	continue;	\
			strncpy(c, ++_p_, n);											\
			break;															\
		}																	\
	}while(0)


#endif /* GND_UTIL_H_ */
