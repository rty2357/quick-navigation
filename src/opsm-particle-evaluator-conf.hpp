/*
 * opsm-particle-evaluator-conf.hpp
 *
 *  Created on: 2012/03/14
 *      Author: tyamada
 */

#ifndef OPSM_PARTICLE_EVALUATOR_CONF_HPP_
#define OPSM_PARTICLE_EVALUATOR_CONF_HPP_

#include <ssmtype/spur-odometry.h>
#include "ssm-laser.hpp"
#include "ssm-particles.hpp"

#include "opsm-particle-evaluator.hpp"

#include "gnd-opsm.hpp"
#include "gnd-config-file.hpp"
#include "gnd-lib-error.h"


/// structure declaration
// ---> namespace opsm
namespace opsm {
	// ---> namespace peval
	namespace peval {
		/*!
		 * @brief process configuration
		 */
		struct proc_configuration;
	} // <--- namespace opsm
} // <--- namespace peval



/// function declaration
// ---> namespace opsm
namespace opsm {
	// ---> namespace peval
	namespace peval {

		int proc_conf_initialize(proc_configuration *c);
		int proc_conf_get(gnd::conf::configuration *src, proc_configuration* dest);
		int proc_conf_set(gnd::conf::configuration *dest, proc_configuration* src);
		int proc_conf_read(const char* f, proc_configuration* dest);
		int proc_conf_write(const char* f, proc_configuration* src);

	} // <--- namespace opsm
} // <--- namespace peval


// constant definition
// ---> namespace peval
namespace opsm {
	// ---> namespace opsm
	namespace peval {

		static const char proc_name[] = "opsm-particle-evaluator";

		// bmp-map
		static const gnd::conf::parameter_array<char, 512> ConfIni_BMPMap = {
				"bmp-map",
				"",		// file path
				"load map with bmp file"
		};

		// raw-map
		static const gnd::conf::parameter_array<char, 512> ConfIni_RawMap = {
				"cnt-map-dir",
				"",		// map file directory
				"load map with counting map"
		};

		// sokuiki-raw-name
		static const gnd::conf::parameter_array<char, 512> ConfIni_SokuikiRawName = {
				"sokuiki-raw-ssm-name",
				SSM_NAME_SCAN_POINT_2D,		// ssm name
				"sokuiki raw data ssm name"
		};

		// sokuiki-raw-id
		static const gnd::conf::parameter<int> ConfIni_SokuikiRawID = {
				"sokuiki-raw-ssm-id",
				0,		// id
				"sokuiki raw data ssm id"
		};

		// odometry-raw-name
		static const gnd::conf::parameter_array<char, 512> ConfIni_OdometryName = {
				"odometry-ssm-name",
				SNAME_ODOMETRY,		// map file directory
				"odometry data ssm name (for sleeping mode)"
		};

		// odometry-raw-id
		static const gnd::conf::parameter<int> ConfIni_OdometryID = {
				"odometry-ssm-id",
				-1,		// map file directory
				"odometry data ssm id (for sleeping mode)"
		};


		// particle-name
		static const gnd::conf::parameter_array<char, 512> ConfIni_ParticleName = {
				"particles-ssm-name",
				SNAME_PARTICLES,		// ssm name
				"particles data ssm name"
		};

		// particle-id
		static const gnd::conf::parameter<int> ConfIni_ParticleID = {
				"particle-ssm-id",
				0,		// ssm id
				"particles data ssm id"
		};


		// particle-name
		static const gnd::conf::parameter_array<char, 512> ConfIni_ParticleEvalName = {
				"particle-eval-ssm-name",
				SNAME_PARTICLES_EVALUATION,		// ssm name
				"particles evaluation data ssm name"
		};

		// particle-id
		static const gnd::conf::parameter<int> ConfIni_ParticleEvalID = {
				"particle-eval-ssm-id",
				0,		// ssm id
				"particles evaluation data ssm id"
		};


		// cycle
		static const gnd::conf::parameter<double> ConfIni_Cycle = {
				"cycle",
				0.083333,	// [s]
				"evaluation cycle"
		};

		// rest-cycle
		static const gnd::conf::parameter<double> ConfIni_SleepingTime = {
				"sleep-time",
				0.0,	// sec
				"if robot did not move, evaluation stop for this time"
		};
		// rest-threshold-distance
		static const gnd::conf::parameter<double> ConfIni_SleepingDist = {
				"sleep-threshold-distance",
				0.01,	// [m]
				"if robot did not move more than this distance, evaluation stop"
		};
		// rest-threshold-orientation
		static const gnd::conf::parameter<double> ConfIni_SleepingOrient = {
				"sleep-threshold-orientation",
				gnd_deg2rad(5),	// [rad]
				"if robot did not move more than this angle, evaluation stop"
		};


		// cull
		static const gnd::conf::parameter<double> ConfIni_Cull = {
				"cull",
				0.1,	// [m]
				"input data culling parameter"
		};

		// matching failure
		static const gnd::conf::parameter<double> ConfIni_MatchingFailureRate = {
				"matching-failure-rate",
				0.05,
		};

	} // <--- namespace opsm
} // <--- namespace peval


// type definition
// ---> namespace opsm
namespace opsm {
	// ---> namespace peval
	namespace peval {

		/**
		 * @brief process configuration parameter
		 */
		struct proc_configuration {
			// initliaze input
			gnd::conf::parameter_array<char, 512>	bmp_map;			///< Bit Map
			gnd::conf::parameter_array<char, 512>	raw_map;			///< Raw Map Data
			// online input output
			gnd::conf::parameter_array<char, 512>	sokuikiraw_name;	///< Sokuiki ssm data name
			gnd::conf::parameter<int> 				sokuikiraw_id;		///< Sokuiki ssm data id
			gnd::conf::parameter_array<char, 512>	odometry_name;		///< Sokuiki ssm data name
			gnd::conf::parameter<int> 				odometry_id;		///< Sokuiki ssm data id
			gnd::conf::parameter_array<char, 512>	particle_name;		///< particles ssm data name
			gnd::conf::parameter<int>				particle_id;		///< particles ssm data id
			gnd::conf::parameter_array<char, 512>	eval_name;			///< particles evaluation ssm data name
			gnd::conf::parameter<int>				eval_id;			///< particles evaluation ssm data id
			// process behavior configuration
			gnd::conf::parameter<double>			cycle;				///< evaluation cycle
			gnd::conf::parameter<double>			sleeping_time;		///< evaluation cycle in rest mode
			gnd::conf::parameter<double>			sleeping_dist;		///< criteria of rest mode (movement distance threshold)
			gnd::conf::parameter<double>			sleeping_orient;	///< criteria of rest mode (movement orient angle threshold)

			gnd::conf::parameter<double>			cull;				///< reflection point cull
			gnd::conf::parameter<double>			mfailure;			///< reflection point cull

			proc_configuration();
		};

		/**
		 * @brief constructor
		 * @param [in/out] conf : initialized
		 */
		inline
		proc_configuration::proc_configuration() {
			proc_conf_initialize(this);
		}
	} // <--- namespace peval
} // <--- namespace opsm


// function definition
// ---> namespace opsm
namespace opsm {
	// ---> namespace peval
	namespace peval {

		/**
		 * @brief initialize process configuration structure
		 * @param [in/out] conf : initialized
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf) {
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->bmp_map,			&ConfIni_BMPMap,			sizeof(ConfIni_BMPMap));
			::memcpy(&conf->raw_map,			&ConfIni_RawMap,			sizeof(ConfIni_RawMap));
			::memcpy(&conf->sokuikiraw_name,	&ConfIni_SokuikiRawName,	sizeof(ConfIni_SokuikiRawName) );
			::memcpy(&conf->sokuikiraw_id,		&ConfIni_SokuikiRawID,		sizeof(ConfIni_SokuikiRawID) );
			::memcpy(&conf->odometry_name,		&ConfIni_OdometryName,		sizeof(ConfIni_OdometryName) );
			::memcpy(&conf->odometry_id,		&ConfIni_OdometryID,		sizeof(ConfIni_OdometryID) );
			::memcpy(&conf->particle_name,		&ConfIni_ParticleName,		sizeof(ConfIni_ParticleName) );
			::memcpy(&conf->particle_id,		&ConfIni_ParticleID,		sizeof(ConfIni_ParticleID) );
			::memcpy(&conf->eval_name,			&ConfIni_ParticleEvalName,	sizeof(ConfIni_ParticleEvalName) );
			::memcpy(&conf->eval_id,			&ConfIni_ParticleEvalID,	sizeof(ConfIni_ParticleEvalID) );

			::memcpy(&conf->cycle,				&ConfIni_Cycle,				sizeof(ConfIni_Cycle));
			::memcpy(&conf->sleeping_time,		&ConfIni_SleepingTime,		sizeof(ConfIni_SleepingTime));
			::memcpy(&conf->sleeping_dist,		&ConfIni_SleepingDist,		sizeof(ConfIni_SleepingDist));
			::memcpy(&conf->sleeping_orient,	&ConfIni_SleepingOrient,	sizeof(ConfIni_SleepingOrient));

			::memcpy(&conf->cull,				&ConfIni_Cull,				sizeof(ConfIni_Cull));
			::memcpy(&conf->mfailure,			&ConfIni_MatchingFailureRate,				sizeof(ConfIni_MatchingFailureRate));
			return 0;
		}

		/**
		 * @brief get configuration parameter
		 * @param [in]  src  : configuration parameter declaration
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_get(gnd::conf::configuration *src, proc_configuration* dest) {
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::get_parameter(src, &dest->bmp_map);
			gnd::conf::get_parameter(src, &dest->raw_map);
			gnd::conf::get_parameter(src, &dest->sokuikiraw_name);
			gnd::conf::get_parameter(src, &dest->sokuikiraw_id);
			gnd::conf::get_parameter(src, &dest->particle_name);
			gnd::conf::get_parameter(src, &dest->particle_id);
			gnd::conf::get_parameter(src, &dest->odometry_name);
			gnd::conf::get_parameter(src, &dest->odometry_id);
			gnd::conf::get_parameter(src, &dest->eval_name);
			gnd::conf::get_parameter(src, &dest->eval_id);
			gnd::conf::get_parameter(src, &dest->cycle);
			gnd::conf::get_parameter(src, &dest->sleeping_time);
			gnd::conf::get_parameter(src, &dest->sleeping_dist);
			gnd::conf::get_parameter(src, &dest->cull);
			gnd::conf::get_parameter(src, &dest->mfailure);
			if( gnd::conf::get_parameter(src, &dest->sleeping_orient) >= 0 ){
				// convert unit of angle(deg2rad)
				dest->sleeping_orient.value = gnd_deg2rad(dest->sleeping_orient.value);
			}
			return 0;
		}


		/**
		 * @brief set configuration parameter declaration
		 * @param [out] dest : configuration parameter declaration
		 * @param [in]  src  : configuration parameter
		 */
		inline
		int proc_conf_set(gnd::conf::configuration *dest, proc_configuration* src) {
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::set_parameter(dest, &src->bmp_map);
			gnd::conf::set_parameter(dest, &src->raw_map);
			gnd::conf::set_parameter(dest, &src->sokuikiraw_name);
			gnd::conf::set_parameter(dest, &src->sokuikiraw_id);
			gnd::conf::set_parameter(dest, &src->odometry_name);
			gnd::conf::set_parameter(dest, &src->odometry_id);
			gnd::conf::set_parameter(dest, &src->particle_name);
			gnd::conf::set_parameter(dest, &src->particle_id);
			gnd::conf::set_parameter(dest, &src->eval_name);
			gnd::conf::set_parameter(dest, &src->eval_id);
			gnd::conf::set_parameter(dest, &src->cycle);
			gnd::conf::set_parameter(dest, &src->sleeping_time);

			gnd::conf::set_parameter(dest, &src->sleeping_dist);
			// convert unit of angle (rad2deg)
			src->sleeping_orient.value = gnd_rad2deg(src->sleeping_orient.value);

			gnd::conf::set_parameter(dest, &src->sleeping_orient);
			// reconvert unit of angle (rad2deg)
			src->sleeping_orient.value = gnd_deg2rad(src->sleeping_orient.value);

			gnd::conf::set_parameter(dest, &src->cull);
			gnd::conf::set_parameter(dest, &src->mfailure);
			return 0;
		}

		/**
		 * @brief read configuration parameter file
		 * @param [in]  f    : configuration file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_read(const char* f, proc_configuration* dest) {
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(f)) < 0 )	return ret;

				return proc_conf_get(&fs, dest);
			} // <--- operation
		}

		/**
		 * @brief write configuration parameter file
		 * @param [in]  f  : configuration file name
		 * @param [in] src : configuration parameter
		 */
		inline
		int proc_conf_write(const char* f, proc_configuration* src){
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = proc_conf_set(&fs, src)) < 0 ) return ret;

				return fs.write(f);
			} // <--- operation
		}

	} // <--- namespace peval
} // <--- namespace opsm


#endif /* PSM_PARTICLE_EVALUATOR_CONF_HPP_ */
