/*
 * opsm-position-tracker-conf.hpp
 *
 *  Created on: 2011/12/22
 *      Author: tyamada
 */

#ifndef OPSM_POSITION_TRACKER_CONF_HPP_
#define OPSM_POSITION_TRACKER_CONF_HPP_

#include <ssmtype/spur-odometry.h>
#include "ssm-laser.hpp"

#include "gnd-opsm.hpp"
#include "gnd-config-file.hpp"
#include "gnd-lib-error.h"

#ifndef opsm_pt
#define opsm_pt opsm::position_tracker
#endif

#define __OPTIMIZER_NEWTON__		"newton"
#define __OPTIMIZER_QMC__	 		"qmc"
#define __OPTIMIZER_MCL__	 		"mcl"
#define __OPTIMIZER_QMC2NEWTON__	"qmc2newton"


// ---> constant definition
namespace opsm {
	namespace position_tracker {
		static const char proc_name[] = "opsm-position-tracker";

		// map-file
		static const gnd::conf::parameter_array<char, 256> ConfIni_ScanMatchingMapDir = {
				"init-opsm-map",
				"",		// map file directory
				"input scan matching map directory path (optional argument)"
		};

		// map-file
		static const gnd::conf::parameter_array<char, 256> ConfIni_CorrectionMapPath = {
				"init-correction-map",
				"",		// map path
				"input correction map file path (optional argument)"
		};

		// ssm-name
		static const gnd::conf::parameter_array<char, 256> ConfIni_OdometrySSMName = {
				"odometry-ssm-name",
				"spur_odometry",
				"odometry ssm name"
		};

		// ssm-id
		static const gnd::conf::parameter<int> ConfIni_OdometrySSMID = {
				"odometry-ssm-id",
				0,
				"odometry ssm id"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter_array<char, 256> ConfIni_LaserScannerSSMName = {
				"laser-scanner-ssm-name",
				"scan_data2d",
				"laser scanner ssm name"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter<int> ConfIni_LaserScannerSSMID = {
				"laser-scanner-ssm-id",
				0,
				"laser scanner ssm id"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter_array<char, 256> ConfIni_CorrectedPosSSMName = {
				"corrected-pos-ssm-name",
				"spur_adjust",
				"laser scanner ssm name"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter<int> ConfIni_CorrectedPosSSMID = {
				"corrected-pos-ssm-name",
				0,
				"corrected position ssm id",
		};



		// ssm corrected position log id
		static const gnd::conf::parameter<int> ConfIni_CorrectedPosLogID = {
				"corrected-position-log-ssmid",
				0,
				"corrected position log id"
		};

		// decimate threshold
		static const gnd::conf::parameter<double> ConfIni_Culling = {
				"culling",
				gnd_m2dist( 0.08 ),	// [m]
				"distance threshold of scan data culling [m]"
		};

		//
		static const gnd::conf::parameter<double> ConfIni_LaserUseDist = {
				"use-range-dist",
				gnd_m2dist( -1 ),	// [m]
				"distance threshold of use laser scanner data for scan matching"
		};

		//
		static const gnd::conf::parameter<double> ConfIni_LaserUseOrient = {
				"use-range-angle",
				gnd_deg2ang(-1),	// [deg]
				"orientation threshold of use laser scanner data for scan matching"
		};


		// failure test parameter threshold
		static const gnd::conf::parameter<double> ConfIni_FailDist = {
				"fail-test-dist",
				gnd_m2dist(1.0),
				"distance threshold for scan matching failure test [m]"
		};

		// failure test parameter orient
		static const gnd::conf::parameter<double> ConfIni_FailOrient = {
				"fail-test-orient",
				gnd_deg2ang(10),
				"orientation threshold for scan matching failure test [deg]"
		};


		// cycle
		static const gnd::conf::parameter<double> ConfIni_Cycle = {
				"cycle",
				gnd_sec2time(0.5),	// [s]
				"scan matching cycle [sec]"
		};

		// rest-cycle
		static const gnd::conf::parameter<double> ConfIni_RestCycle = {
				"pause-time",
				gnd_sec2time(10.0),	// [s]
				"pause scan matching during this time, when robot is stopping"
		};
		// rest-threshold-distance
		static const gnd::conf::parameter<double> ConfIni_RestDist = {
				"pause-distance",
				gnd_m2dist(0.05),	// [m]
				"pause scan matching during robot move less than this parameter ([m])"
		};
		// rest-threshold-orientation
		static const gnd::conf::parameter<double> ConfIni_RestOrient = {
				"pause-orientation",
				gnd_deg2ang(2.5),	// [rad]
				"pause scan matching during robot move less than this parameter ([deg])"
		};

		// rest-threshold-orientation
		static const gnd::conf::parameter<bool> ConfIni_SLAM = {
				"map-update",
				true,
				"this is true, map is updated using this position estimation",
		};
		// map update parameter time
		static const gnd::conf::parameter<double> ConfIni_MapUpdateTime = {
				"map-update-time",
				gnd_sec2time(30),
				"time threshold of map update [sec]"
		};

		// map update parameter threshold
		static const gnd::conf::parameter<double> ConfIni_MapUpdateDist = {
				"map-update-dist",
				gnd_m2dist(0.1),
				"distance threshold of map update [m]"
		};

		// map update parameter orient
		static const gnd::conf::parameter<double> ConfIni_MapUpdateOrient = {
				"map-update-orient",
				gnd_deg2ang(30),
				"orient threshold of map update [deg]"
		};


		// optimizer
		static const char OptNewton[]		= __OPTIMIZER_NEWTON__;
		static const char OptQMC[]			= __OPTIMIZER_QMC__;
		static const char OptMCL[]			= __OPTIMIZER_MCL__;
		static const char OptQMC2Newton[]	= __OPTIMIZER_QMC2NEWTON__;
		static const gnd::conf::parameter_array<char, 256> ConfIni_Optimizer = {
				"optimizer",
				__OPTIMIZER_MCL__,		// map file directory
				"optimize method (newton or mcl or qmc or qmc2newton)"
		};

		// distance threshold of converge test
		static const gnd::conf::parameter<double> ConfIni_ConvergeDist = {
				"converge-distance",
				gnd_m2dist(0.01),	// [m]
				"distance threshold of converge test [m]",
		};
		// orient threshold of converge test
		static const gnd::conf::parameter<double> ConfIni_ConvergeOrient = {
				"converge-orient",
				gnd_deg2ang(0.5),	// [rad]
				"orient threshold of converge test [deg]",
		};

		// number of scan data for first map building
		static const gnd::conf::parameter<int> ConfIni_InitMapCnt = {
				"init-map-cnt",
				1,	// [deg]
				"number of scan data for initial map building",
		};

		// number of scan data for first map building
		static const gnd::conf::parameter<int> ConfIni_InitMatchingCnt = {
				"init-matching-cnt",
				5,	// [deg]
				"count of initial position estimation. in these matching result is not resister on odometry error map",
		};

		// ndt
		static const gnd::conf::parameter<bool> ConfIni_NDT = {
				"ndt",
				false,
				"scan matching evaluation function (ndt or opsm)"
		};


		// debug viewer switch
		static const gnd::conf::parameter<bool> ConfIni_DebugShowMode = {
				"cui-show-mode",
				true,
				"initial cui mode",
		};

		//
		static const gnd::conf::parameter_array<char, 256> ConfIni_OPSMMap = {
				"opsm-map",
				"opsm-map",		// path name
				"opsm map output directory"
		};

		// number of scan data for first map building
		static const gnd::conf::parameter<double> ConfIni_PosGridSizeX = {
				"odometry-error-map-grid-x",
				1.0,	// [m]
				"gird size of odometry error map (x[m])"
		};

		// number of scan data for first map building
		static const gnd::conf::parameter<double> ConfIni_PosGridSizeY = {
				"odometry-error-map-grid-y",
				1.0,
				"gird size of odometry error map (y[m])"
		};

		// number of scan data for first map building
		static const gnd::conf::parameter<int> ConfIni_AngReslution = {
				"odometry-error-map-grid-ori",
				8,	// [m]
				"gird resolution of odometry error map (orientation)"
		};

		// trajectory log
		static const gnd::conf::parameter_array<char, 256> ConfIni_TrajectoryLog = {
				"trajectory-txtlog",
				"",		// file name
				"trajectory log file name (text)"
		};

		// log for route editor
		static const gnd::conf::parameter_array<char, 256> ConfIni_Trajectory4Route = {
				"trajectory-for-route-edit",
				// "spurgl.dat",
				"", // file name
				"name of log file for route editor (text)"
		};

		// laser point log
		static const gnd::conf::parameter_array<char, 256> ConfIni_LaserPointLog = {
				"laser-point-txtlog",
				"",		// file name
				"laser point log file name (text)"
		};

		// bmp
		static const gnd::conf::parameter<bool> ConfIni_BMP = {
				"bmp-map",
				false,		// file name
				"scan matching map file with BMP (for human)"
		};


		// file output directory
		static const gnd::conf::parameter_array<char, 256> ConfIni_OutputDir = {
				"file-output-directory",
				"./",		// file name
				"file output directory"
		};

		// file output directory
		static const gnd::conf::parameter<bool> ConfIni_DebugOdometryErrorMap = {
				"debug-odometry-error-map",
				false
		};

	} // <--- namespace opsm
} // <--- namespace position_tracker
// <--- constant definition


// ---> type declaration
namespace opsm {
	namespace position_tracker {
		struct proc_configuration;
		typedef struct proc_configuration proc_configuration;
	}
}
// ---> type declaration


// ---> function declaration
namespace opsm {
	namespace position_tracker {
		/*
		 * @brief initialize configure to default parameter
		 */
		int proc_conf_initialize(proc_configuration *conf);

		/*
		 * @brief get configuration parameter
		 */
		int proc_conf_get( gnd::conf::configuration* src, proc_configuration* dest );
		/*
		 * @brief set configuration parameter
		 */
		int proc_conf_set( gnd::conf::configuration* dest, proc_configuration* src );

		/*
		 * @brief read configuration parameter
		 */
		int proc_conf_read( const char* f, proc_configuration* dest );
		/*
		 * @brief write configuration parameter
		 */
		int proc_conf_write( const char* f, proc_configuration* src );

	}
}
// ---> function declaration



// ---> type definition
namespace opsm {
	namespace position_tracker {
		/**
		 * \brief particle localizer configure
		 */
		struct proc_configuration {
			proc_configuration();

			gnd::conf::parameter_array<char, 256>	init_opsm_map;			///< scan matching map directory path
			gnd::conf::parameter_array<char, 256>	cmap;				///< correction map path
			gnd::conf::parameter_array<char, 256>	odm_name;			///< odometry position estimation log file name
			gnd::conf::parameter<int>				odm_id;				///< corrected position log id
			gnd::conf::parameter_array<char, 256>	ls_name;			///< laser scanner log file name
			gnd::conf::parameter<int>				ls_id;				///< corrected position log id
			gnd::conf::parameter_array<char, 256>	corrected_name;		///< laser scanner log file name
			gnd::conf::parameter<int>				corrected_id;		///< corrected position log id
			gnd::conf::parameter<double>			culling;			///< laser scanner data decimate parameter [m]
			gnd::conf::parameter<double>			cycle;				///< operation cycle
			gnd::conf::parameter<double>			failure_dist;			///< failure test parameter (distance threshold)
			gnd::conf::parameter<double>			failure_orient;		///< failure test parameter (orient threshold)
			gnd::conf::parameter<double>			use_range_dist;		///< matching data parameter (distance threshold)
			gnd::conf::parameter<double>			use_range_orient;	///< matching data parameter (orient threshold)

			gnd::conf::parameter<double>			pause_time;			///< resting mode cycle
			gnd::conf::parameter<double>			pause_dist;			///< resting threshold (position distance) [m]
			gnd::conf::parameter<double>			pause_orient;		///< resting threshold (position orientation) [deg]
			gnd::conf::parameter<bool>				map_update;				///< slam mode flag
			gnd::conf::parameter<double>			map_update_time;		///< map update parameter (time threshold)
			gnd::conf::parameter<double>			map_update_dist;		///< map update parameter (distance threshold)
			gnd::conf::parameter<double>			map_update_orient;	///< map update parameter (orient threshold)


			gnd::conf::parameter_array<char, 256>	optimizer;			///< kind of optimizer
			gnd::conf::parameter<double>			converge_dist;		///< convergence test threshold (position distance) [m]
			gnd::conf::parameter<double>			converge_orient;	///< convergence test threshold (position orientation) [deg]
			gnd::conf::parameter<int>				ini_map_cnt;		///< number of scan data for first map building
			gnd::conf::parameter<int>				ini_match_cnt;		///< count of initial position estimation. in these matching result is not resister on odometry error map
			gnd::conf::parameter<bool>				ndt;				///< ndt mode
			gnd::conf::parameter<bool>				cui_show;			///< debug show mode

			gnd::conf::parameter_array<char, 256>	opsm_map;			///< opsm map directory
			gnd::conf::parameter<bool>				bmp;				///< opsm map (bmp)

			gnd::conf::parameter<double>			pos_gridsizex;		///< road map position size
			gnd::conf::parameter<double>			pos_gridsizey;		///< road map position size
			gnd::conf::parameter<int>				ang_rsl;			///< road map angular resolution

			gnd::conf::parameter_array<char, 256>	trajectory_log;		///< trajectory log
			gnd::conf::parameter_array<char, 256>	trajectory4route;	///< trajectory for route edit
			gnd::conf::parameter_array<char, 256>	laserpoint_log;		///< laser point log

			gnd::conf::parameter_array<char, 256>	output_dir;			///< file output directory
			gnd::conf::parameter<bool>				debug_odo_err_map;	///< file output directory
		};

		/**
		 * @brief constructor
		 */
		inline
		proc_configuration::proc_configuration(){
			proc_conf_initialize(this);
		}

	}
}
// ---> type definition




// ---> function definition
namespace opsm {
	namespace position_tracker {

		/**
		 * @brief initialize configure
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->odm_name,			&ConfIni_OdometrySSMName,		sizeof(ConfIni_OdometrySSMName) );
			::memcpy(&conf->odm_id,				&ConfIni_OdometrySSMID,			sizeof(ConfIni_OdometrySSMID) );
			::memcpy(&conf->ls_name,			&ConfIni_LaserScannerSSMName,	sizeof(ConfIni_LaserScannerSSMName) );
			::memcpy(&conf->ls_id,				&ConfIni_LaserScannerSSMID,		sizeof(ConfIni_LaserScannerSSMID) );
			::memcpy(&conf->corrected_name,		&ConfIni_CorrectedPosSSMName,	sizeof( ConfIni_CorrectedPosSSMName) );
			::memcpy(&conf->corrected_id,		&ConfIni_CorrectedPosSSMID,		sizeof( ConfIni_CorrectedPosSSMID) );
			::memcpy(&conf->init_opsm_map,		&ConfIni_ScanMatchingMapDir,	sizeof(ConfIni_ScanMatchingMapDir) );
			::memcpy(&conf->cmap,				&ConfIni_CorrectionMapPath,		sizeof(ConfIni_CorrectionMapPath) );
			::memcpy(&conf->culling,			&ConfIni_Culling,				sizeof(ConfIni_Culling) );
			::memcpy(&conf->cycle,				&ConfIni_Cycle,					sizeof(ConfIni_Cycle) );
			::memcpy(&conf->failure_dist,		&ConfIni_FailDist,				sizeof(ConfIni_FailDist) );
			::memcpy(&conf->failure_orient,		&ConfIni_FailOrient,			sizeof(ConfIni_FailOrient) );
			::memcpy(&conf->use_range_dist,		&ConfIni_LaserUseDist,			sizeof(ConfIni_LaserUseDist) );
			::memcpy(&conf->use_range_orient,	&ConfIni_LaserUseOrient,		sizeof(ConfIni_LaserUseOrient) );
			::memcpy(&conf->pause_time,			&ConfIni_RestCycle,				sizeof(ConfIni_RestCycle) );
			::memcpy(&conf->pause_dist,			&ConfIni_RestDist,				sizeof(ConfIni_RestDist) );
			::memcpy(&conf->pause_orient,		&ConfIni_RestOrient,			sizeof(ConfIni_RestOrient) );
			::memcpy(&conf->map_update,			&ConfIni_SLAM,					sizeof(ConfIni_SLAM) );
			::memcpy(&conf->map_update_time,	&ConfIni_MapUpdateTime,			sizeof(ConfIni_MapUpdateTime) );
			::memcpy(&conf->map_update_dist,	&ConfIni_MapUpdateDist,			sizeof(ConfIni_MapUpdateDist) );
			::memcpy(&conf->map_update_orient,	&ConfIni_MapUpdateOrient,		sizeof(ConfIni_MapUpdateOrient) );
			::memcpy(&conf->optimizer,			&ConfIni_Optimizer,				sizeof(ConfIni_Optimizer) );
			::memcpy(&conf->converge_dist,		&ConfIni_ConvergeDist,			sizeof(ConfIni_ConvergeDist) );
			::memcpy(&conf->converge_orient,	&ConfIni_ConvergeOrient,		sizeof(ConfIni_ConvergeOrient) );
			::memcpy(&conf->ini_map_cnt,		&ConfIni_InitMapCnt,			sizeof(ConfIni_InitMapCnt) );
			::memcpy(&conf->ini_match_cnt,		&ConfIni_InitMatchingCnt,		sizeof(ConfIni_InitMatchingCnt) );
			::memcpy(&conf->ndt,				&ConfIni_NDT,					sizeof(ConfIni_NDT) );
			::memcpy(&conf->cui_show,			&ConfIni_DebugShowMode,			sizeof(ConfIni_DebugShowMode) );

			::memcpy(&conf->opsm_map,			&ConfIni_OPSMMap,				sizeof(ConfIni_OPSMMap) );
			::memcpy(&conf->bmp,				&ConfIni_BMP,					sizeof(ConfIni_BMP) );

			::memcpy(&conf->pos_gridsizex,		&ConfIni_PosGridSizeX,			sizeof(ConfIni_PosGridSizeX) );
			::memcpy(&conf->pos_gridsizey,		&ConfIni_PosGridSizeY,			sizeof(ConfIni_PosGridSizeY) );
			::memcpy(&conf->ang_rsl,			&ConfIni_AngReslution,			sizeof(ConfIni_AngReslution) );

			::memcpy(&conf->trajectory_log,		&ConfIni_TrajectoryLog,			sizeof(ConfIni_TrajectoryLog) );
			::memcpy(&conf->trajectory4route,	&ConfIni_Trajectory4Route,		sizeof(ConfIni_Trajectory4Route) );
			::memcpy(&conf->laserpoint_log,		&ConfIni_LaserPointLog,			sizeof(ConfIni_LaserPointLog) );
			::memcpy(&conf->output_dir,			&ConfIni_OutputDir,				sizeof(ConfIni_OutputDir) );
			::memcpy(&conf->debug_odo_err_map,	&ConfIni_DebugOdometryErrorMap,	sizeof(ConfIni_DebugOdometryErrorMap) );

			return 0;
		}

		/**
		 * @brief get configuration parameter
		 * @param  [in] src  : configuration
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_get( gnd::conf::configuration* src, proc_configuration* dest )
		{
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::get_parameter( src, &dest->init_opsm_map );
			gnd::conf::get_parameter( src, &dest->cmap );
			gnd::conf::get_parameter( src, &dest->culling );
			gnd::conf::get_parameter( src, &dest->cycle );
			gnd::conf::get_parameter( src, &dest->failure_dist );
			if( !gnd::conf::get_parameter( src, &dest->failure_orient) )
				dest->failure_orient.value = gnd_deg2ang(dest->failure_orient.value);
			gnd::conf::get_parameter( src, &dest->use_range_dist );
			if( !gnd::conf::get_parameter( src, &dest->use_range_orient) )
				dest->use_range_orient.value = gnd_deg2ang(dest->use_range_orient.value);
			gnd::conf::get_parameter( src, &dest->pause_time );
			gnd::conf::get_parameter( src, &dest->pause_dist );
			if( !gnd::conf::get_parameter( src, &dest->pause_orient ) )
				dest->pause_orient.value = gnd_deg2ang(dest->pause_orient.value);
			gnd::conf::get_parameter( src, &dest->map_update );
			gnd::conf::get_parameter( src, &dest->map_update_time );
			gnd::conf::get_parameter( src, &dest->map_update_dist );
			if( !gnd::conf::get_parameter( src, &dest->map_update_orient) )
				dest->converge_orient.value = gnd_deg2ang(dest->map_update_orient.value);
			gnd::conf::get_parameter( src, &dest->optimizer );
			gnd::conf::get_parameter( src, &dest->converge_dist );
			if( !gnd::conf::get_parameter( src, &dest->converge_orient) )
				dest->converge_orient.value = gnd_deg2ang(dest->converge_orient.value);
			gnd::conf::get_parameter( src, &dest->ini_map_cnt );
			gnd::conf::get_parameter( src, &dest->ini_match_cnt );
			gnd::conf::get_parameter( src, &dest->ndt );
			gnd::conf::get_parameter( src, &dest->cui_show );

			gnd::conf::get_parameter( src, &dest->opsm_map );
			gnd::conf::get_parameter( src, &dest->bmp );

			gnd::conf::get_parameter( src, &dest->pos_gridsizex );
			gnd::conf::get_parameter( src, &dest->pos_gridsizey );
			gnd::conf::get_parameter( src, &dest->ang_rsl );

			gnd::conf::get_parameter( src, &dest->odm_name );
			gnd::conf::get_parameter( src, &dest->odm_id );
			gnd::conf::get_parameter( src, &dest->ls_name );
			gnd::conf::get_parameter( src, &dest->ls_id );
			gnd::conf::get_parameter( src, &dest->corrected_name );
			gnd::conf::get_parameter( src, &dest->corrected_id );

			gnd::conf::get_parameter( src, &dest->trajectory_log );
			gnd::conf::get_parameter( src, &dest->trajectory4route );
			gnd::conf::get_parameter( src, &dest->laserpoint_log );
			gnd::conf::get_parameter( src, &dest->output_dir );
			gnd::conf::get_parameter( src, &dest->debug_odo_err_map );

			return 0;
		}

		/**
		 * @brief file out  configure file
		 */
		inline
		int proc_conf_set( gnd::conf::configuration* dest, proc_configuration* src ) {

			gnd_assert(!dest, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				gnd::conf::set_parameter(dest, &src->odm_name);
				gnd::conf::set_parameter(dest, &src->odm_id);
				gnd::conf::set_parameter(dest, &src->ls_name);
				gnd::conf::set_parameter(dest, &src->ls_id);
				gnd::conf::set_parameter(dest, &src->corrected_name);
				gnd::conf::set_parameter(dest, &src->corrected_id);

				// scan matching parameter
				gnd::conf::set_parameter(dest, &src->cycle);
				gnd::conf::set_parameter(dest, &src->culling);
				gnd::conf::set_parameter(dest, &src->optimizer);
				gnd::conf::set_parameter(dest, &src->converge_dist);
				src->converge_orient.value = gnd_ang2deg(src->converge_orient.value);
				gnd::conf::set_parameter(dest, &src->converge_orient);
				src->converge_orient.value = gnd_deg2ang(src->converge_orient.value);

				gnd::conf::set_parameter(dest, &src->failure_dist);
				src->failure_orient.value = gnd_ang2deg(src->failure_orient.value);
				gnd::conf::set_parameter(dest, &src->failure_orient);
				src->failure_orient.value = gnd_deg2ang(src->failure_orient.value);

				gnd::conf::set_parameter(dest, &src->pause_time);
				gnd::conf::set_parameter(dest, &src->pause_dist);
				src->pause_orient.value = gnd_ang2deg(src->pause_orient.value);
				gnd::conf::set_parameter(dest, &src->pause_orient );
				src->pause_orient.value = gnd_deg2ang(src->pause_orient.value);

				gnd::conf::set_parameter(dest, &src->ini_map_cnt );
				gnd::conf::set_parameter(dest, &src->ini_match_cnt );
				gnd::conf::set_parameter(dest, &src->ndt);

				gnd::conf::set_parameter(dest, &src->use_range_dist);
				src->use_range_orient.value = gnd_ang2deg(src->use_range_orient.value);
				gnd::conf::set_parameter(dest, &src->use_range_orient);
				src->use_range_orient.value = gnd_deg2ang(src->use_range_orient.value);

				gnd::conf::set_parameter(dest, &src->map_update);
				gnd::conf::set_parameter(dest, &src->map_update_time);
				gnd::conf::set_parameter(dest, &src->map_update_dist);
				src->map_update_orient.value = gnd_ang2deg(src->map_update_orient.value);
				gnd::conf::set_parameter(dest, &src->map_update_orient);
				src->map_update_orient.value = gnd_deg2ang(src->map_update_orient.value);

				gnd::conf::set_parameter(dest, &src->output_dir );

				gnd::conf::set_parameter(dest, &src->opsm_map );
				gnd::conf::set_parameter(dest, &src->bmp );

				gnd::conf::set_parameter(dest, &src->pos_gridsizex );
				gnd::conf::set_parameter(dest, &src->pos_gridsizey );
				gnd::conf::set_parameter(dest, &src->ang_rsl );

				gnd::conf::set_parameter(dest, &src->trajectory_log );
				gnd::conf::set_parameter(dest, &src->trajectory4route );
				gnd::conf::set_parameter(dest, &src->laserpoint_log );

				gnd::conf::set_parameter(dest, &src->cui_show );
				gnd::conf::set_parameter(dest, &src->init_opsm_map);
				gnd::conf::set_parameter(dest, &src->cmap );

				return 0;
			} // <--- operation
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
                        if( (ret = fs.read(f)) < 0 )    return ret;

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


	} // namespace position_tracker
}; // namespace opsm
// <--- function definition


#undef __OPTIMIZER_NEWTON__
#undef __OPTIMIZER_QMC__
#undef __OPTIMIZER_QMC2NEWTON__

#endif /* OPSM_POSITION_TRACKER_CONF_HPP_ */
