/*
 * ls-coordinate-converter-conf.hpp
 *
 *  Created on: 2013/05/23
 *      Author: tyamada
 */

#ifndef LS_COORDINATE_CONVERTER_CONF_HPP_
#define LS_COORDINATE_CONVERTER_CONF_HPP_

#include "gnd-config-file.hpp"
#include "ssm-laser.hpp"

// ---> type declaration
namespace gnd {
	namespace ls_cc {
		struct proc_configuration;
		typedef struct proc_configuration configuration;
	}
} // <--- type declaration


// ---> constant value definition
namespace gnd {
	namespace ls_cc {
		// laser scanner raw data ssm name
		static const gnd::conf::parameter_array<char, 256> ConfIni_RawSSMName = {
				"ssm-name-raw",
				SSM_NAME_SCAN_POINT_2D,		// device port path
				"laser scanner raw data ssm name"
		};

		// laser scanner raw data ssm id
		static const gnd::conf::parameter<int> ConfIni_RawSSMID = {
				"ssm-id-raw",
				0,		// device port path
				"laser scanner raw data ssm id"
		};

		// laser scanner fs data ssm name
		static const gnd::conf::parameter_array<char, 256> ConfIni_FSSSMName = {
				"ssm-name-fs",
				SSM_NAME_SOKUIKI_3D_FS,		// device port path
				"laser scanner fs data ssm name"
		};

		// laser scanner fs data ssm id
		static const gnd::conf::parameter<int> ConfIni_FSSSMID = {
				"ssm-id-fs",
				-1,
				"laser scanner fs data ssm id. if this value is less than 0, fs id is same to raw"
		};

		// laser scanner fs storage life
		static const gnd::conf::parameter<double> ConfIni_FSStorageLife = {
				"storage-life-fs",
				5,	 // [sec]
				"laser scanner fs data storage life on ssm"
		};




		// laser scanner gl data ssm name
		static const gnd::conf::parameter_array<char, 256> ConfIni_GLSSMName = {
				"ssm-name-gl",
				"",		// ssm name
				"laser scanner gl data ssm name"
		};

		// position data ssm name
		static const gnd::conf::parameter_array<char, 256> ConfIni_GLPosSSMName = {
				"ssm-name-position",
				"",							// ssm name
				"position data ssm namefor coordinate convert to global coodinate"
		};

		// laser scanner fs data ssm name
		static const gnd::conf::parameter<int> ConfIni_GLPosSSMID = {
				"ssm-id-position",
				-1,							// ssm id
				"position data ssm id for coordinate convert to global coodinate"
		};

		// laser scanner fs data ssm id
		static const gnd::conf::parameter<int> ConfIni_GLSSMID = {
				"ssm-id-gl",
				-1,
				"laser scanner gl data ssm id. if this value is less than 0, fs id is same to raw"
		};

		// laser scanner fs storage life
		static const gnd::conf::parameter<double> ConfIni_GLStorageLife = {
				"storage-life-gl",
				5,	 // [sec]
				"laser scanner gl data storage life on ssm"
		};

		// gl log
		static const gnd::conf::parameter_array<char, 256> ConfIni_GLLogFile= {
				"scan-gl-logtxt",
				"",							// ssm name
				"scan data log on global coordinate (txt format)"
		};

	}
} // <--- constant value definition



// ---> function declaration
namespace gnd {
	namespace ls_cc {
		/**
		 * @brief initialize configure to default parameter
		 */
		int proc_conf_initialize(proc_configuration *conf);

		/**
		 * @brief analyze configure file
		 */
		int analyze_configure(gnd::conf::configuration *fconf, proc_configuration *confp);

		/**
		 * @brief file out  configure file
		 */
		int proc_conf_write( const char* fname, proc_configuration *confp );
	}
} // <--- function declaration



// ---> type definition
namespace gnd {
	namespace ls_cc {
		/**
		 * \brief particle localizer configure
		 */
		struct proc_configuration {
			proc_configuration();

			gnd::conf::parameter_array<char, 256>	raw_name;
			gnd::conf::parameter<int>				raw_id;
			gnd::conf::parameter_array<char, 256>	fs_name;
			gnd::conf::parameter<int>				fs_id;
			gnd::conf::parameter<double>			fs_storage;
			gnd::conf::parameter_array<char, 256>	gl_name;
			gnd::conf::parameter<int>				gl_id;
			gnd::conf::parameter_array<char, 256>	gl_pos_name;
			gnd::conf::parameter<int>				gl_pos_id;
			gnd::conf::parameter<double>			gl_storage;
			gnd::conf::parameter_array<char, 256>	gl_txtlog;
		};

		typedef struct proc_configuration proc_configuration;

		/**
		 * @brief constructor
		 */
		inline
		proc_configuration::proc_configuration(){
			proc_conf_initialize(this);
		}
	}
} // <--- type definition



// ---> function definition
namespace gnd {
	namespace ls_cc {

		/*!
		 * @brief initialize configure
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->raw_name,		&ConfIni_RawSSMName,	sizeof(ConfIni_RawSSMName) );
			::memcpy(&conf->raw_id,			&ConfIni_RawSSMID,		sizeof(ConfIni_RawSSMID) );
			::memcpy(&conf->fs_name,		&ConfIni_FSSSMName,		sizeof(ConfIni_FSSSMName) );
			::memcpy(&conf->fs_id,			&ConfIni_FSSSMID,		sizeof(ConfIni_FSSSMID) );
			::memcpy(&conf->fs_storage,		&ConfIni_FSStorageLife,	sizeof(ConfIni_FSStorageLife) );
			::memcpy(&conf->gl_name,		&ConfIni_GLSSMName,		sizeof(ConfIni_GLSSMName) );
			::memcpy(&conf->gl_pos_name,	&ConfIni_GLPosSSMName,	sizeof(ConfIni_GLPosSSMName) );
			::memcpy(&conf->gl_pos_id,		&ConfIni_GLPosSSMID,	sizeof(ConfIni_GLPosSSMID) );
			::memcpy(&conf->gl_id,			&ConfIni_GLSSMID,		sizeof(ConfIni_GLSSMID) );
			::memcpy(&conf->gl_storage,		&ConfIni_GLStorageLife,	sizeof(ConfIni_GLStorageLife) );
			::memcpy(&conf->gl_txtlog,		&ConfIni_GLLogFile,		sizeof(ConfIni_GLLogFile) );

			return 0;
		}

		/*
		 * @brief analyze
		 */
		inline
		int proc_conf_get(gnd::conf::configuration *src, proc_configuration *dest)
		{
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::get_parameter(src, &dest->raw_name);
			gnd::conf::get_parameter(src, &dest->raw_id);
			gnd::conf::get_parameter(src, &dest->fs_name);
			gnd::conf::get_parameter(src, &dest->fs_id);
			gnd::conf::get_parameter(src, &dest->fs_storage);
			gnd::conf::get_parameter(src, &dest->gl_name);
			gnd::conf::get_parameter(src, &dest->gl_id);
			gnd::conf::get_parameter(src, &dest->gl_pos_name);
			gnd::conf::get_parameter(src, &dest->gl_pos_id);
			gnd::conf::get_parameter(src, &dest->gl_storage);
			gnd::conf::get_parameter(src, &dest->gl_txtlog);
			return 0;
		}


		/*
		 * @brief set configuration parameter
		 */
		inline
		int proc_conf_set(gnd::conf::configuration *dest, proc_configuration *src)
		{
			gnd_assert(!dest, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			gnd::conf::set_parameter(dest, &src->raw_name);
			gnd::conf::set_parameter(dest, &src->raw_id);
			gnd::conf::set_parameter(dest, &src->fs_name);
			gnd::conf::set_parameter(dest, &src->fs_id);
			gnd::conf::set_parameter(dest, &src->fs_storage);
			gnd::conf::set_parameter(dest, &src->gl_name);
			gnd::conf::set_parameter(dest, &src->gl_id);
			gnd::conf::set_parameter(dest, &src->gl_pos_name);
			gnd::conf::set_parameter(dest, &src->gl_pos_id);
			gnd::conf::set_parameter(dest, &src->gl_storage);
			gnd::conf::set_parameter(dest, &src->gl_txtlog);
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
				if( (ret = fs.read(f)) < 0 )    return ret;

				return proc_conf_get(&fs, dest);
			} // <--- operation
		}

		/**
		 * @brief file out  configure file
		 */
		inline
		int proc_conf_write( const char* f, proc_configuration *src ){
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				if( (ret = proc_conf_set(&fs, src)) < 0 )	return ret;

				return fs.write(f);
			} // <--- operation
		}

	}
}; // <--- function definition


#endif /* URG_HANDLER_CONF_HPP_ */
