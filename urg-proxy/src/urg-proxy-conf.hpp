/*
 * urg-handler-conf.hpp
 *
 *  Created on: 2012/01/27
 *      Author: tyamada
 */

#ifndef URG_HANDLER_CONF_HPP_
#define URG_HANDLER_CONF_HPP_

#include "gnd-config-file.hpp"

// ---> type declaration
namespace gnd {
	namespace urg_proxy {
		struct proc_configuration;
		typedef struct proc_configuration configuration;
	}
} // <--- type declaration


// ---> constant value definition
namespace gnd {
	namespace urg_proxy {
		// device port
		static const gnd::conf::parameter_array<char, 512> ConfIni_DevicePort = {
				"device-port",
				"/dev/ttyACM0",		// device port path
		};

		// device configuration file
		static const gnd::conf::parameter_array<char, 512> ConfIni_DeviceConf = {
				"dev-conf",
				"",		// map file directory
		};

	}
} // <--- constant value definition



// ---> function declaration
namespace gnd {
	namespace urg_proxy {
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
	namespace urg_proxy {
		/**
		 * \brief particle localizer configure
		 */
		struct proc_configuration {
			proc_configuration();

			gnd::conf::parameter_array<char, 512>	dev_port;
			gnd::conf::parameter_array<char, 512>	dev_conf;
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
	namespace urg_proxy {

		/*!
		 * @brief initialize configure
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->dev_port,	&ConfIni_DevicePort,	sizeof(ConfIni_DevicePort) );
			::memcpy(&conf->dev_conf,	&ConfIni_DeviceConf,	sizeof(ConfIni_DeviceConf) );

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

			gnd::conf::get_parameter(src, &dest->dev_port);
			gnd::conf::get_parameter(src, &dest->dev_conf);
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

			gnd::conf::set_parameter(dest, &src->dev_port);
			gnd::conf::set_parameter(dest, &src->dev_conf);
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
