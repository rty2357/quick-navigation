/*
 * urg-proxy-device-conf.hpp
 *
 *  Created on: 2012/01/28
 *      Author: tyamada
 */

#ifndef URG_PROXY_DEVICE_CONF_HPP_
#define URG_PROXY_DEVICE_CONF_HPP_


#include <time.h>
#include "gnd-matrix-base.hpp"
#include "gnd-config-file.hpp"
#include "gnd-lib-error.h"

// ---> constant value definition
namespace gnd {
	namespace urg_proxy {

		const char Token_Serial[] = "<Serial>";
		// ssm-name
		static const gnd::conf::parameter_array<char, 64> ConfIni_SSMName = {
				"ssm-name",
				"scan-data2d",
		};
		// ssm-id
		static const gnd::conf::param_int ConfIni_SSMID = {
				"ssm-id",
				10,
		};
		// time-adjust
		static const gnd::conf::parameter_array<int, 2> ConfIni_TimeAdjust = {
				"timeadjust",
				{ 1, 8, },
		};
		// reflection
		static const gnd::conf::param_bool ConfIni_Reflect = {
				"reflect",
				false,
		};
		// scan range
		static const gnd::conf::parameter_array<double, 2> ConfIni_AngleRange = {
				"angle-range",
				{ -180, 180, },
		};
		// position
		static const gnd::conf::parameter_array<double, 3> ConfIni_Position = {
				"position",
				{ 0, 0, 0, },
		};
		// orient
		static const gnd::conf::parameter_array<double, 3> ConfIni_Orient = {
				"orient",
				{ 1.00, 0.00, 0.00, },
		};
		// upside
		static const gnd::conf::parameter_array<double, 3> ConfIni_Upside = {
				"upside",
				{ 0.00, 0.00, 1.00, },
		};
	}
}
// <--- constant value definition


// ---> type declaration
namespace gnd {
	namespace urg_proxy {
		struct device_configuration;
		typedef struct device_configuration device_configuration;
	}
}
// <--- type declaration


// ---> function declaration
namespace gnd {
	namespace urg_proxy {
		/**
		 * @brief initialize configure to default parameter
		 */
		int dev_conf_init(device_configuration *conf);

		/**
		 * @brief analyze configure file
		 */
		int dev_conf_get(gnd::conf::configuration *fconf, device_configuration *dest);

		/**
		 * @brief file out  configure file
		 */
		int dev_conf_write( const char* fname, device_configuration *confp );
	}
}
// <--- function declaration





// ---> type definition
namespace gnd {
	namespace urg_proxy {
		/*
		 * @parameter device configuration
		 */
		struct device_configuration {
			char									serial[64];
			gnd::conf::parameter_array<char, 64>	name;
			gnd::conf::param_int					id;
			gnd::conf::param_bool					reflect;
			gnd::conf::parameter_array<int, 2>		timeadjust;
			gnd::conf::parameter_array<double, 2>	angle_range;
			gnd::conf::parameter_array<double, 3>	position;
			gnd::conf::parameter_array<double, 3>	orient;
			gnd::conf::parameter_array<double, 3>	upside;
			device_configuration();
		};

		device_configuration::device_configuration() {
			dev_conf_init(this);
		}
	}
}
// <--- type definition


// ---> function definition
namespace gnd {
	namespace urg_proxy {

		/*!
		 * @brief initialize configure
		 */
		inline
		int dev_conf_init(device_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memset(conf->serial,			0,						sizeof(conf->serial));
			::strcpy(conf->serial,			Token_Serial);

			::memcpy(&conf->name,			&ConfIni_SSMName,		sizeof(ConfIni_SSMName) );
			::memcpy(&conf->id,				&ConfIni_SSMID,			sizeof(ConfIni_SSMID) );
			::memcpy(&conf->timeadjust,		&ConfIni_TimeAdjust,	sizeof(ConfIni_TimeAdjust) );
			::memcpy(&conf->reflect,		&ConfIni_Reflect,		sizeof(ConfIni_Reflect) );
			::memcpy(&conf->angle_range,	&ConfIni_AngleRange,	sizeof(ConfIni_AngleRange) );
			::memcpy(&conf->position,		&ConfIni_Position,		sizeof(ConfIni_Position) );
			::memcpy(&conf->orient,			&ConfIni_Orient,		sizeof(ConfIni_Orient) );
			::memcpy(&conf->upside,			&ConfIni_Upside,		sizeof(ConfIni_Upside) );
			return 0;
		}

		/*
		 * @brief analyze
		 */
		inline
		int dev_conf_get(gnd::conf::configuration *src, device_configuration *dest)
		{
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::get_parameter(src, &dest->name);
			gnd::conf::get_parameter(src, &dest->id);
			gnd::conf::get_parameter(src, &dest->timeadjust);
			gnd::conf::get_parameter(src, &dest->reflect);
			gnd::conf::get_parameter(src, &dest->angle_range);
			gnd::conf::get_parameter(src, &dest->position);
			gnd::conf::get_parameter(src, &dest->orient);
			gnd::conf::get_parameter(src, &dest->upside);
			return 0;
		}

		/*
		 * @brief analyze
		 */
		inline
		int dev_conf_set(gnd::conf::configuration *dest, device_configuration *src)
		{
			gnd::conf::configuration *ch;
			gnd_assert(!dest, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			dest->child_push_back(src->serial, 0, 0);
			if( (ch = dest->child_find(src->serial) ) ) {

				gnd::conf::set_parameter(ch, &src->name);
				gnd::conf::set_parameter(ch, &src->id);
				gnd::conf::set_parameter(ch, &src->timeadjust);
				gnd::conf::set_parameter(ch, &src->reflect);
				gnd::conf::set_parameter(ch, &src->angle_range);
				gnd::conf::set_parameter(ch, &src->position);
				gnd::conf::set_parameter(ch, &src->orient);
				gnd::conf::set_parameter(ch, &src->upside);
			}

			return 0;
		}

		/**
		 * @brief read configuration parameter file
		 * @param [in]  f    : configuration file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int dev_conf_read(const char* f, device_configuration* dest) {
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(f)) < 0 )    return ret;

				return dev_conf_get(&fs, dest);
			} // <--- operation
		}


		/**
		 * @brief file out  configure file
		 */
		inline
		int dev_conf_write( const char* f, device_configuration *src ){
			gnd_assert(!f, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				if( (ret = dev_conf_set(&fs, src)) < 0 )	return ret;

				return fs.write(f);
			} // <--- operation
		}

	}
}
// <--- function definition

#endif /* URG_PROXY_DEVICE_CONF_HPP_ */
