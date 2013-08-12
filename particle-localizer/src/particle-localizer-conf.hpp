/*
 * particle-localizer-conf.hpp
 *
 *  Created on: 2011/10/10
 *      Author: tyamada
 */

#ifndef PARTICLE_LOCALIZER_CONF_HPP_
#define PARTICLE_LOCALIZER_CONF_HPP_

#include "ssm-particles.hpp"

#include "gnd-config-file.hpp"
#include "gnd-matrix-base.hpp"
#include "gnd-lib-error.h"

namespace Localizer {

	/*
	 * @brief kinematics parameter file
	 */
	static const gnd::conf::parameter_array<char, 512> ConfIni_KFile = {
			"kinematics-file",
			"",
			"kinematics parameter file path"
	};

	/*
	 * @brief right wheel radius parameter
	 */
	static const gnd::conf::parameter<double> ConfIni_KRightWheel = {
			"kinematics-right-wheel-radius",
			0
	};

	/*
	 * @brief right wheel radius parameter
	 */
	static const gnd::conf::parameter<bool> ConfIni_KRightWheelCRot = {
			"right-wheel-counter-rot",
			false
	};

	/*
	 * @brief left wheel radius parameter
	 */
	static const gnd::conf::parameter<double> ConfIni_KLeftWheel = {
			"kinematics-left-wheel-radius",
			0
	};

	/*
	 * @brief left wheel radius parameter
	 */
	static const gnd::conf::parameter<bool> ConfIni_KLeftWheelCRot = {
			"left-wheel-counter-rot",
			false
	};

	/*
	 * @brief tread radius parameter
	 */
	static const gnd::conf::parameter<double> ConfIni_KTread = {
			"kinematics-tread",
			0
	};

	/*
	 * @brief tread radius parameter
	 */
	static const gnd::conf::parameter<double> ConfIni_KGear = {
			"kinematics-gear",
			0,
	};

	/*
	 * @brief tread radius parameter
	 */
	static const gnd::conf::parameter<double> ConfIni_KEncoder = {
			"kinematics-encoder-count-rev",
			0,
	};

	/*
	 * @brief configure parameter for number of particles
	 */
	static const gnd::conf::parameter<int> ConfIni_Particles = {
			"particles",
			500,
			"nunber of particles"
	};

	/*
	 * @brief configure parameter remain resampling ratio
	 */
	static const gnd::conf::parameter<double> ConfIni_ResampleRate_Remain = {
			"resample-rate-remain",
			0.35,
			"re-sampling parameter"
	};

	/*
	 * @brief configure parameter random error resampling ratio
	 */
	static const gnd::conf::parameter<double> ConfIni_ResampleRate_RandomError = {
			"resample-rate-random-error",
			0.1,
			"re-sampling parameter "
	};

	/*
	 * @brief configure parameter random error covariance
	 */
	static const gnd::conf::parameter_array< double , PARTICLE_POS_DIM > ConfIni_RandomError = {
			"random-error",
			{ gnd_mm2dist(100), gnd_mm2dist(100), gnd_deg2ang(5) }
	};

	/*
	 * @brief configure parameter random error covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_POS_DIM > ConfIni_RandomErrorOffset = {
			"random-error-offset",
			{ gnd_mm2dist(10), gnd_mm2dist(10), gnd_deg2ang(0.5) }
	};

	/*
	 * @brief configure parameter kinematics ratio
	 */
	static const gnd::conf::parameter<double> ConfIni_ResampleRate_SysError = {
			"resample-rate-systematic-error",
			0.3
	};

	/*
	 * @brief configure parameter kinematics covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_PROP_DIM > ConfIni_SysError = {
			"systematic-error",
			{ 1.0e-5, 1.0e-5, 1.0e-5 }
	};

	/*
	 * @brief configure parameter wide kinematics sampling
	 */
	static const gnd::conf::parameter<double> ConfIni_ResampleRate_SysError2 = {
			"resample-rate-sys-error2",
			0.2
	};

	/*
	 * @brief configure parameter wide kinematics covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_PROP_DIM > ConfIni_SysError2 = {
			"systematic-error2",
			{ 1.0e-3, 1.0e-3, 1.0e-3 }
	};

	/*
	 * @brief configure parameter reset kinematics sampling ratio
	 */
	static const gnd::conf::parameter<double> ConfIni_ResampleRate_ResetSysError = {
			"resample-rate-reset-sys-error",
			0.05
	};

	/*
	 * @brief configure parameter reset kinematics sampling covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_PROP_DIM > ConfIni_ResetSysError = {
			"reset-systematic-error",
			{ 1.0e-5, 1.0e-5, 1.0e-5 }
	};

	/*
	 * @brief configure parameter random sampling
	 */
	static const gnd::conf::parameter<int> ConfIni_RandomSampling = {
			"random-sampling",
			36
	};




	/*
	 * @brief configure parameter random error covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_POS_DIM > ConfIni_Initial_PositionError = {
			"initial-pos-error",
			{ gnd_m2dist(1.0), gnd_m2dist(1.0), gnd_deg2ang(1.0) }
	};

	/*
	 * @brief configure parameter random error covariance
	 */
	static const gnd::conf::parameter_array< double, PARTICLE_PROP_DIM > ConfIni_Initial_SysErrorParam = {
			"initial-systematic-error",
			{ 1.0e-5, 1.0e-5, 1.0e-5 }
	};

	/*
	 * @brief pws ssm id (input)
	 */
	static const gnd::conf::parameter< int > ConfIni_PWSSSM = {
			"pws-ssm-id",
			0
	};

	/*
	 * @brief output ssm id (spur-adjust, particles)
	 */
	static const gnd::conf::parameter< int > ConfIni_SSMID = {
			"ssm-id",
			0
	};



	/*
	 * @brief pws ssm id (input)
	 */
	static const gnd::conf::parameter< bool > ConfIni_Gyro = {
			"gyro",
			0
	};

	/*
	 * @brief gyro voltage
	 */
	static const gnd::conf::parameter< double > ConfIni_GyroVoltage = {
			"gyro-voltage",
			5000
	};

	/*
	 * @brief gyro voltage
	 */
	static const gnd::conf::parameter< int > ConfIni_GyroADBits = {
			"gyro-ad-bits",
			10
	};

	/*
	 * @brief gyro bias
	 */
	static const gnd::conf::parameter< double > ConfIni_GyroBias = {
			"gyro-bias",
			2500
	};

	/*
	 * @brief gyro scale-factor
	 */
	static const gnd::conf::parameter< double > ConfIni_GyroScaleFactor = {
			"gyro-scale-factor",
			1.0 / 1145.9 // rad/sec par mV
	};



	/*
	 * \brief particle localizer configure
	 */
	struct proc_configuration {
		/*
		 * @brief Constructor
		 */
		proc_configuration();

		/*
		 * @brief kinematics parameter file
		 */
		gnd::conf::parameter_array<char, 512>						kfile;

		gnd::conf::parameter<double>								k_rwheel;
		gnd::conf::parameter<bool>									k_rwheel_crot;
		gnd::conf::parameter<double>								k_lwheel;
		gnd::conf::parameter<bool>									k_lwheel_crot;
		gnd::conf::parameter<double>								k_tread;
		gnd::conf::parameter<double>								k_gear;
		gnd::conf::parameter<double>								k_encoder;

		/*
		 * @brief number of particles
		 */
		gnd::conf::parameter<int>									particles;

		/*
		 * @brief initial position
		 */
		gnd::conf::parameter_array<double, PARTICLE_POS_DIM>		pos_err_ini;

		/*
		 * @brief initial position variance
		 */
		gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM>		poserr_cover_ini;

		/*
		 * @brief initial kinematics covariance
		 */
		gnd::conf::parameter_array<double, PARTICLE_PROP_DIM>		sys_err_ini;
		/*
		 * @brief initial kinematics variance parameter
		 */
		gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM>	syserr_cover_ini;

		/*
		 * @brief remain re-sampling rate
		 */
		gnd::conf::parameter<double>								resmp_rate_remain;
		/*
		 * @brief position error re-sampling rate
		 */
		gnd::conf::parameter<double>								resmp_rate_randerr;

		/*
		 * @brief position error variance parameter
		 */
		gnd::conf::parameter_array<double, PARTICLE_POS_DIM>		randerr_conf;
		/*
		 * @brief position error covariance
		 */
		gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM>		randerr_covar;

		/*
		 * @brief position error static variance parameter
		 */
		gnd::conf::parameter_array<double, PARTICLE_POS_DIM>		randerr_offset_conf;
		/*
		 * @brief position error static covariance
		 */
		gnd::matrix::fixed<PARTICLE_POS_DIM, PARTICLE_POS_DIM>		randerr_covar_offset;

		/*
		 * @brief kinematic re-sampling rate
		 */
		gnd::conf::parameter<double>								resmp_rate_syserr;

		/*
		 * @brief kinematic variance parameter
		 */
		gnd::conf::parameter_array<double, PARTICLE_PROP_DIM>		syserr_conf;
		/*
		 * @brief kinematic covariance
		 */
		gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM>	syserr_cover;

		/*
		 * @brief wide kinematic variance parameter
		 */
		gnd::conf::parameter<double>								resmp_rate_syserr2;
		/*
		 * @brief wide kinematic variance parameter
		 */
		gnd::conf::parameter_array<double, PARTICLE_PROP_DIM>		syserr2_conf;
		/*
		 * @brief wide kinematic covariance
		 */
		gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM>	syserr2_covar;

		/*
		 * @brief reset kinematic variance parameter
		 */
		gnd::conf::parameter<double>								resmp_rate_resetsyserr;
		/*
		 * @brief reset kinematic variance parameter
		 */
		gnd::conf::parameter_array<double, PARTICLE_PROP_DIM>		reset_syserr_conf;
		/*
		 * @brief reset kinematic variance parameter
		 */
		gnd::matrix::fixed<PARTICLE_PROP_DIM, PARTICLE_PROP_DIM>	reset_syserr_covar;

		/*
		 * @brief number of random sampling particle
		 */
		gnd::conf::parameter<int>									random_sampling;

		/*
		 * @brief pws ssm-id (input)
		 */
		gnd::conf::parameter<int>									pws_id;

		/*
		 * @brief output ssm-id
		 */
		gnd::conf::parameter<int>									ssm_id;

		/*
		 * @brief gyro
		 */
		gnd::conf::parameter<bool>									gyro;

		/*
		 * @brief gyro voltage
		 */
		gnd::conf::parameter<double>								gyro_vol;

		/*
		 * @brief gyro-bits
		 */
		gnd::conf::parameter<int>									gyro_bits;

		/*
		 * @brief gyro-bias
		 */
		gnd::conf::parameter<double>								gyro_bias;


		/*
		 * @brief gyro-scalefactor
		 */
		gnd::conf::parameter<double>								gyro_sf;

	};
	typedef struct proc_configuration configure_parameters;


	/**
	 * @brief initialize configure to default parameter
	 */
	int proc_conf_initialize(proc_configuration *conf);
	/**
	 * @brief sampling ratio parameter normalize
	 */
	int proc_conf_sampling_ratio_normalize(proc_configuration *conf);
	/**
	 * @brief kinematics parameter materialize
	 */
	int proc_conf_set_kinematics_parameter(proc_configuration *conf, double wr, double wl, double tread);

	/**
	 * @brief analyze configure file
	 */
	int proc_conf_analyze(gnd::conf::configuration *fconf, proc_configuration *confp);


	// constructor
	proc_configuration::proc_configuration(){
		proc_conf_initialize(this);
	}


	inline
	int proc_conf_sampling_ratio_normalize(proc_configuration *conf)
	{
		gnd_assert(!conf, -1, "invalid null pointer");

		{
			double sum = 0;

			sum += conf->resmp_rate_remain.value;
			sum += conf->resmp_rate_randerr.value;
			sum += conf->resmp_rate_syserr.value;
			sum += conf->resmp_rate_syserr2.value;
			sum += conf->resmp_rate_resetsyserr.value;

			if( sum <= 0 ){
				return -1;
			}
			else {
				conf->resmp_rate_remain.value /= sum;
				conf->resmp_rate_randerr.value /= sum;
				conf->resmp_rate_syserr.value /= sum;
				conf->resmp_rate_syserr2.value /= sum;
				conf->resmp_rate_resetsyserr.value /= sum;
			}
		}
		return 0;
	}



	inline
	int configure_get_covariance(proc_configuration *conf)
	{
		gnd_assert(!conf, -1, "invalid null pointer");

		gnd::matrix::set_unit( &conf->poserr_cover_ini );
		for(size_t i = 0; i < 2; i++)
			gnd::matrix::set(&conf->poserr_cover_ini, i, i, gnd_square( conf->pos_err_ini.value[i] ) );
		gnd::matrix::set(&conf->poserr_cover_ini, 2, 2, gnd_square( gnd_deg2ang( conf->pos_err_ini.value[2] ) ) );

		gnd::matrix::set_unit( &conf->syserr_cover_ini );
		for(size_t i = 0; i < 3; i++)
			gnd::matrix::set(&conf->syserr_cover_ini, i, i, gnd_square( conf->sys_err_ini.value[i] ) );

		gnd::matrix::set_unit( &conf->randerr_covar );
		for(size_t i = 0; i < 2; i++)
			gnd::matrix::set(&conf->randerr_covar, i, i, gnd_square( conf->randerr_conf.value[i] ) );
		gnd::matrix::set(&conf->randerr_covar, 2, 2, gnd_square( gnd_deg2ang( conf->randerr_conf.value[2] ) ) );

		gnd::matrix::set_unit( &conf->randerr_covar_offset );
		for(size_t i = 0; i < 2; i++)
			gnd::matrix::set(&conf->randerr_covar_offset, i, i, gnd_square( conf->randerr_offset_conf.value[i] ) );
		gnd::matrix::set(&conf->randerr_covar_offset, 2, 2, gnd_square( gnd_deg2ang( conf->randerr_offset_conf.value[2] ) ) );

		return 0;
	}


	/*!
	 * @brief initialize configure
	 */
	inline
	int proc_conf_initialize(proc_configuration *conf){
		gnd_assert(!conf, -1, "invalid null pointer");

		::memcpy(&conf->kfile,						&ConfIni_KFile,							sizeof(ConfIni_KFile));
		::memcpy(&conf->k_lwheel,					&ConfIni_KLeftWheel,					sizeof(ConfIni_KLeftWheel));
		::memcpy(&conf->k_lwheel_crot,				&ConfIni_KLeftWheelCRot,					sizeof(ConfIni_KLeftWheelCRot));
		::memcpy(&conf->k_rwheel,					&ConfIni_KRightWheel,					sizeof(ConfIni_KRightWheel));
		::memcpy(&conf->k_rwheel_crot,				&ConfIni_KRightWheelCRot,				sizeof(ConfIni_KRightWheelCRot));
		::memcpy(&conf->k_tread,					&ConfIni_KTread,						sizeof(ConfIni_KTread));
		::memcpy(&conf->k_gear,						&ConfIni_KGear,							sizeof(ConfIni_KGear));
		::memcpy(&conf->k_encoder,					&ConfIni_KEncoder,						sizeof(ConfIni_KEncoder));

		::memcpy(&conf->particles,					&ConfIni_Particles,						sizeof(ConfIni_Particles));
		::memcpy(&conf->resmp_rate_remain,			&ConfIni_ResampleRate_Remain,			sizeof(ConfIni_ResampleRate_Remain));

		::memcpy(&conf->pos_err_ini,				&ConfIni_Initial_PositionError,			sizeof(ConfIni_Initial_PositionError));
		::memcpy(&conf->sys_err_ini,				&ConfIni_Initial_SysErrorParam,			sizeof(ConfIni_Initial_SysErrorParam));

		::memcpy(&conf->randerr_conf,				&ConfIni_RandomError,					sizeof(ConfIni_RandomError));
		::memcpy(&conf->resmp_rate_randerr,			&ConfIni_ResampleRate_RandomError,		sizeof(ConfIni_ResampleRate_RandomError));
		::memcpy(&conf->randerr_offset_conf,		&ConfIni_RandomErrorOffset,				sizeof(ConfIni_RandomErrorOffset));
		::memcpy(&conf->resmp_rate_syserr,			&ConfIni_ResampleRate_SysError,			sizeof(ConfIni_ResampleRate_SysError));
		::memcpy(&conf->syserr_conf,				&ConfIni_SysError,						sizeof(ConfIni_SysError));
		::memcpy(&conf->resmp_rate_syserr2,			&ConfIni_ResampleRate_SysError2,		sizeof(ConfIni_ResampleRate_SysError2));
		::memcpy(&conf->syserr2_conf,				&ConfIni_SysError2,						sizeof(ConfIni_SysError2));
		::memcpy(&conf->resmp_rate_resetsyserr,		&ConfIni_ResampleRate_ResetSysError,	sizeof(ConfIni_ResampleRate_ResetSysError));
		::memcpy(&conf->reset_syserr_conf,			&ConfIni_ResetSysError,					sizeof(ConfIni_ResetSysError));

		::memcpy(&conf->random_sampling,			&ConfIni_RandomSampling,			sizeof(ConfIni_RandomSampling));

		::memcpy(&conf->pws_id,						&ConfIni_PWSSSM,					sizeof(ConfIni_PWSSSM));
		::memcpy(&conf->ssm_id,						&ConfIni_SSMID,						sizeof(ConfIni_SSMID));

		::memcpy(&conf->gyro,						&ConfIni_Gyro,						sizeof(ConfIni_Gyro));
		::memcpy(&conf->gyro_vol,					&ConfIni_GyroVoltage,				sizeof(ConfIni_GyroVoltage));
		::memcpy(&conf->gyro_bits,					&ConfIni_GyroADBits,				sizeof(ConfIni_GyroADBits));
		::memcpy(&conf->gyro_bias,					&ConfIni_GyroBias,					sizeof(ConfIni_GyroBias));
		::memcpy(&conf->gyro_sf,					&ConfIni_GyroScaleFactor,			sizeof(ConfIni_GyroScaleFactor));

		proc_conf_sampling_ratio_normalize(conf);
		configure_get_covariance(conf);
		return 0;
	}

	inline
	int proc_conf_set_kinematics_parameter(proc_configuration *conf, double wr, double wl, double tread)
	{
		double wheel_mean = (wl + wr) / 2.0;
		double wheel_ratio = wl / wheel_mean;
		double tread_ratio = tread / wheel_mean;

		{ // ---> initlaize systematic error
			gnd::matrix::set_unit(&conf->syserr_cover_ini);
			if( !conf->gyro.value ){
				gnd::matrix::set(&conf->syserr_cover_ini, 0, 0,
						gnd_square( conf->sys_err_ini.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr_cover_ini, 1, 1,
						gnd_square( conf->sys_err_ini.value[1] * gnd_square(wheel_ratio) )  );
				gnd::matrix::set(&conf->syserr_cover_ini, 2, 2,
						gnd_square( conf->sys_err_ini.value[2] * gnd_square(tread_ratio) )  );
			}
			else {
				gnd::matrix::set(&conf->syserr_cover_ini, 0, 0,
						gnd_square( conf->sys_err_ini.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr_cover_ini, 1, 1,
						gnd_square( conf->sys_err_ini.value[1] * conf->gyro_bias.value )  );
				gnd::matrix::set(&conf->syserr_cover_ini, 2, 2,
						gnd_square( conf->sys_err_ini.value[2] * conf->gyro_sf.value )  );
			}
		} // <--- initlaize systematic error

		{ // ---> resampleing for systematic error
			gnd::matrix::set_unit(&conf->syserr_cover);
			if( !conf->gyro.value ){
				gnd::matrix::set(&conf->syserr_cover, 0, 0,
						gnd_square( conf->syserr_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr_cover, 1, 1,
						gnd_square( conf->syserr_conf.value[1] * gnd_square(wheel_ratio) )  );
				gnd::matrix::set(&conf->syserr_cover, 2, 2,
						gnd_square( conf->syserr_conf.value[2] * gnd_square(tread_ratio) )  );
			}
			else {
				gnd::matrix::set(&conf->syserr_cover, 0, 0,
						gnd_square( conf->syserr_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr_cover, 1, 1,
						gnd_square( conf->syserr_conf.value[1] * conf->gyro_bias.value )  );
				gnd::matrix::set(&conf->syserr_cover, 2, 2,
						gnd_square( conf->syserr_conf.value[2] * conf->gyro_sf.value )  );
			}
		} // <--- resampleing for systematic error

		{ // ---> resampleing for systematic error
			gnd::matrix::set_unit(&conf->syserr2_covar);
			if( !conf->gyro.value ){
				gnd::matrix::set(&conf->syserr2_covar, 0, 0,
						gnd_square( conf->syserr2_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr2_covar, 1, 1,
						gnd_square( conf->syserr2_conf.value[1] * gnd_square(wheel_ratio) )  );
				gnd::matrix::set(&conf->syserr2_covar, 2, 2,
						gnd_square( conf->syserr2_conf.value[2] * gnd_square(tread_ratio) )  );
			}
			else {
				gnd::matrix::set(&conf->syserr2_covar, 0, 0,
						gnd_square( conf->syserr2_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->syserr2_covar, 1, 1,
						gnd_square( conf->syserr2_conf.value[1] * conf->gyro_bias.value )  );
				gnd::matrix::set(&conf->syserr2_covar, 2, 2,
						gnd_square( conf->syserr2_conf.value[2] * conf->gyro_sf.value )  );
			}
		} // <--- resampleing for systematic error

		{ // ---> resampleing for systematic error reset
			gnd::matrix::set_unit(&conf->reset_syserr_covar);
			if( !conf->gyro.value ){
				gnd::matrix::set(&conf->reset_syserr_covar, 0, 0,
						gnd_square( conf->reset_syserr_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->reset_syserr_covar, 1, 1,
						gnd_square( conf->reset_syserr_conf.value[1] * gnd_square(wheel_ratio) )  );
				gnd::matrix::set(&conf->reset_syserr_covar, 2, 2,
						gnd_square( conf->reset_syserr_conf.value[2] * gnd_square(tread_ratio) )  );
			}
			else {
				gnd::matrix::set(&conf->reset_syserr_covar, 0, 0,
						gnd_square( conf->reset_syserr_conf.value[0] * gnd_square(wheel_mean) )  );
				gnd::matrix::set(&conf->reset_syserr_covar, 1, 1,
						gnd_square( conf->reset_syserr_conf.value[1] * conf->gyro_bias.value )  );
				gnd::matrix::set(&conf->reset_syserr_covar, 2, 2,
						gnd_square( conf->reset_syserr_conf.value[2] * conf->gyro_sf.value )  );
			}
		} // <--- resampleing for systematic error reset
		return 0;
	}





	// configure file data analyze
	inline
	int proc_conf_get(gnd::conf::configuration *src, proc_configuration* dest) {
		gnd_assert(!src, -1, "invalid null pointer");
		gnd_assert(!dest, -1, "invalid null pointer");

		gnd::conf::get_parameter(src, &dest->kfile);
		gnd::conf::get_parameter(src, &dest->k_rwheel);
		gnd::conf::get_parameter(src, &dest->k_rwheel_crot);
		gnd::conf::get_parameter(src, &dest->k_lwheel);
		gnd::conf::get_parameter(src, &dest->k_lwheel_crot);
		gnd::conf::get_parameter(src, &dest->k_tread);
		gnd::conf::get_parameter(src, &dest->k_gear);
		gnd::conf::get_parameter(src, &dest->k_encoder);
		gnd::conf::get_parameter(src, &dest->particles);
		if( gnd::conf::get_parameter(src, &dest->pos_err_ini ) >= 3) {
			dest->pos_err_ini.value[2] = gnd_deg2ang(dest->pos_err_ini.value[2]);
		}
		gnd::conf::get_parameter(src, &dest->sys_err_ini);
		gnd::conf::get_parameter(src, &dest->resmp_rate_remain);
		gnd::conf::get_parameter(src, &dest->resmp_rate_randerr);

		if( gnd::conf::get_parameter(src, &dest->randerr_conf ) >= 3) {
			dest->randerr_conf.value[2] = gnd_deg2ang(dest->randerr_conf.value[2]);
		}

		gnd::conf::get_parameter(src, &dest->randerr_offset_conf );

		gnd::conf::get_parameter(src, &dest->syserr_conf );
		gnd::conf::get_parameter(src, &dest->resmp_rate_syserr2);
		gnd::conf::get_parameter(src, &dest->syserr2_conf );

		gnd::conf::get_parameter(src, &dest->resmp_rate_resetsyserr);
		gnd::conf::get_parameter(src, &dest->reset_syserr_conf );

		gnd::conf::get_parameter(src, &dest->random_sampling);
		gnd::conf::get_parameter(src, &dest->pws_id);
		gnd::conf::get_parameter(src, &dest->ssm_id);
		gnd::conf::get_parameter(src, &dest->gyro);
		gnd::conf::get_parameter(src, &dest->gyro_vol);
		gnd::conf::get_parameter(src, &dest->gyro_bits);
		gnd::conf::get_parameter(src, &dest->gyro_bias);
		gnd::conf::get_parameter(src, &dest->gyro_sf);

		proc_conf_sampling_ratio_normalize(dest);
		configure_get_covariance(dest);
		return 0;
	}


	// configure file data analyze
	inline
	int proc_conf_set(gnd::conf::configuration *dest, proc_configuration* src) {
		gnd_assert(!src, -1, "invalid null pointer");
		gnd_assert(!dest, -1, "invalid null pointer");

		gnd::conf::set_parameter(dest, &src->kfile);
		gnd::conf::set_parameter(dest, &src->k_rwheel);
		gnd::conf::set_parameter(dest, &src->k_rwheel_crot);
		gnd::conf::set_parameter(dest, &src->k_lwheel);
		gnd::conf::set_parameter(dest, &src->k_lwheel_crot);
		gnd::conf::set_parameter(dest, &src->k_tread);
		gnd::conf::set_parameter(dest, &src->k_gear);
		gnd::conf::set_parameter(dest, &src->k_encoder);
		gnd::conf::set_parameter(dest, &src->particles);
		src->pos_err_ini.value[2] = gnd_ang2deg(src->pos_err_ini.value[2]);
		gnd::conf::set_parameter(dest, &src->pos_err_ini);
		src->pos_err_ini.value[2] = gnd_deg2ang(src->pos_err_ini.value[2]);
		gnd::conf::set_parameter(dest, &src->sys_err_ini);
		gnd::conf::set_parameter(dest, &src->resmp_rate_remain);
		gnd::conf::set_parameter(dest, &src->resmp_rate_randerr);

		src->randerr_conf.value[2] = gnd_ang2deg(src->randerr_conf.value[2]);
		gnd::conf::set_parameter(dest, &src->randerr_conf);
		src->randerr_conf.value[2] = gnd_deg2ang(src->randerr_conf.value[2]);

		src->randerr_offset_conf.value[2] = gnd_ang2deg(src->randerr_offset_conf.value[2]);
		gnd::conf::set_parameter(dest, &src->randerr_offset_conf);
		src->randerr_offset_conf.value[2] = gnd_deg2ang(src->randerr_offset_conf.value[2]);


		gnd::conf::set_parameter(dest, &src->resmp_rate_syserr);
		gnd::conf::set_parameter(dest, &src->syserr_conf);

		gnd::conf::set_parameter(dest, &src->resmp_rate_syserr2);
		gnd::conf::set_parameter(dest, &src->syserr2_conf);

		gnd::conf::set_parameter(dest, &src->resmp_rate_resetsyserr);
		gnd::conf::set_parameter(dest, &src->reset_syserr_conf);
		gnd::conf::set_parameter(dest, &src->random_sampling);
		gnd::conf::set_parameter(dest, &src->pws_id);
		gnd::conf::set_parameter(dest, &src->ssm_id);
		gnd::conf::set_parameter(dest, &src->gyro);
		gnd::conf::set_parameter(dest, &src->gyro_vol);
		gnd::conf::set_parameter(dest, &src->gyro_bits);
		gnd::conf::set_parameter(dest, &src->gyro_bias);
		gnd::conf::set_parameter(dest, &src->gyro_sf);

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
};

#endif /* PARTICLE_LOCALIZER_CONF_HPP_ */
