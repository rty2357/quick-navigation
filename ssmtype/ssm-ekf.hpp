/*
 * ssm-ekf.hpp
 *
 *  Created on: 2011/11/29
 *      Author: tyamada
 */

#ifndef SSM_EKF_HPP_
#define SSM_EKF_HPP_


#define SNAME_EKF_STATE	"ssm-ekf-state"
#define SNAME_EKF_EST	"ssm-ekf-est"

#include "gnd-matrix-base.hpp"


// ---> type definition
namespace EKF {
	enum {
		State_X			= 0,
		State_Y			= 1,
		State_Theta		= 2,
		State_RadiusR	= 3,
		State_RadiusL	= 4,
		State_Tread		= 5,
		State_Bias		= 3,
		State_SF		= 4,
		State_Dim		= 6,
		State_Dim_G		= 5,
		Pos_Dim			= 3,
	};

	enum {
		Observ_ECNT_R		= 0, // right encoder counter
		Observ_ECNT_L		= 1, // left encoder counter
		Observ_V			= 0, // observation v
		Observ_Vol			= 1, // observation gyro sensor reading voltage
		Observ_Dim			= 2,
		Observ_Dim_G 		= 2
	};


	typedef gnd::matrix::fixed<State_Dim, 1> ekf_state_var;
	typedef gnd::matrix::fixed<State_Dim_G, 1> ekf_state_var_gyro;

	template < size_t S >
	struct _ekf_state_ {
		gnd::matrix::fixed<S, 1>	p;
		gnd::matrix::fixed<S, S>	Sigma;
	};
	typedef _ekf_state_<State_Dim> ekf_state;
	typedef _ekf_state_<State_Dim_G> ekf_state_gyro;

	struct ekf_estimation {
		gnd::matrix::fixed<Pos_Dim, 1>			q;
		gnd::matrix::fixed<Pos_Dim, Pos_Dim>	inv_Sigma;
		gnd::matrix::fixed<Pos_Dim, Pos_Dim>	J;
	};


	// random error par sec
	const double _Deff_Random_Error_[State_Dim] = {
			1.0e-2,					// m / sec
			1.0e-2,					// m / sec
			1.0e-1 * M_PI / 180.0,	// rad / sec
			1.0e-6,
			1.0e-6,
			1.0e-6
	};
	const ekf_state_var Def_Random_Error(_Deff_Random_Error_, State_Dim);

	// random error par sec
	const double _Deff_Random_Error_Gyro_[State_Dim_G] = {
			1.0e-2,					// m / sec
			1.0e-2,					// m / sec
			1.0e-1 * M_PI / 180.0,	// rad / sec
			1,						// mV / sec
			1.0e-6,					// (rad/mV) / sec
	};
	const ekf_state_var_gyro Def_Random_Error_Gyro(_Deff_Random_Error_Gyro_, State_Dim_G);

};
typedef EKF::ekf_state ekf_state;
typedef EKF::ekf_state_gyro ekf_state_gyro;
typedef EKF::ekf_estimation ekf_estimation;
// <--- type definition


// ---> function declaration
namespace EKF {
	/**
	 * @brief update position and error distribution following dead-reckoning with odometry
	 * @param [i/o] p : input current position and error distribution and output updated these
	 * @param [i]  rr : right wheel rotate angle
	 * @param [i]  rl : left  wheel rotate angle
	 * @param [i]   a : observation error ratio
	 * @param [i]   e : random error ratio
	 */
	int ekf_odometry(ekf_state *p, double rr, double rl, double t,  double a = 1.0e-3, const ekf_state_var *e = &Def_Random_Error);

	/**
	 * @brief update position and error distribution following dead-reckoning with odometry
	 * @param [i/o] p : input current position and error distribution and output updated these
	 * @param [i]   v : velocity
	 * @param [i]   w : angular velocity
	 * @param [i]   a : error ratio
	 * @param [i]   e : random error ratio
	 */
	int ekf_odometry(ekf_state_gyro *p, double v, double vol, double t, double a = 1.0e-3, const ekf_state_var_gyro *e = &Def_Random_Error_Gyro);

	/**
	 * @brief fusion with result of estimation with external sensor observation
	 * @param [i/o] p : input current position and error distribution and output updated these
	 * @param [i]   e : estimation
	 */
	template< size_t S>
	int ekf_fusion(_ekf_state_<S> *p, ekf_estimation *e);
};
// <--- function declaration



// ---> function definition
namespace EKF {
	/**
	 * @brief update position and error distribution following dead-reckoning with odometry
	 */
	inline
	int ekf_odometry(ekf_state *p, double rr, double rl, double t, double a, const ekf_state_var *e)
	{
		gnd_assert(!p, -1, "invalid null argument");

		{ // ---> operation
			double cosv, sinv;
			double qt = (rr * p->p[State_RadiusR][0] + rl * p->p[State_RadiusL][0]) / 2.0,
				   qr = (rr * p->p[State_RadiusR][0] - rl * p->p[State_RadiusL][0]) / p->p[State_Tread][0];

			cosv = ::cos(p->p[State_Theta][0]);
			sinv = ::sin(p->p[State_Theta][0]);

			{ // ---> transition
				p->p[State_X][0] += qt * cosv;
				p->p[State_Y][0] += qt * sinv;
				p->p[State_Theta][0] += qr;
			} // <--- transition

			{ // ---> error distribution
				gnd::matrix::fixed<State_Dim,State_Dim> J;
				gnd::matrix::fixed<State_Dim, Observ_Dim> K;
				gnd::matrix::fixed<Observ_Dim,Observ_Dim> Sigma_v;

				gnd::matrix::fixed<State_Dim,State_Dim> wsSxS;
				gnd::matrix::fixed<State_Dim,Observ_Dim> wsSxO;

				// J*Sigma_p*J^T
				gnd::matrix::set_unit(&J);
				J[State_X][State_Theta]			= qt * sinv;
				J[State_X][State_RadiusR]		= rr * cosv / 2;
				J[State_X][State_RadiusL]		= rl * cosv / 2;
				J[State_Y][State_Theta]			= -qt * cosv;
				J[State_Y][State_RadiusR]		= rr * sinv / 2;
				J[State_Y][State_RadiusL]		= rl * sinv / 2;
				J[State_Theta][State_RadiusR]	= rr / p->p[State_Tread][0];
				J[State_Theta][State_RadiusL]	= -rl / p->p[State_Tread][0];
				J[State_Theta][State_Tread]		= -qr / p->p[State_Tread][0];
				gnd::matrix::prod(&J, &p->Sigma, &wsSxS);
				gnd::matrix::prod_transpose2(&wsSxS, &J, &p->Sigma);

				// K*Sigma_v*K^T
				K[State_X][Observ_ECNT_R]		= p->p[State_RadiusR][0] * cosv / 2.0;
				K[State_X][Observ_ECNT_L]		= p->p[State_RadiusL][0] * cosv / 2.0;
				K[State_Y][Observ_ECNT_R]		= p->p[State_RadiusR][0] * sinv / 2.0;
				K[State_Y][Observ_ECNT_L]		= p->p[State_RadiusL][0] * sinv / 2.0;
				K[State_Theta][Observ_ECNT_R]	=  p->p[State_RadiusR][0] / p->p[State_Tread][0];
				K[State_Theta][Observ_ECNT_L]	= -p->p[State_RadiusL][0] / p->p[State_Tread][0];
				Sigma_v[Observ_ECNT_R][Observ_ECNT_R] = (a*rr) * (a*rr);
				Sigma_v[Observ_ECNT_L][Observ_ECNT_L] = (a*rl) * (a*rl);
				gnd::matrix::prod(&K, &Sigma_v, &wsSxO);
				gnd::matrix::prod_transpose2(&wsSxO, &K, &wsSxS);

				gnd::matrix::add(&p->Sigma, &wsSxS, &p->Sigma);

				// ---> random error add
				if(e){
					ekf_state_var err;
					gnd::matrix::copy(&err, e);
					for(size_t i = 0; i < State_Dim; i++){
						p->Sigma[i][i] += err[i][0] * err[i][0] * t;
					}
				} // <--- random error add

			} // <--- error distribution

		} // <--- operation

		return 0;
	}


	/*
	 * @brief update position and error distribution following dead-reckoning with odometry
	 */
	inline
	int ekf_odometry(ekf_state_gyro *p, double v, double vol, double t, double a, const gnd::matrix::fixed<State_Dim_G, 1> *e)
	{
		gnd_assert(!p, -1, "invalid null argument");

		{ // ---> operation
			double cosv, sinv;
			double dvol = vol - p->p[State_Bias][0];
			gnd::matrix::fixed<State_Dim_G, 1> err;

			cosv = ::cos(p->p[State_Theta][0]);
			sinv = ::sin(p->p[State_Theta][0]);

			{ // ---> transition
				p->p[State_X][0] += v * t * cosv;
				p->p[State_Y][0] += v * t * sinv;
				p->p[State_Theta][0] += (dvol) * p->p[State_SF][0];
			} // <--- transition

			{ // ---> error distribution
				gnd::matrix::fixed<State_Dim_G,State_Dim_G> J;
				gnd::matrix::fixed<Observ_Dim_G,State_Dim_G> K;
				gnd::matrix::fixed<Observ_Dim_G,Observ_Dim_G> Sigma_v;

				gnd::matrix::fixed<State_Dim_G,State_Dim_G> ws5x5;
				gnd::matrix::fixed<State_Dim_G,Observ_Dim_G> ws5x2;

				// J*Sigma_p*J^T
				gnd::matrix::set_unit(&J);
				J[State_X][State_Theta]		=  v * sinv * t;
				J[State_Y][State_Theta]		= -v * cosv * t;
				J[State_Theta][State_Bias]	= - p->p[State_SF][0];
				J[State_Theta][State_SF]	= dvol;
				gnd::matrix::prod(&J, &p->Sigma, &ws5x5);
				gnd::matrix::prod_transpose2(&ws5x5, &J, &p->Sigma);

				// K*Sigma_v*K^T
				K[State_X][Observ_V]		= cosv * t;
				K[State_Y][Observ_V]		= sinv * t;
				K[State_Theta][Observ_Vol]	= p->p[State_SF][0];
				Sigma_v[Observ_V][Observ_V] = (a*v) * (a*v);
				Sigma_v[Observ_Vol][Observ_Vol] = (a*dvol) * (a*dvol);
				gnd::matrix::prod(&K, &Sigma_v, &ws5x2);
				gnd::matrix::prod_transpose2(&ws5x2, &K, &ws5x5);

				gnd::matrix::add(&p->Sigma, &ws5x5, &p->Sigma);

				// ---> random error add
				if(e){
					ekf_state_var err;
					gnd::matrix::copy(&err, e);
					for(size_t i = 0; i < State_Dim; i++){
						p->Sigma[i][i] += err[i][0] * err[i][0] * t;
					}
				} // <--- random error add

			} // <--- error distribution

		} // <--- operation

		return 0;
	}

	/*
	 * @brief fusion with result of estimation with external sensor observation
	 */
	template< size_t S>
	inline
	int ekf_fusion(_ekf_state_<S> *p, ekf_estimation *e)
	{
		gnd_assert(!p, -1, "invalid null argument");
		gnd_assert(!e, -1, "invalid null argument");

		{ // ---> operation
			gnd::matrix::fixed<S, S> q;
			gnd::matrix::fixed<S, S> einv_Sigma;
			gnd::matrix::fixed<S, S> J;

			gnd::matrix::fixed<S, S> inv_Sigma;
			gnd::matrix::fixed<S, S> wsSxS_1;
			gnd::matrix::fixed<S, S> wsSxS_2;
			gnd::matrix::fixed<S, 1> wsSx1;

			gnd::matrix::copy(&q, &e->q);
			gnd::matrix::copy(&einv_Sigma, &e->inv_Sigma);
			gnd::matrix::copy(&J, &e->J);

			{ // ---> compute fused covariance
				// add minimal value for resolution of round-error
				gnd::matrix::set_unit(&wsSxS_1);
				gnd::matrix::scalar_prod(&wsSxS_1, 1.0e-12, &wsSxS_1);
				gnd::matrix::add(&p->Sigma, &wsSxS_1, &wsSxS_1);

				if( gnd::matrix::inverse(&wsSxS_1, &inv_Sigma) < 0 ){
					return -1;
				}
				gnd::matrix::prod(&J, &einv_Sigma, &wsSxS_1);
				gnd::matrix::prod_transpose2(&wsSxS_1, &J, &wsSxS_2);
				gnd::matrix::add(&inv_Sigma, &wsSxS_2, &wsSxS_1);

				if( gnd::matrix::inverse(&wsSxS_1, &p->Sigma) ){
					return -1;
				}
			} // <--- compute fused covariance

			{ // ---> compute fused position
				gnd::matrix::prod_transpose2(&p->Sigma, &J, &wsSxS_1);
				gnd::matrix::prod(&wsSxS_1, &einv_Sigma, &wsSxS_2);
				gnd::matrix::prod(&wsSxS_2, &q, &wsSx1);

				gnd::matrix::add(&p->p, &wsSx1, &p->p);
			} // <--- compute fused position
		} // <--- operation

		return 0;
	}


};
// <--- function definition

#endif /* SSM_EKF_HPP_ */
