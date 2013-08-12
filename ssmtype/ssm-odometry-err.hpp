/*
 * ssm-odometry-err.hpp
 *
 *  Created on: 2012/04/17
 *      Author: tyamada
 */

#ifndef SSM_ODOMETRY_ERR_HPP_
#define SSM_ODOMETRY_ERR_HPP_

#include <ssm.hpp>
#include "ssm-odometry-err.hpp"

#define SNAME_ODOMETRY_ERR	"odometry-err"

struct odometry_error {
	double dx;
	double dy;
	double dtheta;
};

typedef SSMApi<odometry_error>	SSMApi_OdometryErr;

#endif /* SSM_ODOMETRY_ERR_HPP_ */
