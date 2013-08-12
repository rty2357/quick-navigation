/*
 * ssm-psm-map.hpp
 *
 *  Created on: 2011/08/18
 *      Author: tyamada
 */

#ifndef SSM_OPSM_MAP_HPP_
#define SSM_OPSM_MAP_HPP_

#include <ssm.hpp>

#define SNAME_OPSM_MAP	"ssm_opsm_map"

struct SSMOPSMMapDummy {
	int dummy;
};

struct SSMOPSMMapProperty {
	char fname[256];
	// 画像の角の座標
	// 画像左上から反時計回り
	struct {
		double x;
		double y;
	} point[4];
	double scale;	// [m/pixel]
	double offset;
};


typedef SSMApi<SSMOPSMMapDummy, SSMOPSMMapProperty>	SSMOPSMMap;

#endif /* SSM_OPSM_MAP_HPP_ */
