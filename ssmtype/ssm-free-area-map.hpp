/*
 * ssm-free-area-map.hpp
 *
 *  Created on: 2011/08/18
 *      Author: tyamada
 */

#ifndef SSM_FREE_AREA_MAP_HPP_
#define SSM_FREE_AREA_MAP_HPP_

#include <ssm.hpp>

#define SNAME_FREE_AREA_MAP	"ssm_free_area_map"

struct SSMFreeAreaMapDummy {
	int dummy;
};

struct SSMFreeAreaMapProperty {
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


typedef SSMApi<SSMFreeAreaMapDummy, SSMFreeAreaMapProperty>	SSMFreeAreaMap;

#endif /* SSM_FREE_AREA_MAP_HPP_ */
