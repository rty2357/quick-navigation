/*
 * ls-coordinate-converter-cui.hpp
 *
 *  Created on: 2013/12/07
 *      Author: tyamada
 */

#ifndef LS_COORDINATE_CONVERTER_CUI_HPP_
#define LS_COORDINATE_CONVERTER_CUI_HPP_


#include "gnd-cui.hpp"
#include "gnd-util.h"

namespace gnd {
	namespace ls_cc {

		static const gnd::cui_command cui_cmd[] = {
			{"Quit",					'Q',	"shut off process"},
			{"help",					'h',	"show help"},
			{"show",					's',	"state show mode"},
			{"snap-shot",				'n',	"take current scan data"},
			{"", '\0'}
		};

		static const double CuiShowCycle = gnd_sec2time(1.0);
	}
}

#endif /* LS_COORDINATE_CONVERTER_CUI_HPP_ */
