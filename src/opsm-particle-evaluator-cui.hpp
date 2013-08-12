/*
 * opsm-particle-evaluator-cui.hpp
 *
 *  Created on: 2012/03/21
 *      Author: tyamada
 */

#ifndef OPSM_PARTICLE_EVALUATOR_CUI_HPP_
#define OPSM_PARTICLE_EVALUATOR_CUI_HPP_

#include "opsm-particle-evaluator.hpp"
#include "gnd-cui.hpp"

namespace opsm {
	namespace peval {
		const double ShowUpdateCycle = 1.0;

		const gnd::cui_command cui_cmd[] = {
				{"Quit",					'Q',	"localizer shut-off"},
				{"help",					'h',	"show help"},
				{"show",					's',	"state show mode"},
				{"freq",					'f',	"change cycle (frequency)"},
				{"cycle",					'c',	"change cycle (cycle)"},
				{"start",					't',	"start (end stand-by mode)"},
				{"stand-by",				'B',	"stand-by mode"},
				{"likelihood-smoothing",	'l',	"set likelihood-smoothing parameter"},
				{"", '\0'}
		};
	}
}

#endif /* PSM_PARTICLE_EVALUATOR_CUI_HPP_ */
