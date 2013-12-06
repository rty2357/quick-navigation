/*
 * particle-localizer-cui.hpp
 *
 *  Created on: 2012/10/15
 *      Author: tyamada
 */

#ifndef PARTICLE_LOCALIZER_CUI_HPP_
#define PARTICLE_LOCALIZER_CUI_HPP_

#include "gnd-cui.hpp"

const gnd::cui_command cui_cmd[] = {
		{"Quit",		'Q',	"localizer shut-off"},
		{"help",		'h',	"show help"},
		{"show",		's',	"state show mode"},
		{"stand-by",	'B',	"operation stop and wait cui-command"},
		{"start",		'o',	"start operation"},
		{"start-at",	'S',	"initialize position"},
		{"wide",		'w',	"change wide sampling mode on/off"},
		{"debug-log",	'd',	"change debug mode on/off"},
		{"", '\0'}
};

#endif /* PARTICLE_LOCALIZER_CUI_HPP_ */
