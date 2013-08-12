/*
 * gnd-wrap-sys.hpp
 *
 *  Created on: 2013/06/05
 *      Author: tyamada
 */

#ifndef GND_WRAP_SYS_HPP_
#define GND_WRAP_SYS_HPP_

#ifdef _WIN32
#include "gnd-wrap-winsys.hpp"
#endif

#ifdef __linux__
#include "gnd-wrap-linsys.hpp"
#endif

#endif /* GND_WRAP_SYS_HPP_ */
