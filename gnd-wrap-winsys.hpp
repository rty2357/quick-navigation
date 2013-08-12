/*
 * gnd-wrap-winsys.hpp
 *
 *  Created on: 2013/06/05
 *      Author: tyamada
 */

#ifndef GND_WRAP_WINSYS_HPP_
#define GND_WRAP_WINSYS_HPP_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace gnd {

	const int wO_BINARY	= O_BINARY;
	const int wO_TEXT	= O_TEXT;

	const int wS_IRUSR = _S_IREAD;
	const int wS_IWUSR = _S_IWRITE;
	const int wS_IXUSR = 0x00;
	const int wS_IRGRP = 0x00;
	const int wS_IWGRP = 0x00;
	const int wS_IXGRP = 0x00;
	const int wS_IROTH = 0x00;
	const int wS_IWOTH = 0x00;
	const int wS_IXOTH = 0x00;
}


#endif /* GND_WRAP_WINSYS_HPP_ */
