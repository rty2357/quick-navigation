/*
 * gnd-wrap-lin-sys.hpp
 *
 *  Created on: 2013/06/05
 *      Author: tyamada
 */

#ifndef GND_WRAP_LINSYS_HPP_
#define GND_WRAP_LINSYS_HPP_

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace gnd {
	const int wO_BINARY	= 0x00;
	const int wO_TEXT	= 0x00;

	const int wS_IRUSR = S_IRUSR;
	const int wS_IWUSR = S_IWUSR;
	const int wS_IXUSR = S_IXUSR;
	const int wS_IRGRP = S_IRGRP;
	const int wS_IWGRP = S_IWGRP;
	const int wS_IXGRP = S_IXGRP;
	const int wS_IROTH = S_IROTH;
	const int wS_IWOTH = S_IWOTH;
	const int wS_IXOTH = S_IXOTH;
}


#endif /* GND_WRAP_LINSYS_HPP_ */
