/*
 * gnd_bmp.hpp
 *
 *  Created on: 2011/08/10
 *      Author: tyamada
 */

#ifndef GND_BMP_HPP_
#define GND_BMP_HPP_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "gnd-gridmap.hpp"

#include "gnd-wrap-sys.hpp"

/**
 * @defgroup GNDBmp bitmap
 * supply bitmap file read write
 */

// ---> constant value definition
namespace gnd {
	namespace bmp {
		static const char FILETYPE[] = "BM";			///< bit map file type tag
		static const int FILEHEADER_SIZE = 14;			///< bit map file header byte size
		static const int INFOHEADER_SIZE = 40;			///< bit map info header byte size
		static const int INFOHEADER_COMMOM_SIZE = 16;	///< bit map info common header byte size
		static const int RGBQUAD_SIZE = 4;				///< rgb quad byte size
	}
}
// <--- constant value definition


// ---> type definition
namespace gnd {
	namespace bmp {

		/**
		 * @privatesection
		 * @ingroup GNDBmp
		 * @brief bit map file header
		 */
		struct FILEHEADER{
			unsigned short	bfType;			///< file type
			uint32_t		bfSize;			///< file size
			unsigned short	bfReserved1;	///< reserve (0)
			unsigned short	bfReserved2;	///< reserve (0)
			uint32_t		bfOffBits;		///< offset byte size at image data
		};
		typedef struct FILEHEADER FILEHEADER;


		/**
		 * @privatesection
		 * @ingroup GNDBmp
		 * @brief bit map info header
		 */
		struct INFOHEADER{
			uint32_t		biSize;			///< info header size
			int32_t			biWidth;		///< width (pixel)
			int32_t			biHeight;		///< height (pixel)
			unsigned short	biPlanes;		///< number of plane (1)
			unsigned short	biBitCount;		///< data size per pixel
			uint32_t  		biCompression;	///< compression method
			uint32_t		biSizeImage;	///< image size
			int32_t			biXPixPerMeter;	///< pixel per meter on x-axis
			int32_t			biYPixPerMeter;	///< pixel per meter on y-axis
			uint32_t		biClrUsed;		///< number of color index
			uint32_t		biClrImporant;	///< number of important color index
		};
		typedef struct INFOHEADER INFOHEADER;

		/**
		 * @privatesection
		 * @ingroup GNDBmp
		 * @brief RGB quad
		 */
		struct RGBQUAD {
			unsigned char rgbBlue;			///< blue
			unsigned char rgbGreen;			///< green
			unsigned char rgbRed;			///< red
			unsigned char rgbReserved;		///< reserve
		};
		typedef struct RGBQUAD RGBQUAD;

	}
	/**
	 * @typedef bmp_gray_t
	 * @brief bit map data type
	 */
	typedef gridmap::gridplane<unsigned char> bmp8_t;

	/**
	 * @typedef bmp_t
	 * @brief bit map data type
	 */
	typedef gridmap::gridplane<uint32_t> bmp32_t;
} // <--- type definition



// ---> function declaration
namespace gnd {
	namespace bmp {
		int write( const char *fname, bmp8_t *gm );
		int read( const char *fname, bmp8_t *gm );

		int write( const char *fname, bmp32_t *gm );
		int read( const char *fname, bmp32_t *gm );

		int write8( const char *fname, bmp8_t *gm );
		int read8( const char *fname, bmp8_t *gm );

		int write32( const char *fname, bmp32_t *gm);
		int read32( const char *fname, bmp32_t *gm);
	}
}
// <--- function declaration


// ---> function definition
namespace gnd {
	namespace bmp {

		/**
		 * @ingroup GNDBmp
		 * @brief write bit map file
		 * @param[in] fname : file name
		 * @param[in]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int write( const char *fname, bmp8_t *gm ) {
			return write8(fname, gm);
		}


		/**
		 * @ingroup GNDBmp
		 * @brief read bit map file
		 * @param[in]  fname : file name
		 * @param[out]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int read( const char *fname, bmp8_t *gm ) {
			return read8(fname, gm);
		}

		/**
		 * @ingroup GNDBmp
		 * @brief write bit map file
		 * @param[in] fname : file name
		 * @param[in]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int write( const char *fname, bmp32_t *gm ) {
			return write32(fname, gm);
		}

		/**
		 * @ingroup GNDBmp
		 * @brief read bit map file
		 * @param[in]  fname : file name
		 * @param[out]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int read( const char *fname, bmp32_t *gm ) {
			return read32(fname, gm);
		}


		/**
		 * @ingroup GNDBmp
		 * @brief write gray bit map file
		 * @param[in] fname : file name
		 * @param[in]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int write8( const char *fname, bmp8_t *gm)
		{
			FILEHEADER fheader;
			INFOHEADER iheader;
			const size_t pixlsize = 1;
			int linesize, imagesize;
			struct {
				double x;
				double y;
			} org;
			size_t pad;


			{ // ---> initialize
				linesize = pixlsize * gm->column();
				pad = (linesize % 4);
				linesize += pad ? 4 - pad : 0;
				imagesize = linesize * gm->row();
				gm->pget_origin(&org.x, &org.y);
			} // <--- initialize


			{ // ---> file header
				::memset(&fheader, 0, sizeof(fheader));

				::memcpy(&fheader.bfType, FILETYPE, sizeof(fheader.bfType));

				fheader.bfSize = FILEHEADER_SIZE + INFOHEADER_SIZE + imagesize
						+ RGBQUAD_SIZE * 256;

				fheader.bfReserved1 = 0;
				fheader.bfReserved2 = 0;
				fheader.bfOffBits = FILEHEADER_SIZE + INFOHEADER_SIZE
						+ RGBQUAD_SIZE * 256;
			} // <--- file header


			{ // ---> info header
				// add origin data
				iheader.biSize = INFOHEADER_SIZE;
				iheader.biWidth = gm->column();
				iheader.biHeight = gm->row();
				iheader.biPlanes = 1;
				iheader.biBitCount = 8 * pixlsize;
				iheader.biCompression = 0;
				iheader.biSizeImage = imagesize;
				iheader.biXPixPerMeter = ::round( 1.0 / gm->xrsl() );
				iheader.biYPixPerMeter = ::round( 1.0 / gm->yrsl() );
				iheader.biClrUsed      = 0;
				iheader.biClrImporant  = 0;

			} // <--- info header



			{ // ---> file out
				int fd;
				const unsigned char d = 0x00;	// dummy
				RGBQUAD	rgb;

				fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT | gnd::wO_BINARY,
						gnd::wS_IRUSR | gnd::wS_IWUSR |
						gnd::wS_IRGRP | gnd::wS_IWGRP );
				if(fd < 0) return -1;

				{ // ---> write file header
					if( ::write(fd, &fheader.bfType, sizeof(unsigned short))		< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfSize, sizeof(uint32_t))			< (signed) sizeof(uint32_t) )	return -2;
					if( ::write(fd, &fheader.bfReserved1, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfReserved2, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfOffBits, sizeof(uint32_t))		< (signed) sizeof(uint32_t) )	return -2;
				} // <--- write file header

				{ // ---> write info header
					if( ::write(fd, &iheader.biSize, sizeof(uint32_t) )			< (signed) sizeof(uint32_t) )	return -2;
					if( ::write(fd, &iheader.biWidth, sizeof(int32_t) )					< (signed) sizeof(int32_t) )			return -2;
					if( ::write(fd, &iheader.biHeight, sizeof(int32_t) )				< (signed) sizeof(int32_t) )			return -2;
					if( ::write(fd, &iheader.biPlanes, sizeof(unsigned short) )		< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &iheader.biBitCount, sizeof(unsigned short) )	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &iheader.biCompression, sizeof(uint32_t) )	< (signed) sizeof(uint32_t) )	return -2;
					if( ::write(fd, &iheader.biSizeImage, sizeof(uint32_t) )	< (signed) sizeof(uint32_t) )	return -2;
					if( ::write(fd, &iheader.biXPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int32_t) )			return -2;
					if( ::write(fd, &iheader.biYPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int32_t) )			return -2;
					if( ::write(fd, &iheader.biClrUsed, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )	return -2;
					if( ::write(fd, &iheader.biClrImporant, sizeof(uint32_t) )	< (signed) sizeof(uint32_t) )	return -2;
				} // <--- write info header

				// ---> write colar pallete
				for(size_t i = 0; i < (1<<8); i++){
					rgb.rgbBlue = rgb.rgbGreen = rgb.rgbRed = i;
					if( ::write(fd, &rgb.rgbBlue,     1) < 1)	return -2;
					if( ::write(fd, &rgb.rgbGreen,    1) < 1)	return -2;
					if( ::write(fd, &rgb.rgbRed,      1) < 1)	return -2;
					if( ::write(fd, &rgb.rgbReserved, 1) < 1)	return -2;
				} // <--- write colar pallete



				// write image data
				for( size_t h = 0; h < (unsigned)iheader.biHeight; h++ ){
					for( size_t w = 0; w < (unsigned)iheader.biWidth; w += gm->_unit_column_() ){
						if( ::write(fd, gm->pointer(h, w), gm->_unit_column_() * sizeof(unsigned char)) <
								((signed)gm->_unit_column_() * (signed)sizeof(unsigned char)) )	return -1;
					}
					for( size_t i = 0; i < pad; i++ ){
						if( ::write(fd, &d, 1) < 1)	return -1;
					}
				}

				//		if( ::write(fd, &org, sizeof(org)) < (signed) sizeof(org))	return -1;

				close(fd);
			} // <--- file out

			return 0;
		}



		/**
		 * @ingroup GNDBmp
		 * @brief read gray bit map file
		 * @param[in]  fname : file name
		 * @param[out]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int read8( const char *fname, bmp8_t *gm)
		{
			int fd;
			FILEHEADER fheader;
			INFOHEADER iheader;
			const size_t pixlsize = 1;
			size_t pad;


			{ // ---> initialize
				fd = open(fname, O_RDWR | gnd::wO_BINARY );
				if(fd < 0) return -1;
			} // <--- initialize


			{ // ---> file header
				::memset(&fheader, 0, sizeof(fheader));
				if( ::read(fd, &fheader.bfType,      sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfSize,      sizeof(uint32_t))		< (signed) sizeof(uint32_t) )	return -2;
				if( ::read(fd, &fheader.bfReserved1, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfReserved2, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfOffBits,   sizeof(uint32_t))		< (signed) sizeof(uint32_t) )	return -2;
			} // <--- file header

			{ // ---> info header
				::memset(&iheader, 0, sizeof(iheader));
				if( ::read(fd, &iheader.biSize, sizeof(uint32_t) )			< (signed) sizeof(uint32_t) )	return -2;
				if( ::read(fd, &iheader.biWidth, sizeof(int32_t) )					< (signed) sizeof(int32_t) )			return -2;
				if( ::read(fd, &iheader.biHeight, sizeof(int32_t) )					< (signed) sizeof(int32_t) )			return -2;
				if( ::read(fd, &iheader.biPlanes, sizeof(unsigned short) )		< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &iheader.biBitCount, sizeof(unsigned short) )	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &iheader.biCompression, sizeof(uint32_t) )	< (signed) sizeof(uint32_t) )	return -2;
				if( ::read(fd, &iheader.biSizeImage, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )	return -2;
				if( ::read(fd, &iheader.biXPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int32_t) )			return -2;
				if( ::read(fd, &iheader.biYPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int32_t) )			return -2;
				if( ::read(fd, &iheader.biClrUsed, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )	return -2;
				if( ::read(fd, &iheader.biClrImporant, sizeof(uint32_t) )	< (signed) sizeof(uint32_t) )	return -2;
			} // <--- info header


			{ // ---> set data
				unsigned char d;	// dummy
				gm->allocate(iheader.biHeight, iheader.biWidth);
				gm->pset_rsl( 1.0 / iheader.biXPixPerMeter, 1.0 / iheader.biYPixPerMeter);

				pad = (pixlsize * iheader.biWidth) % 4;

				errno = 0;
				if( ::lseek(fd, fheader.bfOffBits, SEEK_SET) < 0) {
					return -2;
				}


				// read image data
				for( size_t h = 0; h < (unsigned)iheader.biHeight; h++ ){
					if( ::read(fd, gm->pointer(h, 0), iheader.biWidth * pixlsize) < (signed) (iheader.biWidth * pixlsize))	return -2;

					for( size_t i = 0; i < pad; i++ ){
						if( ::read(fd, &d, 1) < 1)	return -2;
					}
				}


			} // <--- set data

			{ // ---> finalize
				close(fd);
			} // <--- finalize

			return 0;
		}



		/**
		 * @ingroup GNDBmp
		 * @brief write gray bit map file
		 * @param[in] fname : file name
		 * @param[in]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int write32( const char *fname, bmp32_t *gm)
		{
			FILEHEADER fheader;
			INFOHEADER iheader;
			const size_t pixlsize = 4;
			int linesize, imagesize;
			struct {
				double x;
				double y;
			} org;
			size_t pad;


			{ // ---> initialize
				linesize = pixlsize * gm->column();
				pad = (linesize % 4);
				linesize += pad ? 4 - pad : 0;
				imagesize = linesize * gm->row();
				gm->pget_origin(&org.x, &org.y);
			} // <--- initialize


			{ // ---> file header
				::memset(&fheader, 0, sizeof(fheader));

				::memcpy(&fheader.bfType, FILETYPE, sizeof(fheader.bfType));

				fheader.bfSize = FILEHEADER_SIZE + INFOHEADER_SIZE + imagesize;

				fheader.bfReserved1 = 0;
				fheader.bfReserved2 = 0;
				fheader.bfOffBits = FILEHEADER_SIZE + INFOHEADER_SIZE;
			} // <--- file header


			{ // ---> info header
				// add origin data
				iheader.biSize = INFOHEADER_SIZE;
				iheader.biWidth = gm->column();
				iheader.biHeight = gm->row();
				iheader.biPlanes = 1;
				iheader.biBitCount = 8 * pixlsize;
				iheader.biCompression = 0;
				iheader.biSizeImage = imagesize;
				iheader.biXPixPerMeter = ::round( 1.0 / gm->xrsl() );
				iheader.biYPixPerMeter = ::round( 1.0 / gm->yrsl() );
				iheader.biClrUsed      = 0;
				iheader.biClrImporant  = 0;

			} // <--- info header



			{ // ---> file out
				int fd;
				const unsigned char d = 0x00;	// dummy

				fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT | gnd::wO_BINARY,
						gnd::wS_IRUSR | gnd::wS_IWUSR |
						gnd::wS_IRGRP | gnd::wS_IWGRP );
				if(fd < 0) return -1;

				{ // ---> write file header
					if( ::write(fd, &fheader.bfType, sizeof(unsigned short))		< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfSize, sizeof(uint32_t))				< (signed) sizeof(uint32_t) )		return -2;
					if( ::write(fd, &fheader.bfReserved1, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfReserved2, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &fheader.bfOffBits, sizeof(uint32_t))			< (signed) sizeof(uint32_t) )		return -2;
				} // <--- write file header

				{ // ---> write info header
					if( ::write(fd, &iheader.biSize, sizeof(uint32_t) )				< (signed) sizeof(uint32_t) )		return -2;
					if( ::write(fd, &iheader.biWidth, sizeof(int32_t) )					< (signed) sizeof(int) )			return -2;
					if( ::write(fd, &iheader.biHeight, sizeof(int32_t) )				< (signed) sizeof(int) )			return -2;
					if( ::write(fd, &iheader.biPlanes, sizeof(unsigned short) )		< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &iheader.biBitCount, sizeof(unsigned short) )	< (signed) sizeof(unsigned short) )	return -2;
					if( ::write(fd, &iheader.biCompression, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )		return -2;
					if( ::write(fd, &iheader.biSizeImage, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )		return -2;
					if( ::write(fd, &iheader.biXPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int) )			return -2;
					if( ::write(fd, &iheader.biYPixPerMeter, sizeof(int32_t) )			< (signed) sizeof(int) )			return -2;
					if( ::write(fd, &iheader.biClrUsed, sizeof(uint32_t) )			< (signed) sizeof(uint32_t) )		return -2;
					if( ::write(fd, &iheader.biClrImporant, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )		return -2;
				} // <--- write info header

				// write image data
				for( size_t h = 0; h < (unsigned)iheader.biHeight; h++ ){
					for( size_t w = 0; w < (unsigned)iheader.biWidth; w += gm->_unit_column_() ){
						if( ::write(fd, gm->pointer(h, w), gm->_unit_column_() * sizeof(uint32_t)) <
								((signed)gm->_unit_column_() * (signed)sizeof(uint32_t)) )	return -1;
					}
					for( size_t i = 0; i < pad; i++ ){
						if( ::write(fd, &d, 1) < 1)	return -1;
					}
				}

				//		if( ::write(fd, &org, sizeof(org)) < (signed) sizeof(org))	return -1;

				close(fd);
			} // <--- file out

			return 0;
		}



		/**
		 * @ingroup GNDBmp
		 * @brief read gray bit map file
		 * @param[in]  fname : file name
		 * @param[out]    gm : grid map data
		 * @return  ==0 : success
		 *          < 0 : failure
		 */
		inline
		int read32( const char *fname, bmp32_t *gm)
		{
			int fd;
			FILEHEADER fheader;
			INFOHEADER iheader;
			const size_t pixlsize = 8;
			size_t pad;


			{ // ---> initialize
				fd = open(fname, O_RDWR | gnd::wO_BINARY);
				if(fd < 0) return -1;
			} // <--- initialize


			{ // ---> file header
				::memset(&fheader, 0, sizeof(fheader));
				if( ::read(fd, &fheader.bfType,      sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfSize,      sizeof(uint32_t))			< (signed) sizeof(uint32_t) )		return -2;
				if( ::read(fd, &fheader.bfReserved1, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfReserved2, sizeof(unsigned short))	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &fheader.bfOffBits,   sizeof(uint32_t))			< (signed) sizeof(uint32_t) )		return -2;
			} // <--- file header

			{ // ---> info header
				::memset(&iheader, 0, sizeof(iheader));
				if( ::read(fd, &iheader.biSize, sizeof(uint32_t) )				< (signed) sizeof(uint32_t) )		return -2;
				if( ::read(fd, &iheader.biWidth, sizeof(int32_t) )				< (signed) sizeof(int32_t) )		return -2;
				if( ::read(fd, &iheader.biHeight, sizeof(int32_t) )				< (signed) sizeof(int32_t) )		return -2;
				if( ::read(fd, &iheader.biPlanes, sizeof(unsigned short) )		< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &iheader.biBitCount, sizeof(unsigned short) )	< (signed) sizeof(unsigned short) )	return -2;
				if( ::read(fd, &iheader.biCompression, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )		return -2;
				if( ::read(fd, &iheader.biSizeImage, sizeof(uint32_t) )			< (signed) sizeof(uint32_t) )		return -2;
				if( ::read(fd, &iheader.biXPixPerMeter, sizeof(int32_t) )		< (signed) sizeof(int32_t) )		return -2;
				if( ::read(fd, &iheader.biYPixPerMeter, sizeof(int32_t) )		< (signed) sizeof(int32_t) )		return -2;
				if( ::read(fd, &iheader.biClrUsed, sizeof(uint32_t) )			< (signed) sizeof(uint32_t) )		return -2;
				if( ::read(fd, &iheader.biClrImporant, sizeof(uint32_t) )		< (signed) sizeof(uint32_t) )		return -2;
			} // <--- info header


			{ // ---> set data
				unsigned char d;	// dummy
				gm->allocate(iheader.biHeight, iheader.biWidth);
				gm->pset_rsl( 1.0 / iheader.biXPixPerMeter, 1.0 / iheader.biYPixPerMeter);

				pad = (pixlsize * iheader.biWidth) % 4;

				errno = 0;
				if( ::lseek(fd, fheader.bfOffBits, SEEK_SET) < 0) {
					return -2;
				}


				// read image data
				for( size_t h = 0; h < (unsigned)iheader.biHeight; h++ ){
					if( ::read(fd, gm->pointer(h, 0), iheader.biWidth * pixlsize) < (signed) (iheader.biWidth * pixlsize))	return -2;

					for( size_t i = 0; i < pad; i++ ){
						if( ::read(fd, &d, 1) < 1)	return -2;
					}
				}


			} // <--- set data

			{ // ---> finalize
				close(fd);
			} // <--- finalize

			return 0;
		}


	}
}
// <--- function definition


#endif /* GND_BMP_HPP_ */
