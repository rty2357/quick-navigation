/*
 * gnd_cui.hpp
 *
 *  Created on: 2011/08/10
 *      Author: tyamada
 */

#ifndef GND_CUI_HPP_
#define GND_CUI_HPP_

#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/unistd.h>

#include <errno.h>

#include "gnd-queue.hpp"

#define GND_DEBUG_LOG_NAMESPACE1 gnd
#define GND_DEBUG_LOG_NAMESPACE2 cui
#include "gnd-debug-log.hpp"
#undef GND_DEBUG_LOG_NAMESPACE2
#undef GND_DEBUG_LOG_NAMESPACE1
#undef GND_DEBUG_LOG

#include "gnd-debug-log-util-def.h"


/**
 * @ifnot GNDCUI
 * @defgroup GNDCUI cui
 * supply cui function: command reading, and analysis
 * @endif
 */

// ---> type declaration
namespace gnd {
	namespace cui {
		struct command ;
		typedef struct command command;
		class reader;
	}
	typedef cui::reader cui_reader;
	typedef cui::command cui_command;
}
// <--- type declaration



// ---> type definition
namespace gnd {
	namespace cui {
		/**
		 * @ingroup GNDCUI
		 * @brief command type
		 */
		struct command {
			/// @brief token
			char token[32];
			/// @brief return value
			int val;
			/// @brief command brief
			char brief[256];
		};
	}
}
// <--- type definition



// ---> class definition
namespace gnd { // ---> gnd
	namespace cui { // ---> CUI

		/**
		 * @ingroup GNDCUI
		 * @brief cui class
		 */
		class reader {
		public:
			reader();
			reader(const command *cmd, size_t n);
			~reader();

		private:
			/// @brief command list
			queue< command > _clist;

		public:
			int set_command(const  command *cmd, size_t n );

		public:
			int poll(int *v, char *a = 0, size_t n = 0, double t = 0);

		public:
			int show( FILE* f = 0, const char *h = 0);
		};



		/**
		 * @brief constructor
		 */
		inline
		reader::reader()
		{
		}



		/**
		 * @brief constructor
		 * @param[in] cmd : command list
		 * @param[in]   n : number of command
		 * @details input command list
		 */
		inline
		reader::reader(const command *cmd, size_t n)
		{
			set_command(cmd, n);
		}


		/**
		 * @brief destructor
		 */
		inline
		reader::~reader()
		{
		}


		/**
		 * @brief set comman
		 * @param[in] cmd : command list
		 * @param[in]   n : number of command list
		 */
		inline
		int reader::set_command(const command *cmd, size_t n)
		{
			{ // --->gnd_cui_cmdte
				_clist.clear();
				_clist.push_back(cmd, n);
			} // <--- allocate

			return 0;
		}


		/**
		 * @brief poll
		 * @param [out] v  : value
		 * @param [out] a  : argument buffer
		 * @param  [in] n  : argument buffer size
		 * @param  [in] t  : timeout (=0: nonblocking, <0: wait)
		 * @return > 0     : input
		 *         ==0     : timeout
		 *         < 0	   : error
		 */
		inline
		int reader::poll(int *v, char *a, size_t n, double t){
			int ret;
			fd_set fds;
			char buf[1024];
			struct timeval *top = 0;
			struct timeval to;
			char *p, *s;

			{ // ---> set file-descriptor of stdin
				FD_ZERO(&fds);
				FD_SET(0, &fds);
			} // <--- set file-descriptor of stdin

			// ---> set timeout
			if( t >= 0){
				to.tv_sec  = t;
				to.tv_usec = (t - to.tv_sec) * 1.0e+6;
				top = &to;
			}
			else {
				top = 0;
			}// <--- set timeout

			// polling
			if( (ret = ::select(1, &fds, 0, 0, top)) <= 0 )	return ret;

			// output zero clear
			*a = '\0';

			{ // ---> read from stdin
				::memset(buf, 0, sizeof(buf));
				if( (ret = ::read(0, buf, sizeof(buf) ) ) < 0 )	return -1;
			} // <--- read from stdin


			// ignore space of head
			for(s = buf;  *s != '\0' && ::isspace(*s); s++);

			// no argument error
			if(*s == '\0')	return ret;

			// ---> short argument
			if(s[1] == '\0' || ::isspace(s[1]) ) {
				*v = *s;
				if(a){
					for( p = s + 1; *p != '\0' && ::isspace(*p); p++);
					::strncpy(a, p, n);
				}
				return ret;
			} // <--- short argument

			// analyze long argument
			for( size_t i = 0; i < (unsigned)_clist.size() && _clist[i].token[0] != '\0' ; i++){
				int len = ::strlen(_clist[i].token);


				if( ::strncmp( _clist[i].token, s, len ) ) continue;
				if( s[len] != '\0' && !::isspace(s[len]) ) continue;

				*v = _clist[i].val;
				if(a){
					for( p = s + len; *p != '\0' && ::isspace(*p); p++);
					::strncpy(a, p, n);
				}
				break;
			}

			return ret;
		}



		/**
		 * @brief show command list
		 * @param[in] f: file stream
		 * @param[in] h: header
		 */
		inline
		int reader::show( FILE* f, const char *h )
		{
			FILE* fp = f ? f : stderr;
			size_t i;

			for( i = 0; i < (unsigned)_clist.size() && _clist[i].token[0] != '\0' ; i++){
				fprintf( fp, "%s\x1b[1m%s\x1b[0m, \x1b[1m%c\x1b[0m%s%s\n", h ? h : "",
						_clist[i].token,_clist[i].val,
						_clist[i].brief[0] ? " - " : "", _clist[i].brief);
			}
			return 0;
		}

	} // <--- CUI
} // <--- gnd
// <--- class definition



#include "gnd-debug-log-util-undef.h"

#endif /* GND_CUI_HPP_ */
