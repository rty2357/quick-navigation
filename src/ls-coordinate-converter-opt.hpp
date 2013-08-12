/*
 * ls-coordinate-converter-conf.hpp
 *
 *  Created on: 2012/01/27
 *      Author: tyamada
 */

#ifndef LS_COORDINATE_CONVERTER_OPT_HPP_
#define LS_COORDINATE_CONVERTER_OPT_HPP_

#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include "ls-coordinate-converter-conf.hpp"

namespace gnd {
	namespace ls_cc {
		class options
		{
		// ---> return value define
		public:
			static const int RFail = -1;		///<! return value Failure
			static const int RHelp = 1;			///<! return value Help
			static const int RWriteConf = 2;	///<! return value Write Configure File
		// <--- return value define

		// ---> constructor
		public:
			options(): param(0) { init(); }
			options(proc_configuration *p) : param(p) { init(); }
			~options(){}
		private:
			void init();
		public:
			proc_configuration *param;

		public:
			int set(proc_configuration *p){
				param = p;
				return 0;
			}
			// <--- constructor

			// ---> operation
		public:
			bool get_option(int aArgc, char **aArgv);
			// <--- operation
		};


		const char proc_name[] = "ls-coordinate-converter";


		const char ShortOpt[] = "hg:G::D::";

		const struct option LongOpt[] = {
				{"help", 						no_argument,		0,	'h'},
				{"config",						required_argument,	0,	'g'},
				{"write-config",				optional_argument,	0,	'G'},
				{"write-dev-conf",				optional_argument,	0,	'D'},
				{0, 0, 0, 0}	// end of array
		};



		inline void options::init()
		{
		}

		inline bool options::get_option(int aArgc, char **aArgv)
		{
			gnd_assert(!param, -1, "parameter storage is null.");

			while(1){
				int opt;
				optarg = 0;
				opt = ::getopt_long(aArgc, aArgv, ShortOpt, LongOpt, 0);
				if(opt < 0)	break;

				switch(opt){

				// read configure
				case 'g':
				{
					gnd::conf::file_stream conf_fs;
					if( conf_fs.read(optarg) < 0 ){
						::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, Fail to read configure file\n");
						return RFail;
					}
					if( proc_conf_get(&conf_fs, param) < 0){
						::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, configure file syntax error\n");
						return RFail;
					}
				} break;

				// write configure
				case 'G': {
					proc_conf_write( optarg ? optarg : "ls-coordinate-converter.conf", param);
					::fprintf(stdout, " ... output configuration file \"\x1b[4m%s\x1b[0m\"\n", optarg ? optarg : "ls-coordinate-converter.conf");
				} return RWriteConf;
				// write configure

				// show help
				case 'h':
				{
					int i = 0;
					fprintf(stderr, "\t\x1b[1mNAME\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m - laser scan coordinate convert \n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mSYNAPSIS\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m [\x1b[4mOPTIONS\x1b[0m]\n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mDISCRIPTION\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m convert the coordinate of laser scan.\n", proc_name);

					fprintf(stderr, "\n");
					fprintf(stderr, "\t\x1b[1mOPTIONS\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tprint help\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val,  LongOpt[i].name);
					fprintf(stderr, "\t\t\tread configure file\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val,  LongOpt[i].name);
					fprintf(stderr, "\t\t\twirte configure file\n");
					fprintf(stderr, "\n");
					i++;

					return RHelp;
				}break;
				}
			}
			return 0;
		}
	} // namespace ls
}; // namespace gnd

#endif /* LS_COORDINATE_CONVERTER_OPT_HPP_ */
