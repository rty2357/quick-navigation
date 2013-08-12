/*
 * opsm-position-tracker-opt.hpp
 *
 *  Created on: 2011/12/22
 *      Author: tyamada
 */

#ifndef OPSM_POSITION_TRACKER_OPT_HPP_
#define OPSM_POSITION_TRACKER_OPT_HPP_


#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <ssmtype/spur-odometry.h>
#include <ssmtype/pws-motor.h>

#include "opsm-position-tracker-conf.hpp"

#ifndef opsm_pt
#define opsm_pt opsm::position_tracker
#endif

namespace opsm {
	namespace position_tracker {

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
			options(): pconf(0) { init(); }
			options(proc_configuration *p) : pconf(p) { init(); }
			~options(){}
		private:
			void init();
		public:
			proc_configuration *pconf;

		public:
			int set(proc_configuration *p){
				pconf = p;
				return 0;
			}
			// <--- constructor

			// ---> operation
		public:
			bool get_option(int aArgc, char **aArgv);
			// <--- operation
		};


		const char ShortOpt[] = "hg:G::m:S:O:d:c:lN";

		const struct option LongOpt[] = {
				{"help", 							no_argument,		0,	'h'},
				{"config",							required_argument,	0,	'g'},
				{"write-config",					required_argument,	0,	'G'},
				{ConfIni_ScanMatchingMapDir.item,	required_argument,	0,	'm'},
				{ConfIni_LaserScannerSSMName.item,	required_argument,	0,	'S'},
				{ConfIni_OdometrySSMName.item,		required_argument,	0,	'O'},
				{ConfIni_Culling.item,				required_argument,	0,	'd'},
				{ConfIni_Cycle.item,				required_argument,	0,	'c'},
				{ConfIni_SLAM.item,					no_argument,		0,	'l'},
				{ConfIni_NDT.item,					no_argument,		0,	'N'},
				{0, 0, 0, 0}	// end of array
		};



		inline void options::init()
		{
		}

		inline bool options::get_option(int aArgc, char **aArgv)
		{
			gnd_assert(!pconf, -1, "parameter storage is null.");

			while(1){
				int opt;
				optarg = 0;
				opt = ::getopt_long(aArgc, aArgv, ShortOpt, LongOpt, 0);
				if(opt < 0)	break;

				switch(opt){

				// read configure
				case 'g':
				{
					if( proc_conf_read(optarg, pconf) < 0 ){
						::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, Fail to read configure file\n");
						return RFail;
					}
				} break;

				// write configure
				case 'G': {
					proc_conf_write( optarg ? optarg : "opsm-position-tracker.conf", pconf);
					::fprintf(stderr, " ... output configuration file \"\x1b[4m%s\x1b[0m\"\n", optarg ? optarg : "opsm-position-tracker.conf");
				} return RWriteConf;

				// entry map file directory
				case 'm': ::strcpy(pconf->init_opsm_map.value, optarg);			break;
				// entry sokuiki ssm-data id
				case 'S': ::strcpy( pconf->ls_name.value, optarg);		break;
				// entry sokuiki ssm-data id
				case 'O': ::strcpy( pconf->odm_name.value, optarg);		break;

				// data decimate threshold
				case 'd': pconf->culling.value = ::atoi(optarg);			break;


				case 'c':
				{
					pconf->cycle.value = ::atof(optarg);
					if(pconf->cycle.value == 0)	{
						::fprintf(stderr, "[\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: option -c argument is invalid. requiared not 0\n");
						return RFail;
					}
					break;
				}
				case 'l':
					if(optarg){
						if( ::strlen(optarg) == ::strlen("true") && ::strncmp(optarg, "true", ::strlen("true")) == 0 )
							pconf->map_update.value = true;
						else if( ::strlen(optarg) == ::strlen("false") && ::strncmp(optarg, "false", ::strlen("false")) == 0 )
							pconf->map_update.value = false;
						else if( ::strlen(optarg) == ::strlen("on") && ::strncmp(optarg, "on", ::strlen("on")) == 0 )
							pconf->map_update.value = true;
						else if( ::strlen(optarg) == ::strlen("off") && ::strncmp(optarg, "off", ::strlen("off")) == 0 )
							pconf->map_update.value = false;
						else {
							pconf->map_update.value = ::atoi(optarg);
						}
					}
				break;

				// NDT mode
				case 'N':
					if(optarg){
						if( ::strlen(optarg) == ::strlen("true") && ::strncmp(optarg, "true", ::strlen("true")) == 0 )
							pconf->ndt.value = true;
						else if( ::strlen(optarg) == ::strlen("false") && ::strncmp(optarg, "false", ::strlen("false")) == 0 )
							pconf->ndt.value = false;
						else if( ::strlen(optarg) == ::strlen("on") && ::strncmp(optarg, "on", ::strlen("on")) == 0 )
							pconf->ndt.value = true;
						else if( ::strlen(optarg) == ::strlen("off") && ::strncmp(optarg, "off", ::strlen("off")) == 0 )
							pconf->ndt.value = false;
						else {
							pconf->ndt.value = ::atoi(optarg);
						}
					}
					break;

				// show help
				case 'h':
				{
					int i = 0;
					fprintf(stderr, "\t\x1b[1mNAME\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m - position tracking using probabilistic scan matching\n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mSYNAPSIS\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m [\x1b[4mOPTIONS\x1b[0m]\n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mDISCRIPTION\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m estimate the robot position and optimize it with probabilistic scan matching.\n", proc_name);

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

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tinput directory containing map files.\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tchenge tracking option\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tscan matching with Normal Distribution Transform algorithm.\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset newton optimization\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset quasi monte calro optimization\n");
					fprintf(stderr, "\n");
					i++;

					return RHelp;
				}break;
				}
			}
			return 0;
		}
	}; // namespace Optimizer
}; // namespace opsm



#endif /* OPSM_POSITION_TRACKER_OPT_HPP_ */
