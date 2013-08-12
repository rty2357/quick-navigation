/*
 * opsm-particle-evaluator-opt.hpp
 *
 *  Created on: 2012/03/14
 *      Author: tyamada
 */

#ifndef OPSM_PARTICLE_EVALUATOR_OPT_HPP_
#define OPSM_PARTICLE_EVALUATOR_OPT_HPP_

#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include "opsm-particle-evaluator.hpp"

#include "opsm-particle-evaluator-conf.hpp"

// type declaration
// ---> namespace opsm
namespace opsm {
	// ---> namespace Particle Evaluator
	namespace peval {
		class proc_option_reader;

	} // <--- namespace Particle Evaluator
 }// <--- namespace opsm


// constant declaration
// ---> namespace opsm
namespace opsm {
	// ---> namespace Particle Evaluator
	namespace peval {
		const char ConfFile[] = "opsm-particle-evaluator.conf";
		const char ShortOpt[] = "hg:G::m:r:s:p:e:c:";
		const struct option LongOpt[] = {
				{"help", 							no_argument,		0,	'h'},
				{"config",							required_argument,	0,	'g'},
				{"write-config",					optional_argument,	0,	'G'},
				{ConfIni_BMPMap.item,				required_argument,	0,	'm'},
				{ConfIni_RawMap.item,				required_argument,	0,	'r'},
				{ConfIni_SokuikiRawID.item,			required_argument,	0,	's'},
				{ConfIni_ParticleID.item,			required_argument,	0,	'p'},
				{ConfIni_ParticleEvalID.item,		required_argument,	0,	'e'},
				{ConfIni_Cycle.item,				required_argument,	0,	'c'},
				{0, 0, 0, 0}	// end of array
		};

	} // <--- namespace Particle Evaluator
 }// <--- namespace opsm



// type definition
// ---> namespace opsm
namespace opsm {
	// ---> namespace Particle Evaluator
	namespace peval {
		class proc_option_reader {
		// ---> return value definition
		public:
			static const int RetFail = -1;
			static const int RetHelp = 1;
			static const int RetWriteConf = 2;

		public:

		// ---> constructor
		public:
			proc_option_reader();
			proc_option_reader(proc_configuration *c);
			~proc_option_reader();
		private:
			proc_configuration *conf;

		// set storage
		public:
			int set(proc_configuration *c);

		// read
		public:
			bool read(int argc, char **argv);

		};

		/**
		 * @brief constructor
		 */
		inline
		proc_option_reader::proc_option_reader(): conf(0) {
		}

		/**
		 * @brief constructor
		 */
		inline
		proc_option_reader::proc_option_reader(proc_configuration *c): conf(c) {
		}

		/**
		 * @brief destructor
		 */
		inline
		proc_option_reader::~proc_option_reader() {
		}

		/**
		 * @brief set configuration parameter storage
		 */
		inline
		int proc_option_reader::set(proc_configuration *c) {
			conf = c;
			return 0;
		}


		/**
		 * @brief read option
		 */
		inline
		bool proc_option_reader::read(int argc, char **argv)
		{
			gnd_error(!conf, RetFail, "Invalid Member");

			while(1){
				int opt;
				optarg = 0;
				opt = ::getopt_long(argc, argv, ShortOpt, LongOpt, 0);
				if(opt < 0)	break;

				switch(opt){

				// read configure
				case 'g':
				{
					if( proc_conf_read(optarg, conf) < 0){
						::fprintf(stderr, " ... [\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: -g option, configure file syntax error\n");
						return RetFail;
					}
				} break;

				// write configure
				case 'G': {
					proc_conf_write( optarg ? optarg : ConfFile, conf);
					::fprintf(stderr, " ... output configuration file \"\x1b[4m%s\x1b[0m\"\n", optarg ? optarg : "opsm-particle-evaluator.conf");
				} return RetWriteConf;

				// entry map file
				case 'm': ::strcpy(conf->bmp_map.value, optarg);			break;
				// entry raw map file directory
				case 'r': ::strcpy( conf->raw_map.value, optarg);			break;

				// entry sokuiki ssm-data id
				case 's': conf->sokuikiraw_id.value = ::atoi(optarg);		break;
				// entry particle ssm-data id
				case 'p': conf->particle_id.value = ::atoi(optarg);			break;
				// entry particle evaluation ssm-date id
				case 'e': conf->eval_id.value = ::atoi(optarg);				break;
				// entry proc cycle
				case 'c':
				{
					conf->cycle.value = ::atof(optarg);
					if(conf->cycle.value == 0)	{
						::fprintf(stderr, "[\x1b[1m\x1b[31mERROR\x1b[30m\x1b[0m]: option -c argument is invalid. requiared not 0\n");
						return RetFail;
					}
					break;
				}

				// show help
				case 'h':
				{
					int i = 0;
					fprintf(stderr, "\t\x1b[1mNAME\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m - particle evaluation using probabilistic scan matching\n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mSYNAPSIS\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m [\x1b[4mOPTIONS\x1b[0m]\n", proc_name);
					fprintf(stderr, "\n");

					fprintf(stderr, "\t\x1b[1mDISCRIPTION\x1b[0m\n");
					fprintf(stderr, "\t\t\x1b[1m%s\x1b[0m evaluate particles for particle filter localization using probabilistic scan matching.\n", proc_name);

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
					fprintf(stderr, "\t\t\tinput map files.\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tinput directory containing map files.\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset sokuiki raw ssm-data id\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset particle ssm-data id\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset particle evaluation ssm-data id\n");
					fprintf(stderr, "\n");
					i++;

					fprintf(stderr, "\t\t\x1b[1m-%c\x1b[0m, \x1b[1m--%s\x1b[0m\n", LongOpt[i].val, LongOpt[i].name);
					fprintf(stderr, "\t\t\tset operation cycle\n");
					fprintf(stderr, "\n");
					i++;

					return RetHelp;
				}break;
				}
			}
			return 0;
		}


	} // <--- namespace Particle Evaluator
 }// <--- namespace opsm



#endif /* OPSM_PARTICLE_EVALUATOR_OPT_HPP_ */
