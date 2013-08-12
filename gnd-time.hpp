/*
 * gnd-time.hpp
 *
 *  Created on: 2012/01/29
 *      Author: tyamada
 */

#ifndef GND_TIME_HPP_
#define GND_TIME_HPP_

#include <time.h>
#include <errno.h>
#include <limits.h>
#include "gnd-lib-error.h"
#include "gnd-util.h"


/**
 * @ifnot GNDTIME
 * @defgroup GNDTIME time
 * supply timer
 * this module need to link librt, so you link with -lrt
 * @endif
 */

// ---> interval timer
namespace gnd {
	namespace timer {
		/**
		 * @brief interval timer
		 */
		class interval_timer {
			// ---> constructor, destructor
		public:
			interval_timer();
			interval_timer(clockid_t id, double c);
			~interval_timer();
			// <--- constructor, destructor

			// ---> time
		private:
			struct {
				unsigned long cnt;
				unsigned long loop;		///<! next time
				struct timespec cycle;		///<! cycle
				struct timespec next;		///<! next time
			} var;
			// <--- time

			// ---> mode
		private:
			struct {
				clockid_t clock;		///<! clock
				// todo make nonblocking mode
				char block;				///<! blocking mode
			} property;
			// <--- time

		public:
			virtual int begin(clockid_t id, double c, double o = 0);
			virtual int begin(clockid_t id, struct timespec* c, double o = 0);
			virtual int wait();
			virtual int clock(double *l = 0);
			virtual int end();
			virtual double cycle();
		};


		/**
		 * @brief constructor
		 */
		inline
		interval_timer::interval_timer() {
			end();
		}

		/**
		 * @brief constructor
		 */
		inline
		interval_timer::interval_timer(clockid_t id, double c) {
			begin(id, c);
		}

		/**
		 * @brief destructor
		 */
		inline
		interval_timer::~interval_timer() {
		}

		/**
		 * @brief begin
		 * @param[in] id: clock
		 * @param[in]  c: cycle
		 * @param[in]  o: offset
		 */
		inline
		int interval_timer::begin(clockid_t id, double c, double o) {
			gnd_error(c < 0, -1, "invalid argument");
			{ // ---> operation
				int ret = 0;
				double next;
				property.clock = id;
				var.cnt = 0;
				var.loop = 0;
				gnd_time2timespec(&var.cycle, c);

				if( (ret = ::clock_gettime(property.clock, &var.next) ) < 0 )	return ret;
				next = gnd_timespec2time(&var.next);
				next += c + o;
				gnd_time2timespec(&var.next, next);
			} // <--- operation
			return 0;
		}

		/**
		 * @brief begin
		 * @param[in] id: clock
		 * @param[in]  c: cycle
		 */
		inline
		int interval_timer::begin(clockid_t id, struct timespec* c, double o) {
			double cc = gnd_timespec2time(c);

			return begin(id, cc, o);
		}


		/**
		 * @brief wait (block)
		 */
		inline
		int interval_timer::wait() {
			if( var.cycle.tv_sec == 0 && var.cycle.tv_nsec == 0 )	return -1;

			{ // ---> wait
				int sig;
				for(sig = clock_nanosleep(property.clock, TIMER_ABSTIME, &var.next, 0);
						sig != 0 && errno != EINTR;
						sig = clock_nanosleep(property.clock, TIMER_ABSTIME, &var.next, 0));
			} // <--- wait

			{ // ---> set next
				int cnt = 1;
				struct timespec cur, next;
				double sub_d;
				::clock_gettime(property.clock, &cur);
				gnd_timespec_add(&var.next, &var.cycle, &next);
				sub_d = gnd_timespec2time(&next) - gnd_timespec2time(&cur);

				// check exception
				if(sub_d < 0){
					// ---> exception by tardiness
					cnt = ::ceil( sub_d / gnd_timespec2time(&var.cycle) );
				}
				var.cnt += cnt;

				var.next = next;
				if( var.cnt > LONG_MAX ) {
					var.loop++;
					var.cnt -= LONG_MAX;
				}
				return cnt;
			} // <--- set next
		}


		/**
		 * @brief check timer
		 * @param [o] l : left clock
		 * @return == 0 : not come in
		 *          < 1 : come in
		 */
		inline
		int interval_timer::clock(double *l) {
			if( var.cycle.tv_sec == 0 && var.cycle.tv_nsec == 0 )	return -1;

			{ // ---> operation
				int cnt = 0;
				struct timespec cur, next;
				double sub_d;
				::clock_gettime(property.clock, &cur);

				if( gnd_timespec_comp(&cur, &var.next) >= 0 ) {
					gnd_timespec_add(&var.next, &var.cycle, &next);
					sub_d = gnd_timespec2time(&next) - gnd_timespec2time(&cur);

					// check exception
					if(sub_d < 0){
						// ---> exception by tardiness
						cnt = ::ceil( -sub_d / gnd_timespec2time(&var.cycle) );
					}
					else {
						cnt = 1;
					}
					var.cnt += cnt;

					var.next = next;
					if( var.cnt > LONG_MAX ) {
						var.loop++;
						var.cnt -= LONG_MAX;
					}
				}
				// left
				if(l) *l = gnd_timespec2time(&var.next) - gnd_timespec2time(&cur);
				return cnt;
			} // <--- operation
		}


		/**
		 * @brief get interval timer cycle
		 */
		inline
		double interval_timer::cycle() {
			return gnd_timespec2time(&var.cycle);
		}

		/**
		 * @brief destroy timer
		 */
		inline
		int interval_timer::end() {
			var.cnt = 0;
			var.loop = 0;
			gnd_timespec_zero(&var.cycle);
			gnd_timespec_zero(&var.next);

			return 0;
		}

	} // <--- namespace time
	typedef timer::interval_timer inttimer;
} // <--- namespace gnd
// <--- interval timer



// ---> stopwatch
namespace gnd {
	namespace timer {

		/**
		 * @brief stopwatch
		 */
		class stopwatch {
			// ---> constructor, destructor
		public:
			stopwatch();
			~stopwatch();
			// <--- constructor, destructor

			// ---> time
		private:
			struct {
				struct timespec beginning;		///<! beginning time
				struct timespec prev_rec;	///<! prev_record
			} var;
			// <--- time

			// ---> mode
		private:
			struct {
				clockid_t clock;		///<! clock
			} property;
			// <--- time

		public:
			virtual int begin( clockid_t id, struct timespec *rec);
			virtual int begin( clockid_t id, double *rec);
			virtual int begin( clockid_t id );
			virtual int get( double *t, double *lap = 0 );
			virtual int end( );
		};


		/**
		 * @brief constructor
		 */
		inline
		stopwatch::stopwatch() {
			end();
		}

		/**
		 * @brief destructor
		 */
		inline
		stopwatch::~stopwatch() {
		}

		/**
		 * @brief begin
		 * @param[in] id: clock
		 * @param[in]  c: time
		 */
		inline
		int stopwatch::begin( clockid_t id, struct timespec *rec) {
			int ret = 0;

			property.clock = id;
			if( (ret = ::clock_gettime(property.clock, &var.beginning) ) < 0 )	return ret;
			::memcpy(&var.prev_rec, &var.beginning, sizeof(var.prev_rec));

			if( rec ) ::memcpy(rec, &var.beginning, sizeof(struct timespec));

			return 0;
		}

		/**
		 * @brief begin
		 * @param[in]  id: clock
		 * @param[in] rec: time
		 */
		inline
		int stopwatch::begin( clockid_t id, double *rec) {
			struct timespec r;
			int ret;
			ret = begin( id, &r );
			*rec = gnd_timespec2time( &r );
			return ret;
		}


		/**
		 * @brief begin
		 * @param[in] id: clock
		 */
		inline
		int stopwatch::begin( clockid_t id ) {
			struct timespec *p = 0;
			return begin( id, p );
		}


		/**
		 * @brief wait (block)
		 * @param [out] t : time from timer beginning
		 * @param [out] l : time from previous record
		 */
		inline
		int stopwatch::get( double *t, double *l ) {

			{
				int ret = 0;
				struct timespec ws;

				if( (ret = ::clock_gettime(property.clock, &ws) ) < 0 )	return ret;
				if( t ) *t = gnd_timespec2time( &ws ) - gnd_timespec2time( &var.beginning );
				if( l ) *l = gnd_timespec2time( &ws ) - gnd_timespec2time( &var.prev_rec );
				::memcpy(&ws, &var.prev_rec, sizeof(ws));
			}

			return 0;
		}



		/**
		 * @brief destroy timer
		 */
		inline
		int stopwatch::end() {
			gnd_timespec_zero(&var.beginning);
			gnd_timespec_zero(&var.prev_rec);

			return 0;
		}

	} // <--- namespace time
	typedef timer::stopwatch stopwatch;
} // <--- namespace gnd
// <--- interval timer



#endif /* GND_TIME_HPP_ */
