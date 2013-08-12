/*
 * yp_lib-error.h
 *
 *  Created on: 2011/06/10
 *      Author: tyamada
 */

#ifndef GND_LIB_ERROR_H_
#define GND_LIB_ERROR_H_

#include <stdlib.h>
#include <stdio.h>

#ifdef __cpulspuls
extern "C" {
#endif


#ifdef NDEBUG
#define gnd_assert(pred, retval, msg) 			\
	do{											\
		if(pred){ return (retval); }			\
	} while(0)
#else
#define gnd_assert(pred, retval, msg)									\
	do {																	\
		if(pred){ 															\
			fprintf(stderr, "\x1b[1m\x1b[31mAbort\x1b[30m\x1b[0m: %s\n", msg);							\
			fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);	\
			abort();														\
			return (retval); 												\
		}																	\
	}while(0)
#endif


#ifdef NDEBUG
#define gnd_assert_void(pred, msg) 											\
	do{																		\
		if(pred){ return; }													\
	} while(0)
#else
#define gnd_assert_void(pred, msg)											\
	do {																	\
		if(pred){ 															\
			fprintf(stderr, "\x1b[1m\x1b[31mAbort\x1b[30m\x1b[0m: %s\n", msg);					\
			fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);	\
			abort();														\
			return;			 												\
		}																	\
	}while(0)
#endif


#ifdef NDEBUG
#define gnd_error(pred, retval, msg) 			\
	do{											\
		if(pred){ return (retval); }			\
	} while(0)
#else
#define gnd_error(pred, retval, msg)											\
	do {																	\
		if(pred){ 															\
			fprintf(stderr, "\x1b[1m\x1b[31mError\x1b[30m\x1b[0m: %s\n", msg);							\
			fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);	\
			return (retval); 												\
		}																	\
	}while(0)
#endif



#ifdef NDEBUG
#define gnd_error_void(pred, msg) 			\
	do{										\
		if(pred){ return ; }				\
	} while(0)
#else
#define gnd_error_void(pred, msg)											\
	do {																	\
		if(pred){ 															\
			fprintf(stderr, "\x1b[1m\x1b[31mError\x1b[30m\x1b[0m: %s\n", msg);							\
			fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);	\
			return ; 														\
		}																	\
	}while(0)
#endif


#ifdef NDEBUG
#define gnd_exit(retval, msg) 		\
	do{								\
		return (retval);			\
	} while(0)
#else
#define gnd_exit(retval, msg)												\
	do {																	\
		fprintf(stderr, "\x1b[1m\x1b[31mError\x1b[30m\x1b[0m: %s\n", msg);	\
		fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);		\
		return (retval); 													\
	}while(0)
#endif

#ifdef NDEBUG
#define gnd_exit_void(msg) 			\
	do{								\
		return;						\
	} while(0)
#else
#define gnd_exit_void(msg)													\
	do {																	\
		fprintf(stderr, "\x1b[1m\x1b[31mError\x1b[30m\x1b[0m: %s\n", msg);	\
		fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);		\
		return ; 															\
	}while(0)
#endif


#ifdef NDEBUG
#define gnd_warnning(pred, msg)
#else
#define gnd_warnning(pred, msg)												\
	do {																	\
		if(pred){ 															\
			fprintf(stderr, "\x1b[1m\x1b[31mWarning\x1b[30m\x1b[0m: %s\n", msg);							\
			fprintf(stderr, "%s:%d :%s\n", __FILE__, __LINE__, __func__);	\
		}																	\
	}while(0)
#endif


#include <sys/time.h>

#ifdef NDEBUG
#define gnd_test_timer(code, msg)		code
#else
#define gnd_test_timer(code, msg)											\
	do {																	\
		struct timeval st, end, tm;											\
		gettimeofday(&st, 0);												\
		code ;																\
		gettimeofday(&end, 0);												\
		timersub(&end, &st, &tm);											\
		fprintf(stderr, "%s %ld.%06ld\n", msg, tm.tv_sec, tm.tv_usec);		\
	}while(0)
#endif


#endif /* YP_LIB_ERROR_H_ */
