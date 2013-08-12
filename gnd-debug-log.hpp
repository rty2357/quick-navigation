/*
 * gnd-debug.hpp
 *
 *  Created on: 2011/12/08
 *      Author: tyamada
 */

#ifndef GND_DEBUG_LOG
#define GND_DEBUG_LOG

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>


/**
 * @if GNDDebug
 * @defgroup GNDDebug gnd-debug
 * supply logging, error-wornning message and assertion
 * @endif GndDebug
 */

/**
 * @if GNDDebugLog
 * @defgroup GNDDebugLog logging
 * @endif
 */

// ---> log indent
#ifndef GND_DEBUGLOG_INDENT
#define GND_DEBUGLOG_INDENT
int	__g_GND_DEBUGLOG_INDENT_LEVEL__ = 0;
static const int __g_GND_DEBUGLOG_INDENTSPACE__ = 2;
static const int __g_GND_DEBUGLOG_INDENTMAX__ = 8;

int _debug_indent_();
int _debug_unindent_();

/**
 * @ingroup GNDDebugLog
 * @brief log indent
 * @retval indent level
 */
inline
int _debug_indent_() {
	return __g_GND_DEBUGLOG_INDENT_LEVEL__ = __g_GND_DEBUGLOG_INDENT_LEVEL__ < __g_GND_DEBUGLOG_INDENTMAX__ ? __g_GND_DEBUGLOG_INDENT_LEVEL__ + 1 : __g_GND_DEBUGLOG_INDENTMAX__;
}

/**
 * @ingroup GNDDebugLog
 * @brief log unindent
 * @retval indent level
 */
inline
int _debug_unindent_() {
	return __g_GND_DEBUGLOG_INDENT_LEVEL__ = __g_GND_DEBUGLOG_INDENT_LEVEL__ > 0 ? __g_GND_DEBUGLOG_INDENT_LEVEL__ - 1 : 0;
}
#endif // ifndef GND_DEBUGLOG_INDENT
// <--- log indent





// ---> log level
#ifndef GND_DEBUGLOG_LEVEL
#define GND_DEBUGLOG_LEVEL

/**
 * @addtogroup GNDDebugLog
 * @{
 */
enum {
	Log0 = 0,		///< no logging
	LogVerbose,		///< vervose mode
	LogDebug,		///< Debug
};
/** @} */
#endif // ifndef GND_DEBUGLOG_LEVEL
// <--- log level


#ifdef GND_DEBUG_LOG_NAMESPACE1
namespace GND_DEBUG_LOG_NAMESPACE1 {
#endif
#ifdef GND_DEBUG_LOG_NAMESPACE2
namespace GND_DEBUG_LOG_NAMESPACE2 {
#endif
#ifdef GND_DEBUG_LOG_NAMESPACE3
namespace GND_DEBUG_LOG_NAMESPACE3 {
#endif

	void debug_set_log_level(int l);
	int  debug_set_fstream(const char *f);
	void debug_set_fstream(FILE *f);

	void _debug_log_(int lv, const char *f, ...);

	/**
	 * @ingroup GNDDebugLog
	 * @brief debug logger class
	 */
	class DebugLogger{
	private:
		DebugLogger();
	public:
		~DebugLogger();

	private:
		int l;		///< @brief debug-level
		FILE* f;	///< @brief log file stream

	public:
		static DebugLogger d;	///< logger

	public:
		friend void debug_set_log_level(int);
		friend int debug_set_fstream(const char *f);
		friend void debug_set_fstream(FILE *f);
		friend FILE* debug_get_fstream();
		friend void _debug_log_(int lv, const char *f, ...);
		friend int _debug_indent_();
		friend int _debug_unindent_();
	};
	DebugLogger DebugLogger::d;

	/**
	 * @brief constructor
	 */
	inline
	DebugLogger::DebugLogger(){
		l = Log0;
		f = stdout;
	}

	/**
	 * @brief destructor
	 */
	inline
	DebugLogger::~DebugLogger(){
		if( f != 0 && f != stdout && f != stderr ){
			fclose(f);
		}
	}

	/**
	 * @ingroup GNDDebugLog
	 * @brief set debug logging file stream
	 * @param [i] d: debug level (0 is no debug)
	 */
	inline
	void debug_set_log_level(int l) {
		DebugLogger::d.l = l;
	}


	/**
	 * @ingroup GNDDebugLog
	 * @brief set debug logging file stream
	 */
	inline
	int debug_set_fstream(const char *f)
	{
		FILE *fp;

		if( !(fp = fopen(f, "w"))){
			return -1;
		}
		DebugLogger::d.f = fp;
		return 0;
	}

	/**
	 * @ingroup GNDDebugLog
	 * @brief set debug logging file stream
	 */
	inline
	void debug_set_fstream(FILE *f)
	{
		DebugLogger::d.f = f;
	}



	/**
	 * @ingroup GNDDebugLog
	 * @brief get debug logging file stream
	 */
	inline
	FILE* debug_get_fstream(){
		return DebugLogger::d.f;
	}

	/**
	 * @ingroup GNDDebugLog
	 * @brief logging
	 * @param [in] : lv
	 */
	inline
	void _debug_log_(int lv, const char *f, ...){

		if( DebugLogger::d.l < lv ){
			return;
		}

		{ // ---> operation
			char form[512];
			char indent[__g_GND_DEBUGLOG_INDENTMAX__*__g_GND_DEBUGLOG_INDENTSPACE__ + 1];
			struct timeval t;
			va_list args;
			va_start(args, f);

			gettimeofday(&t, 0);
			if( __g_GND_DEBUGLOG_INDENT_LEVEL__ )
				memset(indent, ' ', __g_GND_DEBUGLOG_INDENTSPACE__ * __g_GND_DEBUGLOG_INDENT_LEVEL__);
			indent[__g_GND_DEBUGLOG_INDENTSPACE__ * __g_GND_DEBUGLOG_INDENT_LEVEL__] = '\0';
			sprintf(form, "%ld.%06ld: %s%s", t.tv_sec, t.tv_usec, indent, f);
			vfprintf(DebugLogger::d.f, form, args);
			fflush(DebugLogger::d.f);
		} // <--- operation
	}

#ifdef GND_DEBUG_LOG_NAMESPACE3
}
#endif
#ifdef GND_DEBUG_LOG_NAMESPACE2
}
#endif
#ifdef GND_DEBUG_LOG_NAMESPACE1
}
#endif

#endif /* GND_DEBUG_LOG */
