/*
 * gnd-debug-log-util.h
 *
 *  Created on: 2012/02/05
 *      Author: tyamada
 */

#ifndef NDEBUG

#ifndef LogIndent
#define LogIndent()			_debug_indent_()
#endif

#ifndef LogUnindent
#define LogUnindent()		_debug_unindent_()
#endif


#ifndef LogDebugf
#define LogDebugf(f, ...)	_debug_log_(LogDebug, f, __VA_ARGS__)
#endif

#ifndef LogVerbosef
#define LogVerbosef(f, ...)	_debug_log_(LogVerbose, f, __VA_ARGS__)
#endif

#ifndef LogDebug
#define LogDebug(s)			_debug_log_(LogDebug, s)
#endif

#ifndef LogVerbose
#define LogVerbose(s)		_debug_log_(LogVerbose, s)
#endif

#else
#ifndef LogIndent
#define LogIndent()
#endif

#ifndef LogUnindent
#define LogUnindent()
#endif

#ifndef LogDebugf
#define LogDebugf(f, ...)
#endif

#ifndef LogVerbosef
#define LogVerbosef(f, ...)
#endif

#ifndef LogDebug
#define LogDebug(s)
#endif

#ifndef LogVerbose
#define LogVerbose(s)
#endif

#endif

