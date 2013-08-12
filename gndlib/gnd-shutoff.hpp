/*
 * gnd-shutoff.hpp
 *
 *  Created on: 2011/12/10
 *      Author: tyamada
 */

#ifndef GND_SHUTOFF_HPP_
#define GND_SHUTOFF_HPP_

#include <signal.h>
#include <string.h>

bool is_proc_shutoff(void);
void proc_shutoff(int s);
void proc_shutoff_clear(void);
int proc_shutoff_alloc_signal(int sig, int flags = SA_RESETHAND | SA_RESTART);
int proc_shutoff_ignore_signal(int sig, int flags = SA_RESTART);

class __proc_shutoff__{
// ---> constructor, destructor
private:
	__proc_shutoff__();
public:
	~__proc_shutoff__();
// <--- constructor, destructor

// ---> shut-off flag
private:
	bool shutoff;
// <--- shut-off flag

// ---> sealed
private:
	static __proc_shutoff__ s;
// <--- sealed

// ---> friend function
private:
	friend bool is_proc_shutoff(void);
	friend void proc_shutoff(int s = -1);
	friend void proc_shutoff_clear(void);
// ---> friend function

};
// declaration
__proc_shutoff__ __proc_shutoff__::s;

inline
__proc_shutoff__::__proc_shutoff__()
{
	shutoff = false;
}

inline
__proc_shutoff__::~__proc_shutoff__()
{
}

inline
bool is_proc_shutoff(void){
	return __proc_shutoff__::s.shutoff;
}

inline
void proc_shutoff(int s) {
	__proc_shutoff__::s.shutoff = true;
}

inline
void proc_sig_ignore(int s) {
	return;
}

inline
void proc_shutoff_clear( void ) {
	__proc_shutoff__::s.shutoff = false;
}

inline
int proc_shutoff_alloc_signal(int sig, int flags) {
	struct sigaction sigact;

	::memset(&sigact, 0, sizeof(sigact));

	sigact.sa_handler = proc_shutoff;
	sigact.sa_flags |= flags;

	return ::sigaction(sig, &sigact, 0);
}

inline
int proc_shutoff_ignore_signal(int sig, int flags) {
	struct sigaction sigact;

	::memset(&sigact, 0, sizeof(sigact));

	sigact.sa_handler = proc_sig_ignore;
	sigact.sa_flags |= flags;

	return ::sigaction(sig, &sigact, 0);
}


#endif /* GND_SHUTOFF_HPP_ */
