#ifndef QSTREAM_H
#define QSTREAM_H

#include <string>
#include <cstring>
#include <sys/stat.h>
#include <sys/fcntl.h>

class qstream
{
	private:

		int handle,length;

	protected:

		void qopen(const char *filename, int amode)
		{
			if(handle != -1) return;

			mkfifo(filename, 0666);	
			handle = ::open(filename, amode);
		}

		void qread( void *p , int size )
		{
			if( !good() ) return;
			if( size==0 ) return;
			while(length=read(handle, p, size), length>0)
			{
				if(size-=length, size==0) break;
			}
		}

		void qwrite( const void *p , int size )
		{
			if( !good() ) return;
			if( size==0 ) return;
			write(handle, p, size);
		}

	public:

		 qstream() { handle=-1; length=1; }
		~qstream() { close(); }
	
		virtual void open(const char *filename) = 0;
		void close() { ::close(handle); handle=-1; }
		bool good()  { return (handle>=0 && length>0); }
		operator bool() { return good(); }
};

class iqstream : public qstream
{
	public:

	iqstream  ()                     {}
	iqstream  (const char *filename) { qopen(filename, O_RDONLY); }
	void open (const char *filename) { qopen(filename, O_RDONLY); }

	iqstream& operator>>(         bool   &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         short  &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         char   &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         int    &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         long   &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         float  &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(         double &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(unsigned short  &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(unsigned char   &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(unsigned int    &val) { qread(&val, sizeof(val)); return *this; }
	iqstream& operator>>(unsigned long   &val) { qread(&val, sizeof(val)); return *this; }

	iqstream& operator>>(char *str)
	{
		int len = 0;
		qread(&len, sizeof(len));
		qread(str, len);
		str[len] = 0;
		return *this;
	}

	iqstream& operator>>(std::string &str)
	{
		int len = 0;
		qread(&len, sizeof(len));
		char *buf = new char[len+1];
		qread(buf, len);
		buf[len] = 0;
		str = buf;
		delete[] buf;
		return *this;
	}
};

class oqstream : public qstream
{
	public:

	oqstream  ()                     {}
	oqstream  (const char *filename) { qopen(filename, O_WRONLY); }
	void open (const char *filename) { qopen(filename, O_WRONLY); }

	oqstream& operator<<(const          bool   &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          short  &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          char   &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          int    &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          long   &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          float  &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const          double &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const unsigned short  &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const unsigned char   &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const unsigned int    &val) { qwrite(&val, sizeof(val)); return *this; }
	oqstream& operator<<(const unsigned long   &val) { qwrite(&val, sizeof(val)); return *this; }

	oqstream& operator<<(const char *str)
	{
		int len = strlen(str);
		qwrite(&len, sizeof(len));
		qwrite(str, len);
		return *this;
	}

	oqstream& operator<<(const std::string &str)
	{
		return operator<<( str.c_str() );
	}
};


#endif
