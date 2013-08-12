/*
 * gnd-config-file.hpp
 *
 *  Created on: 2011/09/16
 *      Author: tyamada
 */

#ifndef GND_CONFIG_FILE_HPP_
#define GND_CONFIG_FILE_HPP_


// include logging function for library debug
#define GND_DEBUG_LOG_NAMESPACE1 gnd
#define GND_DEBUG_LOG_NAMESPACE2 conf
#include "gnd-debug-log.hpp"
#undef GND_DEBUG_LOG_NAMESPACE2
#undef GND_DEBUG_LOG_NAMESPACE1
#undef GND_DEBUG_LOG
#include "gnd-debug-log-util-def.h"


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>


#include "gnd-queue.hpp"

#include "gnd-lib-error.h"
#include "gnd-matrix-base.hpp"

/**
 * @defgroup GNDConf configuration
 * supply configuration file reading and writing
 */

// ---> class declaration
namespace gnd {
	namespace conf {
		class file_stream;
		class configuration;
	}
} // <--- class declaration


// ---> constant value definition
namespace gnd {
	namespace conf {
		static const uint32_t ItemBufferSize = 32;		///<! item buffer size
		static const uint32_t ValueBufferSize = 256;		///<! buffer size
		static const uint32_t CommentBufferSize = 512;	///<! buffer size

		static const char TokenSubst = '=';				///<! substitution token
		static const char TokenComment = '#';			///<! comment token
		static const char TokenBeginScope = '{';		///<! scope beginning token
		static const char TokenEndScope = '}';			///<! scope ending token
		static const char TokenEndLine = '\n';			///<! end-line token
		static const char TokenEndValue = ',';			///<! end value token

		/**
		 * @ingroup GNDConf
		 * @brief token-list
		 */
		static const char TokenList[] ={
				TokenSubst,
				TokenComment,
				TokenBeginScope,
				TokenEndScope,
				TokenEndLine,
				TokenEndValue,
				'\0'
		};
	}
} // <--- constant value definition





// ---> class confparam definition
namespace gnd {
	namespace conf {
		/**
		 * @ingroup GNDConf
		 * @brief configuration parameter class
		 */
		class configuration {
			friend class file_stream;
			friend class queue<configuration>;
		public:
			configuration();
			configuration(const char *n, const char *v, const char *c = 0);
			~configuration();

		private:
			/// @brief item
			char _item[gnd::conf::ItemBufferSize];
			/// @brief value string
			char _value[gnd::conf::ValueBufferSize];
			/// @brief comment
			char _comment[gnd::conf::CommentBufferSize];
			/// @brief nesting child list
			queue<configuration> _children;

			// setter
		public:
			int set(const char* n, const char* v, const char* c = 0);

			// getter
		public:
			int get(char* n, char* v, char *c) const;
			const char* name() const;
			const char* value() const;
			const char* comment() const;


			// set child
		public:
			int child_push_back(const char* n, const char* v = 0, const char* c = 0);
			int child_push_front(const char* n, const char* v = 0, const char* c = 0);

			// get child
		public:
			int child_pop_back(configuration* dest);
			int child_pop_front(configuration* dest);
			configuration* child_find(const char* _name, const char* _value = 0);

		public:
			int nchild();

			// deallocate
		public:
			int child_erase(configuration *ch);
			int child_clear();
			int clear();

			// indexer
		public:
			configuration& operator[](int i);
			// show
		public:
			static const char NoValComment = 0x01;
		public:
			int show(FILE* fp, const char flags = 0x00);
		private:
			int __show__(FILE* fp, const int depth, const char flags = 0x00);
		};
	}
} // <---  class confparam definition


namespace gnd {
	template<>
	inline
	int queue<gnd::conf::configuration>::__move__(gnd::conf::configuration* dest, const gnd::conf::configuration* src, uint64_t len)
	{
		uint64_t i = 0;
		gnd::conf::configuration *p = const_cast<gnd::conf::configuration* >(src);

		for(i = 0; i < len; i++){
			::strncpy(dest[i]._item, src[i]._item, sizeof(dest[i]._item));
			::strncpy(dest[i]._value, src[i]._value, sizeof(dest[i]._value));
			::strncpy(dest[i]._comment, src[i]._comment, sizeof(dest[i]._comment));
			p[i]._children.assign(&dest[i]._children);
		}

		return 0;
	}


	template<>
	inline
	int queue<gnd::conf::configuration>::__copy__(gnd::conf::configuration* dest, const gnd::conf::configuration* src, uint64_t len)
	{
		return __move__(dest, src, len);
	}

}

// ---> constructor, destructor
/**
 * @brief constructor
 */
inline
gnd::conf::configuration::configuration()
{
	clear();
}

/**
 * @brief constructor
 * @param[in] n: configuration parameter name
 * @param[in] v: configuration parameter value
 */
inline
gnd::conf::configuration::configuration(const char* n, const char* v, const char *c)
{
	set(n, v, c);
}

/**
 * @brief destructor
 */
inline
gnd::conf::configuration::~configuration()
{
}




// ---> setter
/**
 * @brief set parameter
 * @param[in] n: configuration parameter name
 * @param[in] v: configuration parameter value
 * @return 0
 */
inline
int gnd::conf::configuration::set(const char* n, const char* v, const char *c)
{
	::memset(_item, 0, sizeof(_item));

	if(n){
		::strncpy(_item, n, sizeof(_item));
	}

	::memset(_value, 0, sizeof(_value));
	if(v){
		::strncpy(_value, v, sizeof(_value));
	}

	::memset(_comment, 0, sizeof(_comment));
	if(c){
		::strncpy(_comment, c, sizeof(_comment) );
	}
	return 0;
}


// ---> getter
/**
 * @brief get parameter
 * @param[out] n: configuration parameter name
 * @param[out] v: configuration parameter value
 * @param[out] c: configuration parameter comment
 * @return 0
 */
inline
int gnd::conf::configuration::get(char* n, char* v, char *c) const
{
	if(n)	::strcpy(n, _item);
	if(v)	::strcpy(v, _value);
	if(c)	::strcpy(c, _comment);
	return 0;
}

/**
 * @brief return name
 * @return configuration parameter name
 */
inline
const char* gnd::conf::configuration::name() const
{
	return _item;
}

/**
 * @brief return value
 * @return configuration parameter value
 */
inline
const char* gnd::conf::configuration::value() const
{
	return _value;
}

/**
 * @brief return comment
 * @return configuration parameter comment
 */
inline
const char* gnd::conf::configuration::comment() const
{
	return _comment;
}


// set child
/**
 * @brief set child (push back)
 * @param[in] n: child configuration data name
 * @param[in] v: child configuration data value
 * @return insert index
 */
inline
int gnd::conf::configuration::child_push_back(const char* n, const char* v, const char *c)
{
	configuration child;

	child.set(n, v, c);

	return _children.push_back(&child);
}

/**
 * @brief set child (push front)
 * @param[in] n: child configuration data name
 * @param[in] v: child configuration data value
 * @return 0
 */
inline
int gnd::conf::configuration::child_push_front(const char* n, const char* v, const char *c)
{
	configuration child;

	child.set(n, v, c);

	return _children.push_front(&child);
}


/**
 * @brief pop back child
 * @param[out] dest : destination
 * @return 0;
 */
inline
int gnd::conf::configuration::child_pop_back(configuration* dest)
{
	return _children.pop_back(dest);
}



/**
 * @brief pop back child
 * @param[out] dest : destination
 * @return 0;
 */
inline
int gnd::conf::configuration::child_pop_front(configuration* dest)
{
	return _children.pop_front(dest);
}


/**
 * @brief fild child
 * @param[in] n : key name
 * @param[in] v : key value
 * @return ==0  : not found
 */
inline
gnd::conf::configuration* gnd::conf::configuration::child_find(const char* n, const char* v)
{
	uint64_t i;

	for(i = 0; i < _children.size(); i++){
		if( ::strncmp(_children[i]._item, n, sizeof(_children[i]._item)) == 0 &&
				(v == 0 || ::strncmp(_children[i]._value, v, sizeof(_children[i]._value)) == 0 )) {
			return &_children[i];
		}
	}


	return 0;
}



/**
 * @brief return number of child node
 * @return number of child node
 */
inline
int gnd::conf::configuration::nchild()
{
	return _children.size();
}



// ---> deallocate
/**
 * @brief erase child
 * @param[in] ch : child
 * @return number of child node
 */
inline
int gnd::conf::configuration::child_erase(configuration *ch)
{
	return _children.erase(ch);
}



/**
 * @brief clear child list
 */
inline
int gnd::conf::configuration::child_clear()
{
	return _children.clear();
}



/**
 * @brief clear child list and my name and value
 */
inline
int gnd::conf::configuration::clear()
{
	set(0, 0, 0);
	return child_clear();
}


// ---> indexer
/**
 * @brief indexer operator over-ride
 * @details like array indexer
 */
inline
gnd::conf::configuration& gnd::conf::configuration::operator[](int i)
{
	return _children[i];
}


// ---> show
/**
 * @brief show configuration
 * @param[in] fp : file stream
 */
inline
int gnd::conf::configuration::show(FILE* fp, const char flags)
{
	uint64_t i;

	for(i = 0; i < _children.size(); i++){
		_children[i].__show__(fp, 0, flags);
	}
	return 0;
}


/**
 * @brief show configuration parameter name, value
 * @param[in]    fp : file stream
 * @param[in] depth : nesting depth
 */
int gnd::conf::configuration::__show__(FILE* fp, const int depth, const char flags)
{
	char head[32];
	uint64_t i;
	bool nl_flg = false;	// new line flag


	if( _item[0] && _comment[0] ) {
		if(fp) ::fprintf(fp, "# %s\n", _comment);
		LogDebugf("# %s\n", _comment);
		nl_flg = true;
	}

	if ( !_value[0] && _children.size() == 0) {
		if(fp) ::fprintf(fp, "#");
		LogDebug("#");
	}


	::memset(head, 0, sizeof(head));
	for(i = 0; (signed)i < depth; i++){
		head[i] = '\t';
	}
	if(fp) ::fprintf(fp, "%s", head);
	LogDebugf("%s", head);

	if( _item[0] != '\0' ){
		if(fp) ::fprintf(fp, "%s=", _item);
		LogDebugf("%s=", _item);
	}

	if( _value[0] != '\0' ){
		if(fp) ::fprintf(fp, "%s%s", _value, _item[0] ? "" : ",");
		LogDebugf("%s%s", _value, _item[0] ? "" : ",");
	}
	else if( _children.size() != 0 ){
		if(fp) ::fprintf(fp, "{\n");
		LogDebug("{");
		for(i = 0; i < _children.size(); i++){
			_children[i].__show__(fp, depth + 1);
		}
		if(fp) ::fprintf(fp, "%s}", head);
		LogDebugf("%s}", head);
	}
	if(fp) ::fprintf(fp, "\n");
	LogDebug("\n");

	// new line
	if(nl_flg) {
		if(fp) ::fprintf(fp, "\n");
		LogDebug("\n");
	}

	return 0;
}


// ---> class fstream definition
namespace gnd {
	namespace conf {
		/**
		 * @ingroup GNDConf
		 * @brief configuration file stream
		 */
		class file_stream
		: public configuration
		  {
		  public:
			file_stream();
			~file_stream();

			// read wirte
		  public:
			int read(const char *name);
			int write(const char *name, const char flags = NoValComment);
		  private:
			static int _parse_(FILE *fp, configuration *conf, int *line);
			static int substitution(FILE *fp, char *n, char* v, configuration *conf, int *line);
			static int unnamed(char *v, configuration *conf, int *line);
			static int nesting(char *v, configuration *conf);

		  private:
			static int erase_blank(char *str);
			static int erase_nonblank(char *str);
			static int erase_comment(char *str);
			static int get_ascii(char *dest, char *src);
			static int find_separater(char *str);
		  };
		typedef file_stream gnd_conf_fs;

		/**
		 * @brief constructor
		 */
		file_stream::file_stream()
		{

		}

		/**
		 * @brief destructor
		 */
		file_stream::~file_stream()
		{

		}
	}
}
// <--- class fstream definition



// ---> class fstream member function definition
namespace gnd {
	namespace conf {

		/**
		 * @brief read configure file
		 * @param[in] name : configure file name
		 */
		inline
		int file_stream::read(const char *name)
		{
			FILE *fp;
			int ret = 0;
			int line = 0;

			if( !(fp = fopen(name, "r")) ) return -1;

			ret = _parse_(fp, this, &line);

			fclose(fp);
			return ret;
		}




		/**
		 * @brief write configure file
		 * @param[in] name : configure file name
		 */
		inline
		int file_stream::write(const char *name, const char flags)
		{
			FILE *fp;

			if( !(fp = fopen(name, "w")) ) return 0;

			show(fp, flags);

			fclose(fp);
			return 0;

		}


		/**
		 * @brief parse
		 * @param[in]    fp : file stream
		 * @param[out] conf : configuration
		 * @param[in]  line : reading line
		 * @retval ==0 : success
		 * @retval < 0 : failure
		 */
		inline
		int file_stream::_parse_(FILE *fp, configuration *conf, int *line)
		{
			char ws[512];
			int ret;

			LogDebugf("Begin - int FileStream::_parse_(%p, %p, %p)\n", fp, conf, line);
			LogIndent();

			// ---> file read loop
			while( !::feof(fp) ){
				// zero clear workspace
				::memset(ws, 0, sizeof(ws));
				// file read
				if( !::fgets(ws + ::strlen(ws), sizeof(ws) - ::strlen(ws), fp) )
					break;
				// count line number
				(*line)++;
				LogDebugf("line \'%d\'\n", *line);

				conf->show(0);
				// erase comment
				erase_comment(ws);

				if( (ret = find_separater(ws)) < 0){
					if( unnamed(ws, conf, line) < 0){
						LogUnindent();
						LogDebugf("Fail - int FileStream::_parse_(%p, %p, %p)", fp, conf, line);
						return -1;
					}
				}
				else {
					LogDebugf("get separater \'%c\'\n", ws[ret]);
					switch( ws[ret] ){
					// substruct
					case TokenSubst:
					{
						ws[ret] = '\0';
						if( substitution(fp, ws, ws + ret + 1, conf, line) < 0){
							LogUnindent();
							LogDebugf("Fail - int FileStream::_parse_(%p, %p, %p)", fp, conf, line);
							return -1;
						}
					}
					break;

					// nesting
					case TokenBeginScope:
						LogDebug("invalid nesting");
						break;
					case TokenEndScope:
						if( unnamed(ws, conf, line) < 0){
							LogDebugf("configure read error at line %d\n", *line);
							LogUnindent();
							LogDebugf(" Fail: int FileStream::_parse_(%p, %p, %p)", fp, conf, line);
							return -1;
						}
						return 0;

						// unnamed value
					case TokenComment:
					case TokenEndValue:
					case TokenEndLine:
						if( unnamed(ws, conf, line) < 0){
							LogDebugf("configure read error at line %d\n", *line);
							LogUnindent();
							LogDebugf(" Fail: int FileStream::_parse_(%p, %p, %p)", fp, conf, line);
							return -1;
						}
						break;
					} // <--- switch

				}
			}
			// <--- file read loop

			LogUnindent();
			LogDebugf("  End: int FileStream::_parse_(%p, %p, %p)\n", fp, conf, line);
			return 0;
		}




		/**
		 * @brief parse - substruction context
		 * @param[in]    fp : file stream
		 * @param[in]     n : name string header pointer
		 * @param[in]     v : value string header pointer
		 * @param[in]     s : configuration
		 * @param[in]  line : reading line
		 * @return ==0 : success
		 */
		inline
		int file_stream::substitution(FILE *fp, char *n, char *v, configuration *conf, int *line)
		{
			char name[512];
			char value[512];

			LogDebugf("Begin: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);

			{ // ---> get name
				// zero initialize
				::memset(name, 0, sizeof(name));

				get_ascii(name, n);
				erase_blank(n);
				if( *n != '\0'){
					LogDebugf(" Fail: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);
					return -1;
				}
				LogDebugf("     : name \"%s\"\n", name);
			} // <--- get name

			{ // ----> get value or nesting
				erase_blank(v);
				LogDebugf("     : value \"%s\"\n", v);
				if( v[0] != TokenBeginScope ){

					// check error
					if( *v == '\0' ){
						LogDebugf(" Fail: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);
						return -1;
					}

					{// ---> get value
						get_ascii(value, v);
						erase_blank(v);
						if( *value == '\0'){
							LogDebugf(" Fail: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);
							return -1;
						}
						if( *v != '\0'){
							LogDebugf(" Fail: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);
							return -1;
						}
						conf->child_push_back(name, value);
					}// <--- get value
				}
				// ---> nesting
				else {
					LogDebug("     : nesting\n");
					::memmove(v, v + 1, ::strlen(v) - 1);
					erase_blank(v);
					LogDebugf("     : nesting \"%s\"\n", v);
					if( *v == '\0' ){
						configuration *p;
						// todo modify to not need search
						conf->child_push_back(name, 0);
						p = conf->child_find(name, 0);
						_parse_(fp, p, line);
					}
					else {
						configuration *p;
						// todo modify to not need search
						conf->child_push_back(name, 0);
						p = conf->child_find(name, 0);
						if( unnamed(v, p, line) == 1) {
							// not end of nesting
							// read next line
							_parse_(fp, p, line);
						}
					}
				} // <--- nesting

			} // <---- get value or nesting


			LogDebugf("  End: int FileStream::substitution(%p, \"%s\", \"%s\", %p, %d)\n", fp, n, v, conf, *line);
			return 0;
		}


		/**
		 * @brief parse - substruction context
		 * @param[in]    v : value string header pointer
		 * @param[in] conf : configuration
		 * @param[in] line : reading line
		 * @return ==0 : success
		 */
		inline
		int file_stream::unnamed(char *v,configuration *conf, int *line)
		{
			char value[512];
			int ret = 0;
			int sep = 0;
			char sep_token = 0;

			LogDebugf("Begin - int FileStream::unnamed(\"%s\", %p, %d)\n", v, conf, *line);
			LogIndent();

			{ // ----> get value or nesting
				// get separater
				if((sep = find_separater(v)) >= 0){
					sep_token = v[sep];
					v[sep] = '\0';
				}

				// erase blank
				erase_blank(v);
				// check blank
				if(  *v  == '\0'){
					LogDebug("next line\n");
					LogUnindent();
					LogDebugf("End - int FileStream::unnamed(\"%s\", %p, %d)\n", v, conf, *line);
					return 1;
				}
				else if(*v == TokenEndScope ){
					// no value
					LogDebug("end nesting\n");
					LogUnindent();
					LogDebugf("End - int FileStream::unnamed(\"%s\", %p, %d)\n", v, conf, *line);
					return 0;
				}
				else if( *v == TokenEndValue){
					LogDebug("invalid token\n");
					LogUnindent();
					LogDebugf("Fail - int FileStream::unnamed(\"%s\", %p, %d)\n", v, conf, *line);
					return -1;
				}
				else {// ---> get value
					get_ascii(value, v);
					erase_blank(v);

					if( *v  != '\0') return -1;
				}// <--- get value


				// get unnamed value
				if( *value != '\0'){
					conf->child_push_back(0, value);
					if( sep_token == TokenEndValue){
						ret = unnamed(v + sep + 1, conf, line);
					}
				}
			} // <---- get value or nesting

			LogUnindent();
			LogDebugf("End - int FileStream::unnamed(\"%s\", %p, %d)\n", v, conf, *line);
			return ret;
		}





		/**
		 * @breif erase blank character from str head
		 */
		inline
		int file_stream::erase_blank(char *str)
		{
			int i;
			gnd_assert(!str, -1, "invalid null pointer");


			for(i = 0; str[i] != '\0' && ::isspace(str[i]); i++);
			::memmove(str, str + i, ::strlen(str + i) + 1);


			return i;
		}


		/**
		 * @breif erase non-blank character from str head
		 * @param[in/out] str
		 */
		inline
		int file_stream::erase_nonblank(char *str)
		{
			int i;
			gnd_assert(!str, -1, "invalid null pointer");


			for(i = 0; str[i] != '\0' && !::isspace(str[i]); i++);
			::memmove(str, str + i, ::strlen(str + i) + 1);

			return i;
		}



		/**
		 * @breif search ascii cord and move those form src to dest
		 * @param[out] dest : destination
		 * @param[in]   src : data source
		 */
		inline
		int file_stream::get_ascii(char *dest, char *src)
		{
			size_t i;

			erase_blank(src);
			for(i = 0; src[i] != '\0' && isascii(src[i]) && !::isspace(src[i]); i++){
				dest[i] = src[i];
			}
			dest[i] = '\0';
			::memmove(src, src + i, ::strlen(src + i) + 1);
			return i;
		}


		/**
		 * @breif erase comment paragraph
		 * @param[out] str : string
		 */
		inline
		int file_stream::erase_comment(char *str)
		{
			char *p;
			gnd_assert(!str, -1, "invalid null pointer");

			if( !(p = ::strchr(str, TokenComment)) )	return 0;
			*p = '\0';
			return 0;
		}

		inline
		int file_stream::find_separater(char *str)
		{
			int i;
			for( i = 0; str[i] != '\0'; i++ ){
				for(const char *p = TokenList; *p != '\0'; p++){
					if(str[i] == *p)	return i;
				}
			}
			return -1;

		}



	}
}
// <--- class fstream member function definition



// ---> parameter class definition
namespace gnd {
	namespace conf {

		/**
		 * @ingroup GNDConfObject
		 * @brief configuration parameter
		 */
		template< typename T >
		struct parameter{
			char item[ItemBufferSize];		///< item
			T value;							///< value
			char comment[CommentBufferSize];	///< comment
		};


		typedef parameter<bool>				param_bool;
		typedef parameter<char>				param_char;
		typedef parameter<unsigned char>	param_uchar;
		typedef parameter<short>			param_short;
		typedef parameter<unsigned short>	param_ushort;
		typedef parameter<int>				param_int;
		typedef parameter<unsigned int>		param_uint;
		typedef parameter<long>				param_long;
		typedef parameter<unsigned long>	param_ulong;
		typedef parameter<float>			param_float;
		typedef parameter<double>			param_double;

		// anonymous
		template< typename T >
		inline
		int get_paramter( configuration * conf, parameter<T> * param ){
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = p->value();
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}

		// bool
		inline
		int get_parameter( configuration * conf, param_bool * param ){
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				if( ::strlen(p->value()) == ::strlen("true") && ::strncmp(p->value(), "true", ::strlen("true")) == 0 ){
					param->value = true;
				}
				else if( ::strlen(p->value()) == ::strlen("false") && ::strncmp(p->value(), "false", ::strlen("false")) == 0 ){
					param->value = false;
				}
				else if( ::strlen(p->value()) == ::strlen("on") && ::strncmp(p->value(), "on", ::strlen("on")) == 0 ){
					param->value = true;
				}
				else if( ::strlen(p->value()) == ::strlen("off") && ::strncmp(p->value(), "off", ::strlen("off")) == 0 ){
					param->value = false;
				}
				else {
					param->value = ::atoi(p->value());
				}
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}


		// short
		inline
		int get_parameter( configuration * conf, param_short * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}


		// unsigned short
		inline
		int get_parameter( configuration * conf, param_ushort * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}



		// int
		inline
		int get_parameter( configuration * conf, param_int * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}


		// unsigned int
		inline
		int get_parameter( configuration * conf, param_uint * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}


		// long
		inline
		int get_parameter( configuration * conf, param_long * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}


		// unsigned long
		inline
		int get_parameter( configuration * conf, param_ulong * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atoi( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}

		// float
		inline
		int get_parameter( configuration * conf, param_float * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atof( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}

		// double
		inline
		int get_parameter(configuration * conf, param_double * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				param->value = ::atof( p->value() );
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}



		//  matrix fixed
		template<uint32_t R, uint32_t C>
		inline
		int get_parameter(configuration * conf, struct parameter< gnd::matrix::fixed<R,C> > * param )
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				for( uint32_t r = 0; r < R && p->nchild() <= r * C; r++){
					for( uint32_t c = 0; c < C && p->nchild() <= r * C + c; c++){
						gnd::matrix::set(&param->value, r, c, ::atof( (*p)[r * C + c].value()) );
					}
				}
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return 0;
		}





		// bool
		inline
		int set_parameter( configuration * conf, const struct parameter<bool> * param ){
			char value[8];

			::sprintf(value, "%s", param->value ? "true" : "false");
			return conf->child_push_back(param->item, value, param->comment);
		}


		// short
		inline
		int set_parameter( configuration * conf, const struct parameter<short> * param )
		{
			char value[512];

			::sprintf(value, "%d", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}


		// unsigned short
		inline
		int set_parameter( configuration * conf, const struct parameter<unsigned short> * param )
		{
			char value[512];

			::sprintf(value, "%d", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}


		// int
		inline
		int set_parameter( configuration * conf, const struct parameter<int> * param )
		{
			char value[512];

			::sprintf(value, "%d", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}


		// unsigned int
		inline
		int set_parameter( configuration * conf, const struct parameter<unsigned int> * param )
		{
			char value[512];

			::sprintf(value, "%d", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}


		// long
		inline
		int set_parameter( configuration * conf, const struct parameter<long> * param )
		{
			char value[512];

			::sprintf(value, "%ld", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}


		// unsigned long
		inline
		int set_parameter( configuration * conf, const struct parameter<unsigned long> * param )
		{
			char value[512];

			::sprintf(value, "%ld", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}

		// float
		inline
		int set_parameter( configuration * conf, const struct parameter<float> * param )
		{
			char value[512];

			::sprintf(value, "%f", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}

		// double
		inline
		int set_parameter(configuration * conf, const struct parameter<double> * param )
		{
			char value[512];

			::sprintf(value, "%lf", param->value);
			return conf->child_push_back(param->item, value, param->comment);
		}



		// matrix_fixed
		template<uint32_t R, uint32_t C>
		inline int set_parameter(configuration * conf, const struct parameter< gnd::matrix::fixed<R,C> > * param )
		{
			int ret = 0;
			char value[512];
			configuration *p = 0;

			if( (ret = conf->child_push_back(param->item, 0, param->comment)) < 0){
				return ret;
			}

			if( !(p = conf->child_find(param->item, value) )) return -1;

			for( uint32_t r = 0; r < R; r++){
				for( uint32_t c = 0; c < C; c++){
					::sprintf(value, "%lf", param->value[r][c]);
					p->child_push_back(0, value);
				}
			}

			return 0;
		}


		/**
		 * @ingroup GNDConfObject
		 * @brief configuration parameter array
		 */
		template< typename T, size_t S >
		struct parameter_array{
			/// @brief parameter item
			char item[ItemBufferSize];
			/// @brief value
			T value[S];
			/// @brief comment for parameter
			char comment[CommentBufferSize];
		};


		template< size_t S>
		inline int get_parameter(configuration * conf, struct parameter_array<char, S> *param, size_t n = 0)
		{
			configuration *p = 0;

			p = conf->child_find(param->item);
			if(p){
				::strncpy(param->value, p->value(), n == 0 ? S : n);
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return ::strlen(param->value);
		}

		template< size_t S>
		inline int get_parameter(configuration * conf, struct parameter_array<int, S> *param, size_t n = 0 )
		{
			configuration *p = 0;
			size_t nn;

			p = conf->child_find(param->item);
			if(p){
				nn = n == 0u ? p->nchild() :
						(signed)n < p->nchild() ? n : p->nchild();
				for(size_t i = 0; i < nn; i++){
					param->value[i] = ::atoi( (*p)[i].value() );
				}
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return nn;
		}

		template< size_t S>
		inline int get_parameter(configuration * conf, struct parameter_array<unsigned int, S> *param, size_t n = 0 )
		{
			configuration *p = 0;
			size_t nn;

			p = conf->child_find(param->item);
			if(p){
				nn = n == 0 ? p->nchild() :
						n < p->nchild() ? n : p->nchild();
				for(size_t i = 0; i < nn; i++){
					param->value[i] = ::atoi( (*p)[i].value() );
				}
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return nn;
		}

		template< size_t S>
		inline int get_parameter(configuration * conf, struct parameter_array<double, S> *param, size_t n = 0u )
		{
			configuration *p = 0;
			size_t nn;

			p = conf->child_find(param->item);
			if(p){
				nn = n == 0u ? p->nchild() :
						(signed)n < p->nchild() ? (signed)n : p->nchild();
				for(size_t i = 0; i < nn; i++){
					param->value[i] = ::atof( (*p)[i].value() );
				}
				::memcpy(param->comment, conf->comment(), sizeof(param->comment));
			}
			else {
				return -1;
			}
			return nn;
		}




		template< size_t S>
		inline int set_parameter(configuration * conf, const struct parameter_array<char, S> *param, size_t n = 0)
		{
			return conf->child_push_back(param->item, param->value, param->comment);
		}

		template< size_t S>
		inline int set_parameter(configuration * conf, const struct parameter_array<int, S> *param, size_t n = 0 )
		{
			int ret = 0;
			char value[512];
			configuration *p = 0;

			if( (ret = conf->child_push_back(param->item, 0, param->comment)) < 0){
				return ret;
			}

			if( !(p = conf->child_find(param->item, 0) )) return -1;

			for( size_t i = 0; i < S; i++){
				::sprintf(value, "%d", param->value[i]);
				p->child_push_back(0, value);
			}

			return 0;
		}

		template< size_t S>
		inline int set_parameter(configuration * conf, const struct parameter_array<unsigned int, S> *param, size_t n = 0 )
		{
			int ret = 0;
			char value[512];
			configuration *p = 0;

			if( (ret = conf->child_push_back(param->item, 0, param->comment)) < 0){
				return ret;
			}

			if( !(p = conf->child_find(param->item, 0) )) return -1;

			for( size_t i = 0; i < S; i++){
				::sprintf(value, "%d", param->value[i]);
				p->child_push_back(0, value);
			}

			return 0;
		}




		template< size_t S>
		inline int set_parameter(configuration * conf, const struct parameter_array<double, S> *param )
		{
			int ret = 0;
			char value[512];
			configuration *p = 0;

			if( (ret = conf->child_push_back(param->item, 0, param->comment)) < 0){
				return ret;
			}

			if( !(p = conf->child_find(param->item, 0) )) return -1;

			for( size_t i = 0; i < S; i++){
				::sprintf(value, "%lf", param->value[i]);
				p->child_push_back(0, value);
			}

			return 0;
		}

	}

}; // <--- parameter class definition

#include "gnd-debug-log-util-undef.h"

#endif /* GND_CONFIGUREFILE_HPP_ */

