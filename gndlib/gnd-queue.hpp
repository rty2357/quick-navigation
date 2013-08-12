/*
 * gnd-queue.hpp
 *
 *  Created on: 2011/10/03
 *      Author: tyamada
 */

#ifndef GND_QUEUE_HPP_
#define GND_QUEUE_HPP_

#include <string.h>

#include "gnd-lib-error.h"


/**
 * @ifnot GNDStorage
 * @defgroup GNDStorage storage
 * @endif
 */

/**
 * @defgroup GNDQueue queue
 * @ingroup GNDStorage
 */

// ---> class definition
namespace gnd {

	/**
	 * @ingroup GNDQueue
	 * @brief queue storage
	 */
	template < typename T >
	class queue {
		// ---> constants
	public:
		static const uint64_t InitAlloc = 8;	///< initialize allocate buffer

		// <--- constants

		// ---> constructor, destructor
	public:
		queue();
		~queue();
	private:
		int __initialize__();
		// <--- constructor, destructor



		// ---> data
	protected:
		T* _data;							///< data buffer
		uint64_t _n;							///< number of data
		uint64_t _nalloc;						///< number of allocate
	protected:
		int __allocate__(uint64_t n);
		int __deallocate__();
		int __reallocate__(uint64_t n = 0);
		// <--- data
	public:
		int allocate(uint64_t n);

		// assign
	public:
		int assign(queue<T> *a);

		// ---> get property
	public:
		uint64_t nalloc() const;
		uint64_t size() const;
		// <--- get property

		// ---> data entry and delete
	public:
		// insert
		int insert(const uint64_t i, const T* src, const uint64_t n = 1);
		// erase
		int erase(const uint64_t i, const uint64_t n = 1);
		int erase(T* p);

		int push_back(const T* src, const uint64_t n = 1);
		int push_front(const T* src, const uint64_t n = 1);
		int pop_back(T* dest, const uint64_t n = 1);
		int pop_front(T* dest, const uint64_t n = 1);

		int clear();

		// ---> copy and move
	public:
		// mode
		int move(const uint64_t i, T* dest, const uint64_t n = 1);
		int copy(const T* src, const uint64_t n);
		// <--- copy and move


		// ---> get reference pointer
	public:
		T* begin();
		T* end();
		const T* const_begin() const;
		const T* const_end() const;
		// <--- get reference pointer

		// ---> data copy and move method
	public:
		virtual int __copy__(T* dest, const T* src, uint64_t len);
		virtual int __move__(T* dest, const T* src, uint64_t len);
		// <--- data copy and move method

		// ---> operator over ride
	public:
		T& operator[](int i);
		T* operator +(int i);
	};


	/**
	 * @brief constructor
	 */
	template < typename T >
	inline
	queue<T>::queue():
	_data(0), _n(0), _nalloc(0)
	{

	}


	/**
	 * @brief destructor
	 * @note deallocate buffer memory
	 */
	template < typename T >
	inline
	queue<T>::~queue()
	{
		__deallocate__();
	}


	/**
	 * @brief initialize at constructor
	 */
	template < typename T >
	inline
	int queue<T>::__initialize__()
	{
		return 0;
	}



	/**
	 * @brief allocate memory
	 */
	template < typename T >
	inline
	int queue<T>::__allocate__(uint64_t n)
	{
		return __reallocate__();
	}



	/**
	 * @brief deallocate memory
	 */
	template < typename T >
	inline
	int queue<T>::__deallocate__()
	{
		if(!_data)	return -1;

		delete [] _data;
		return 0;
	}



	/**
	 * @brief allocate memory
	 * @note allocate more memory and copy current data
	 */
	template < typename T >
	inline
	int queue<T>::__reallocate__(uint64_t n)
	{
		// first allocate
		if(!_data){
			_data = new T[InitAlloc];
			if( !_data ){
				return -1;
			}
			_nalloc = InitAlloc;
			return 0;
		}

		{ // ---> reallocate
			T* stack;
			uint64_t nalloc;

			// compute allocate size
			for(nalloc = _nalloc << 1; nalloc < n; nalloc <<= 1);

			// allocate for stack and copy
			if( !(stack = new T[nalloc]) ){
				return -1;
			}
			__copy__(stack, _data, _n);
			// deallocate former memory
			__deallocate__();

			// swap
			_data = stack;
			_nalloc = nalloc;
		} // <--- reallocate
		return 0;
	}

	template < typename T >
	inline
	int queue<T>::allocate(uint64_t n) {
		return __reallocate__(n);
	}

	// ---> assign
	/**
	 * @brief assign other storage
	 * @param[in] a :other buffer
	 * @note assign other storage. and clear this storage data
	 */
	template < typename T >
	inline
	int queue<T>::assign(queue<T> *a)
	{
		// clear
		a->__deallocate__();

		// move allocate memory
		a->_data = _data;
		a->_n = _n;
		a->_nalloc = _nalloc;

		// clear
		_data = 0;
		_n = 0;
		_nalloc = 0;
		return 0;
	}



	// ---> getter, setter
	/**
	 * @brief allocate byte size
	 * @return allocate byte size
	 */
	template < typename T >
	inline
	uint64_t queue<T>::nalloc() const
	{
		return _nalloc;
	}

	/**
	 * @brief data size
	 * @return entry data size
	 */
	template < typename T >
	inline
	uint64_t queue<T>::size() const
	{
		return _n;
	}



	/**
	 * @brief insert data
	 * @param[in]   i: inset index
	 * @param[in] src: data source
	 * @param[in]   n: size of insert data
	 * @return < 0 fail to insert
	 * 			insert index
	 */
	template < typename T >
	inline
	int queue<T>::insert(const uint64_t i, const T* src, const uint64_t n)
	{
		gnd_assert(!src, -1, "invalid argument");
		gnd_assert(i > _n, -1, "out of buffer");
		gnd_error(n == 0, 0, "ineffectual argument");

		// reallocate
		while(_nalloc < _n + n)	__reallocate__(_n + n);

		// move
		if(i != _n)	__move__(_data + i + n, _data + i, (_n - i));
		// copy
		__copy__(_data + i, src, n);
		_n += n;

		return i;
	}



	/**
	 * @brief move data
	 * @param[in]     i: object index
	 * @param[out] dest: destination
	 * @param[in]     n: size of moved data
	 * @return < 0 fail to move
	 * 			0
	 */
	template < typename T >
	inline
	int queue<T>::move(const uint64_t i, T* dest, const uint64_t n)
	{
		gnd_assert(i + n >= _n, -1, "out of buffer");
		gnd_error(n == 0, 0, "ineffectual argument");

		// copy
		if(dest) __copy__(dest, _data + i, n);
		// move
		if(i + n != _n)	__move__(_data + i, _data + i + n, (_n - i - n));
		_n -= n;

		return 0;
	}



	/**
	 * @brief erase data
	 * @param[in]     i: object index
	 * @param[in]     n: size of erase data
	 * @return < 0 fail to erase
	 * 			0
	 */
	template < typename T >
	inline
	int queue<T>::erase(const uint64_t i, const uint64_t n)
	{
		return move(i, 0, n);
	}



	/**
	 * @brief erase data
	 * @param[in]     p: object pointer
	 * @return < 0 fail to erase
	 * 			0
	 */
	template < typename T >
	inline
	int queue<T>::erase(T* p)
	{
		uint64_t i;
		gnd_error(_n == 0, -1, "out of buffer");
		gnd_error(!p, -1, "ineffectual argument");

		for(i = 0; _data + i != p && i < _n && _data + i != 0; p++);

		if( i >= _n || _data + i == 0 )	return -1;

		// remove
		if(i + 1 != _n)	__move__(_data + i, _data + i + 1, (_n - i - 1));
		_n--;

		return move(i, p);
	}

	/**
	 * @brief push back data
	 * @param[in] src : source data
	 * @param[in]   n : number of data
	 * @return insert index
	 */
	template < typename T >
	inline
	int queue<T>::push_back(const T* src, const uint64_t n)
	{
		return insert(size(), src, n);
	}

	/**
	 * @brief push front data
	 * @param[in] src : source data
	 * @param[in]   n : number of data
	 * @return 0(insert index)
	 */
	template < typename T >
	inline
	int queue<T>::push_front(const T* src, const uint64_t n)
	{
		return insert(0, src, n);
	}


	/**
	 * @brief pop back data
	 * @param[out] dest: destination
	 * @param[in]    n : number of data
	 * @return 0
	 */
	template < typename T >
	inline
	int queue<T>::pop_back(T* dest, const uint64_t n)
	{
		return move(size(), dest, n);
	}

	/**
	 * @brief pop front data
	 * @param[out] dest: destination
	 * @param[in]    n : number of data
	 * @return 0
	 */
	template < typename T >
	inline
	int queue<T>::pop_front(T* dest, const uint64_t n)
	{
		return move(0, dest, n);
	}



	/**
	 * @brief copy
	 * @param[out] dest : destination
	 * @param[in]   src : source data
	 * @param[in]   len : data length
	 * @return 0
	 */
	template < typename T >
	inline
	int queue<T>::__copy__(T* dest, const T* src, uint64_t len)
	{
		::memcpy(dest, src, sizeof(T) * len);
		return 0;
	}


	/**
	 * @param[out] dest : destination
	 * @param[in]   src : data source
	 * @param[in]   len : length of data
	 */
	template < typename T >
	inline
	int queue<T>::__move__(T* dest, const T* src, uint64_t len)
	{
		::memmove(dest, src, sizeof(T) * len);
		return 0;
	}

	/**
	 * @brief get buffer header pointer
	 * @return header pointer
	 */
	template < typename T >
	inline
	T* queue<T>::begin()
	{
		return _data;
	}

	/**
	 * @brief get buffer header pointer
	 * @return header pointer
	 */
	template < typename T >
	inline
	const T* queue<T>::const_begin() const
	{
		return _data;
	}



	/**
	 * @brief get buffer end pointer
	 * @return end pointer
	 */
	template < typename T >
	inline
	T* queue<T>::end()
	{
		return _data + _n;
	}


	/**
	 * @brief get buffer end pointer
	 * @return end pointer
	 */
	template < typename T >
	inline
	const T* queue<T>::const_end() const
	{
		return _data + _n;
	}



	/**
	 * @brief clear
	 */
	template < typename T >
	inline
	int queue<T>::clear()
	{
		_n = 0;
		return 0;
	}



	/**
	 * @brief copy data
	 * @param[in] src : copy
	 * @param[in]   n : size of copy data
	 * @note clear current data and copy source data
	 */
	template < typename T >
	inline
	int queue<T>::copy(const T* src, const uint64_t n)
	{
		gnd_assert(!src, -1, "invalid arugment");
		gnd_error(n == 0, 0, "ineffectual argument");

		// reallocate
		while(_nalloc < _n + n)	__reallocate__(_n + n);

		__copy__(_data, src, n);
		_n = n;
		return 0;
	}



	/**
	 * @brief indexer over-ride
	 * @param[in] i : index
	 * @note interface like array
	 */
	template < typename T>
	inline
	T& queue<T>::operator[](int i)
	{
		return _data[i];
	}



	/**
	 * @brief operater over-ride
	 * @param[in] i : index
	 * @note interface like array
	 * @return pointer like array
	 */
	template < typename T>
	inline
	T* queue<T>::operator +(int i)
	{
		return _data + i;
	}

}
// <--- class definition

#endif /* GND_QUEUE_HPP_ */
