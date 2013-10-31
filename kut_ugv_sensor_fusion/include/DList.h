/*
 * DList.h
 *
 *  Created on: April 12, 2013
 *      Author: Daniel Oñoro Rubio
 */


#ifndef _DLIST
#define _DLIST

#include <iostream>

template <class T>
class Node
{
public:
	Node(T _element):element(_element),next(NULL),prev(NULL){};

	T element;
	Node *next, *prev;
};


template <class T>
class DList
{
public:
	DList();
	~DList();

	// Modifiers
	// Increaser
	void push_back(T element);
	void push_front(T element);

	// Reducer
	void clear();
	bool pop_front();
	bool pop_back();
	bool erase(Node<T> *n_ptr);

	// Access
	Node<T>* begin() const;
	Node<T>* end() const;
	T front();
	T back();

	// Data control
	unsigned int size();
	bool empty();

private:
	Node<T> *_begin, *_end;
	unsigned int _nelements;
	bool _empty;
};

template <class T>
DList<T>::DList()
{
	_begin = NULL;
	_end = NULL;
	_empty = true;
	_nelements = 0;
}

template <class T>
DList<T>::~DList()
{
  clear();
}

template <class T>
void DList<T>::push_back(T element)
{
	Node<T> *new_end =  new Node<T>(element);

	if(_empty)
	{
		_end = new_end;
		_begin = _end;
		_empty = false;
	}
	else
	{
		_end->next = new_end;
		new_end->prev = _end;
		_end = new_end;
	}

	_nelements++;
}

template <class T>
void DList<T>::clear()
{
  while(!_empty)
  {
          pop_front();
  }
}

template <class T>
void DList<T>::push_front(T element)
{
	Node<T> *new_begin =  new Node<T>(element);

	if(_empty)
	{
		_end = new_begin;
		_begin = _end;
		_empty = false;
	}
	else
	{
		_begin->prev = new_begin;
		new_begin->next = _begin;
		_begin = new_begin;
	}

	_nelements++;
}

template <class T>
bool DList<T>::pop_front()
{
	if(_empty)
	{
		return false;
	}

	if(_begin == _end)
	{
		delete _begin;
	}
	else
	{
		Node<T> *to_delete = _begin;
		_begin = _begin->next;
		_begin->prev = NULL;
		delete to_delete;
	}
	_nelements--;
	_empty = _nelements==0;

	return true;
}

template <class T>
bool DList<T>::pop_back()
{
	if(_empty)
	{
		return false;
	}

	if(_begin == _end)
	{
		delete _begin;
	}
	else
	{
		Node<T> *to_delete = _end;
		_end = _end->prev;
		_end->next = NULL;
		delete to_delete;
		}
	_nelements--;
	_empty = _nelements==0;

	return true;
}

template <class T>
bool DList<T>::erase(Node<T> *n_ptr)
{
	if(_empty || !n_ptr)
	{
		return false;
	}
	/*
	 * Three cases:
	 */
	if(n_ptr == _begin)
	{
		pop_front();
	} else if(n_ptr == _end)
	{
		pop_back();
	} else
	{
		Node<T> *prev, *next;
		prev = n_ptr->prev;
		next = n_ptr->next;

		prev->next = next;
		next->prev = prev;

		_nelements--;
		_empty = _nelements==0;
		delete n_ptr;
	}

	return true;
}

template <class T>
Node<T>* DList<T>::begin() const
{
	return _begin;
}

template <class T>
Node<T>* DList<T>::end() const
{
	return _end;
}

template <class T>
unsigned int DList<T>::size()
{
	return _nelements;
}

template <class T>
bool DList<T>::empty()
{
	return _empty;
}

template <class T>
T DList<T>::front()
{
	return _begin->element;
}

template <class T>
T DList<T>::back()
{
	return _end->element;
}

#endif // END _DLIST

