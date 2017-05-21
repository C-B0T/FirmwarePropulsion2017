/**
 * @file	Observable.cpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Observable class
 */

#include "Observable.hpp"
#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Utils
{
	Observable::Observable()
	{
	}

	Observable::~Observable()
	{
	}

	void Observable::Subscribe(void * observer, Observer::ObserverCallback cb)
	{
		this->observerInstanceList.push_back(observer);
		this->observerCallbackList.push_back(cb);
	}

	void Observable::Unsubscribe (void * observer, Observer::ObserverCallback cb)
	{
		auto observerIterator = std::find(this->observerInstanceList.begin(), this->observerInstanceList.end(), observer);

		if(observerIterator != this->observerInstanceList.end())
		{
			this->observerInstanceList.erase(observerIterator);
		}

		auto callbackIterator = std::find(this->observerCallbackList.begin(), this->observerCallbackList.end(), cb);

		if(callbackIterator != this->observerCallbackList.end())
		{
			this->observerCallbackList.erase(callbackIterator);
		}

	}

	void Observable::notify()
	{
		for(unsigned int i=0; i<this->observerInstanceList.size(); i++)
		{
			this->observerCallbackList[i](this->observerInstanceList[i]);
		}
	}

	/*void Observable::notify (Observable& observable, void * obj)
	{
		for(unsigned int i=0; i<observable.observerCallbackList.size(); i++)
		{
			observable.observerCallbackList[i](obj);
		}
	}*/
}
