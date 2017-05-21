/**
 * @file	Event.hpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Event class
 */

#ifndef INC_EVENT_HPP_
#define INC_EVENT_HPP_

#include "Observable.hpp"

#include <vector>
#include <algorithm>

/*----------------------------------------------------------------------------*/
/* Types		   		                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Redefine observer callback
 */
typedef Observer::ObserverCallback EventCallback;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
	/**
	 * @class Event
	 * @brief Event is a concrete implementation of Observable class
	 *
	 * HOWTO:
	 * 	An object can subscribe to an event raised by another object using "+=" operator
	 * 	and unsubscribe to it using '-=' operator.
	 *
	 */
	class Event : public Utils::Observable
	{
	public:

		/**
		 * @brief Event constructor
		 */
		Event() : Observable()
		{
		}

		/**
		 * @brief Add a new event callback to the callback list
		 *
		 * This method shouldn't be used if you expect your object instance passed as argument
		 * when event is raised. Use subscribe() method instead.
		 *
		 * @param cb : Event callback
		 * @return Event object reference
		 */
		Event& operator += (EventCallback cb);

		/**
		 * @brief Remove a callback from the callback list
		 * @param cb : Event callback
		 * @return Event object reference
		 */
		Event& operator -= (EventCallback cb);

		/**
		 * @brief Raise an event
		 * @return Event object reference
		 */
		Event& operator () ()
		{
			//this->notify(*this);
			this->notify();

			return *this;
		}

	private:

	};
}

#endif /* INC_EVENT_HPP_ */
