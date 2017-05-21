/**
 * @file	Observable.hpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Observable class
 */

#ifndef INC_OBSERVABLE_HPP_
#define INC_OBSERVABLE_HPP_

#include <vector>
#include <algorithm>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
	/**
	 * @brief Observer callback
	 * @param obj : any object instance
	 */
	typedef void (*ObserverCallback) (void * obj);

	/**
	 * @brief Observer instance
	 */
	void * obj;
}Observer;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Utils
 */
namespace Utils
{
	/**
	 * @class Observable
	 * @brief Abstract Observable class can be used to notify objects called "obervers"
	 *
	 * HOWTO :
	 * To be notified, an "observer" has to register itself by calling Subscribe() method
	 * passing as argument a callback which will be called when the observable notify its obersvers.
	 * Unsubscription can be achieved by calling Unsubscribe() method.
	 */
	class Observable
	{
	public:

		/**
		 * @brief Default constructor;
		 */
		Observable();

		/**
		 * @brief Subscribe to observable notifications
		 * @param cb : Callback to call on observable notifications
		 */
		void Subscribe (void * observer, Observer::ObserverCallback cb);

		/**
		 * @brief Unsubscribe to observable notifications
		 * @param cb : Callback which where called on observable notifications
		 */
		void Unsubscribe (void * observer, Observer::ObserverCallback cb);

	protected:

		/**
		 * @pure
		 * @brief Virtual pure destructor
		 */
		virtual ~Observable() = 0;

		/**
		 * @brief Notification method - Used by child class
		 * @param observable
		 * @param obj
		 */
		//void notify (Observable& observable);
		void notify ();
	private:

		/**
		 * @private
		 * @brief Observer callback list used by notifications
		 */
		std::vector<Observer::ObserverCallback> observerCallbackList;

		/**
		 * @private
		 * @brief Observer instance lsit used by notifications
		 */
		std::vector<void*> observerInstanceList;
	};
}

#endif /* INC_OBSERVABLE_HPP_ */
