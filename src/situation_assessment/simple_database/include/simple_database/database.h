/* 
 * File:   Database.h
 * Author: mfiore
 *
 * this class rerepsents a simple fake database, a vector of elements.
 * Created on February 2, 2015, 6:31 PM
 */

#ifndef DATABASE_H
#define	DATABASE_H

#include <map>
#include <string>
#include <vector>
#include "simple_database/database_element.h"
#include <algorithm>
#include <iostream>
#include "ros/ros.h"

#include <boost/thread/locks.hpp> 
#include <boost/thread/mutex.hpp>

using namespace std;

class Database {
/**
    * this class rerepsents a simple fake database, a vector of elements.
*/
public:
    /**
     * Constructor of the class
     */
    Database();
    Database(const Database& orig);
    virtual ~Database();

    /**
     * Adds an element to the database
     * @param element element to add
     */
    void addElement(DatabaseElement element);
    /**
     * Gets elements corresponding to the DatabaseElement received in input.
     * Empty field in the element will be treated as a jolly
     * @param element element to search
     * @return a vector of found elements
     */
    vector<DatabaseElement> getElements(DatabaseElement element);
    /**
     * Removes an element in the database
     * @param element element to remove
     */
    void removeElement(DatabaseElement element);


private:

    vector<DatabaseElement> database_;
	boost::mutex db_mutex_;


};

#endif	/* DATABASE_H */

