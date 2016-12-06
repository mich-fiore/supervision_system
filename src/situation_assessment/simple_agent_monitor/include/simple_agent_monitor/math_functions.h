/*
 * MathFunctions.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 *
 *	Contains some useful mathematical functions for situation assessment
 */


#ifndef MATHFUNCTIONS_H
#define MATHFUNCTIONS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


#include <math.h>
using namespace std;


/**
 * Calculates the distance between two points
 * @param  p1 starting point
 * @param  p2 ending point
 * @return    distance between the points
 */
double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

/**
 * Calculates if an entity is moving
 * @param  vp a vector of positions of an entity
 * @return    true if the entity is moving
 */
bool isMoving(vector<geometry_msgs::Point> vp);

/**
 * Calculates if an entity is facing another entity
 * @param  p1 pose of first entity
 * @param  p2 pose of second entity
 * @return    true if p1 is oriented toward p2
 */
bool isFacing(geometry_msgs::Pose p1, geometry_msgs::Point p2);

/**
 * Utility function to check if an entity is in a specific angle centered on another entity
 * @param  p1             first entity
 * @param  p2             second entity
 * @param  angleDir       direction of the angle in radiants
 * @param  angleThreshold width of the angle
 * @return                a value different than zero if p2 is in the angle
 */
double isInAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir, double angleThreshold);

/**
 * Returns the relative angle between p1 and p2, considering a chosen direction
 * @param  p1       first entity
 * @param  p2       second entity
 * @param  angleDir direction of the angle
 * @return          relative angle between the two entities
 */
double relativeAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir);

#endif
