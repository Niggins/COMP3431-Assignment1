/*
 * path.h
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */

#ifndef COMP3431_PATH_H_
#define COMP3431_PATH_H_

#include <geometry_msgs/Point.h>

namespace comp3431 {

class Path {
public:
	std::vector< geometry_msgs::Point > points;

	Path();
};

} // namespace comp3431



#endif /* PATH_H_ */
