/*
 * colours.h
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */

#ifndef COMP3431_COLOURS_H_
#define COMP3431_COLOURS_H_

#include <string>
#include <vector>

namespace comp3431 {

class ColourSpace {
public:
	std::string name;
	int min[4],
		mean[4],
		max[4];

	std::vector< ColourSpace > exclusions;
	std::vector< ColourSpace > inclusions;

	ColourSpace(const std::string& name, const int channel0, const int channel1, const int channel2) : name(name)
	{
		min[0] = mean[0] = max[0] = channel0;
		min[1] = mean[1] = max[1] = channel1;
		min[2] = mean[2] = max[2] = channel2;
	}

	ColourSpace(const int channel0Min, const int channel0Max, const int channel1Min, const int channel1Max, const int channel2Min, const int channel2Max) {
		min[0] = channel0Min; min[1] = channel1Min; min[2] = channel2Min;
		max[0] = channel0Max; max[1] = channel1Max; max[2] = channel2Max;
		mean[0] = (channel0Min + channel0Max) / 2;
		mean[1] = (channel1Min + channel1Max) / 2;
		mean[2] = (channel2Min + channel2Max) / 2;
	}

	inline bool includes(const uint8_t&channel0, const uint8_t& channel1, const uint8_t& channel2) const {
		if (channel0 >= min[0] && channel0 <= max[0] && channel1 >= min[1] && channel1 <= max[1] && channel2 >= min[2] && channel2 <= max[2]) {
			for (size_t i = 0; i < exclusions.size(); ++i) {
				if (exclusions[i].includes(channel0,channel1,channel2))
					return false;
			}
			return true;
		}
		for (size_t i = 0; i < inclusions.size(); ++i) {
			if (inclusions[i].includes(channel0,channel1,channel2)) {
				return true;
			}
		}
		return false;
	}

	inline int& operator[](int idx) {
		return mean[idx];
	}

	inline const int& operator[](int idx) const {
		return mean[idx];
	}

	void addExclusion(const ColourSpace& exclude) {
		exclusions.push_back(exclude);
	}

	void addInclusion(const ColourSpace& include) {
		inclusions.push_back(include);
	}
};

} // namespace comp3431

#endif /* COMP3431_COLOURS_H_ */
