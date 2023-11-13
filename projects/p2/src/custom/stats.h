#ifndef stats_h
#define stats_h

#include <vector>
#include <opencv2/core.hpp>


struct Stats {  // Holds basic statistics of keypoints' neighborhood distribution

  int vec_size;

  double min, max;
  
  double mean, std_dev;
  double median; 
  
  double p25, p75;
};

double percentile(std::vector<cv::KeyPoint>& vec, float p);

std::vector<double> extractNeighborhoodSizes(std::vector<cv::KeyPoint>& keypoints);

Stats calculateStatistics(std::vector<double>& vec);

void printStatistics(Stats& s);

#endif /* stats_h */