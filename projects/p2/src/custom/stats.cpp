#include "stats.h"

using namespace std;


vector<double> extractNeighborhoodSizes(vector<cv::KeyPoint>& keypoints)
{
  vector<double> sizes;

  for (auto& keypoint : keypoints)
    sizes.push_back(keypoint.size);

  return sizes;
}

/* Calculate the p-th percentile of a distribution of values in an sorted vector.
 */
double percentile(vector<double>& vec, float p)
{
  int vec_size = vec.size();
  double res;

  switch(vec_size % 2)
  {
    case 0:
      res = vec[vec_size * p];
      break;

    case 1:
      res = 0.5 * (vec[floor(vec_size * p)] + vec[floor(vec_size * p) + 1]);
      break;
  }

  return res;
}

/* Calculate statistics on the distributions of keypoints' neighborhood sizes.
 */
Stats calculateStatistics(vector<double>& vec)
{
  Stats stats;
  stats.vec_size = vec.size();

  sort(vec.begin(), vec.end());

  double mean = accumulate(vec.begin(), vec.end(), 0.) / stats.vec_size;

  double squared_dev = accumulate(vec.begin(), vec.end(), 0., 
    [mean](double sum, const double x) { return sum + pow(x - mean, 2); });

  stats.mean = mean;
  stats.std_dev = sqrt(squared_dev / (stats.vec_size - 1));  // Sample standard deviation

  stats.median = percentile(vec, .50);

  stats.min = vec[0];
  stats.max = vec.back();

  stats.p25 = percentile(vec, .25);
  stats.p75 = percentile(vec, .75);

  return stats;
}

void printStatistics(Stats& s)
{
  cout << endl;
  cout << "\t" << "Number of keypoints: " << s.vec_size << endl;
  cout << "\t" << "Mean: " << s.mean << endl;
  cout << "\t" << "Median: " << s.median << endl;
  cout << "\t" << "Standard deviation: " << s.std_dev << endl;

  cout << "\t" << "Min: " << s.min << endl;
  cout << "\t" << "Max: " << s.max << endl;
  cout << "\t" << "Range: " << (s.max - s.min) << endl;

  cout << "\t" << "25th percentile: " << s.p25 << endl;
  cout << "\t" << "75th percentile: " << s.p75 << endl;
  cout << "\t" << "IQR: " << (s.p75 - s.p25) << endl;
  cout << endl;
}