#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <vector>

#define pi 3.14159265358979323846

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double);
double rad2deg(double);
double euclidean_distance(double lat1, double lon1, double lat2, double lon2, char unit='K');

double euclidean_distance_sum(double lat, double lon, const std::vector<std::vector<double>>& coords);

#endif