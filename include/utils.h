#ifndef _UTILS_H_
#define _UTILS_H_

#include <memory>

#include <yaml-cpp/yaml.h>

#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer createViewer(const YAML::Node &config);

#endif // _UTILS_H_