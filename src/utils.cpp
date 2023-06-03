#include "utils.h"

pcl::visualization::PCLVisualizer createViewer(const YAML::Node &config)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setCameraPosition(-100, 0, 50, 0, 0, 0, 0, 0, 1);
    return viewer;
}