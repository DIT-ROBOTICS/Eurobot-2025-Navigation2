#include "object_layer/object_layer.hpp"

using namespace std;

namespace custom_path_costmap_plugin{
    void ObjectLayer::OnInitialize(){}

    void ObjectLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y){
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
    }

    bool ObjectLayer::isClearable(){
        return true;
    }

    bool hasObject(){
        return !objectList.empty();
    }
}