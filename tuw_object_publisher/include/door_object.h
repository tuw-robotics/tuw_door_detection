#ifndef DOOR_OBJECT_H
#define DOOR_OBJECT_H

#include "base_pub_object.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace tuw {

  enum class DoorObjectConstants { Position_x = 0,
                                 Position_y = 1, 
                                 Position_z = 2, 
                                 width=3, 
                                 height=4, 
                                 angle_w = 5, 
                                 angle_d = 6, 
                                 leaves = 7, 
                                 clockwise=8};

  enum class DoorObjectShapeVariables {
    width = 0,
    height = 1,
    angle_w = 2,
    angle_d = 3,
    leaves = 4,
    clockwise = 5
  };
  
  class DoorObject : public tuw::BasePubObject
  {
    public:
      DoorObject(std::string type, std::string file_path, std::string publisher_topic);
      DoorObject(std::string &type, tf::Transform &_tf);
      virtual ~DoorObject();
      virtual bool createMsg();
      virtual bool createMsgFromView(Eigen::Vector3d &position, Eigen::Quaterniond &orientation);

    private:
      std::vector<std::vector<double>> file_contents_parsed_;
  };
}
#endif
