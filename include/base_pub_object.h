#ifndef BASE_PUB_OBJECT_H
#define BASE_PUB_OBJECT_H

#include <string>
#include <vector>
#include <tuw_object_msgs/ObjectDetection.h>
#include <eigen3/Eigen/Core>

namespace tuw {
  class BasePubObject {
    public:
      BasePubObject(std::string &type, std::string &file_path);
      virtual ~BasePubObject();

      virtual bool read();
      virtual bool createMsg() = 0;
      //TODO: remove
      static Eigen::Matrix3d rotation_matrix_z(double rad);

    protected:
      std::string file_path_;
      std::string type_;
      std::vector<std::vector<std::string>> file_contents_;
      unsigned int nr_line_parameters;
      tuw_object_msgs::ObjectDetection msg_;

      double deg2rad(int degrees);
  };
}

#endif
