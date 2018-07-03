#ifndef DOOR_OBJECT
#define DOOR_OBJECT

#include "base_pub_object.h"
#include <eigen3/Eigen/Core>

namespace tuw {
  class DoorObject : public tuw::BasePubObject
  {
    public:
      DoorObject(std::string &type, std::string &file_path);
      virtual ~DoorObject();
      virtual bool createMsg();

    private:
      std::vector<std::vector<double>> file_contents_parsed_;
  };
}
#endif
