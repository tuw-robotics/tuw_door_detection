#include <base_pub_object.h>
#include <memory>
#include <tuw_object_msgs/ObjectDetection.h>
#include <door_object.h>
#include <cone_object.h>

namespace tuw {

  class ObjectFacade {
  public:
    static std::unique_ptr<BasePubObject> construct(std::string type, std::string file_path, std::string publisher_topic)
    {
      if (type == tuw_object_msgs::ObjectDetection::OBJECT_TYPE_DOOR)
      {
        return std::make_unique<tuw::DoorObject>(type, file_path, publisher_topic);
      }
      else if (type == tuw_object_msgs::ObjectDetection::OBJECT_TYPE_TRAFFIC_CONE)
      {
        return std::make_unique<tuw::ConeObject>(type, file_path, publisher_topic);
      }
    }
  };
}
