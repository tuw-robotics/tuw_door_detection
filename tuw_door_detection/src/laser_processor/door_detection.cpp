
#include <laserproc/door_detection.h>

using namespace tuw;
using namespace tuw::door_laser_proc;

DoorDetection::DoorDetection() {
  this->response_ = std::nan("uninitialized");
  this->response_normalized_ = std::nan("uninitialized");
  this->valid_ = false;
}

DoorDetection::Beam &DoorDetection::operator[](size_t id) {
  return this->end_points[id];
}

const DoorDetection::Beam &DoorDetection::operator[](size_t id) const {
  return this->end_points[id];
}

tuw_object_msgs::ObjectWithCovariance DoorDetection::toMsg(int id) {
  tuw_object_msgs::ObjectWithCovariance dd;

  //TODO refine accordingly (more beams per door measurement i.e. all beams connected to the door)
  dd.object.pose.position.x = this->end_points[0].point.x();
  dd.object.pose.position.y = this->end_points[0].point.y();
  dd.object.pose.position.z = 0;

  dd.object.pose.orientation.x = 0;
  dd.object.pose.orientation.y = 0;
  dd.object.pose.orientation.z = 0;
  dd.object.pose.orientation.w = 1;

  //TODO: dummy for now
  dd.object.shape = tuw_object_msgs::Object::SHAPE_CONE;
  dd.object.ids = {
      static_cast<int>(id)
  };
  dd.object.ids_confidence = {0};
  dd.object.shape_variables = {0.1, 1, 0.5};

  return std::move(dd);
}

namespace tuw {

  namespace door_laser_proc {

    std::ostream &operator<<(std::ostream &output, const DoorDetection &d) {
      output << "Beams: {(" << d[0].point.x() << ", " << d[0].point.y() << ")";
      for (int i = 1; i < d.size(); ++i) {
        output << ", (" << d[i].point.x() << ", " << d[i].point.y() << ")";
      }
      output << "}" << "\n";
      output << "resp: " << std::to_string(d.response()) << "\n";
      output << "resp_n: " << std::to_string(d.responseNormalized()) << "\n";
      output << "valid_det: " << (d.validDetection() ? "true" : "false") << "\n";
      output << "next: " << (d.next_.lock() ? "true" : "false") << std::endl;
      output << "previous: " << (d.previous_.lock() ? "true" : "false") << std::endl;

      return output;
    }

    double &DoorDetection::responseNormalized() {
      return response_normalized_;
    }

    double &DoorDetection::response() {
      return response_;
    }

    bool &DoorDetection::validDetection() {
      return valid_;
    }

    const double &DoorDetection::response() const {
      return response_;
    }

    const double &DoorDetection::responseNormalized() const {
      return response_normalized_;
    }

    const bool &DoorDetection::validDetection() const {
      return valid_;
    }

    void DoorDetection::next(DoorDetectionPtr &_other) {
      next_ = _other;
    }

    DoorDetectionPtr DoorDetection::next() {
      return next_.lock();
    }

    void DoorDetection::previous(DoorDetectionPtr &_other) {
      previous_ = _other;
    }

    DoorDetectionPtr DoorDetection::previous() {
      return previous_.lock();
    }

    void DoorDetection::link(DoorDetectionPtr &_this, DoorDetectionPtr &_other) {
      _this->next(_other);
      _other->previous(_this);
    }
  }
}
