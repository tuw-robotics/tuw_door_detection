#ifndef DOOR_DETECTION_HPP
#define DOOR_DETECTION_HPP

#include <tuw_object_msgs/ObjectWithCovariance.h>
#include <memory>
#include <tuw_geometry/point2d.h>

namespace tuw {

    class DoorDetection;

    using DoorDetectionPtr = std::shared_ptr<DoorDetection>;
    using DoorDetectionConstPtr = std::shared_ptr<DoorDetection const>;

    class DoorDetection {

    public:
        struct Beam {
            Point2D point;
            double length;
        };

        DoorDetection();

        double &response();

        double &responseNormalized();

        bool &validDetection();

        const double &response() const;

        const double &responseNormalized() const;

        const bool &validDetection() const;

        tuw_object_msgs::ObjectWithCovariance toMsg(int id);

        void link(DoorDetectionPtr &_this, DoorDetectionPtr &_other);

        Beam &operator[](size_t i);

        const Beam &operator[](size_t id) const;

        const std::size_t size() const {
          return end_points.size();
        }

        void resize(std::size_t sz) {
          end_points.resize(sz);
        }

        friend std::ostream &operator<<(std::ostream &output, const DoorDetection &d);

    protected:
        std::vector<Beam> end_points;
        double response_;
        double response_normalized_;
        bool valid_;
        std::weak_ptr<DoorDetection> next_;
        std::weak_ptr<DoorDetection> previous_;

        void next(DoorDetectionPtr &_other);

        DoorDetectionPtr next();

        void previous(DoorDetectionPtr &_other);

        DoorDetectionPtr previous();

    };

};

#endif
