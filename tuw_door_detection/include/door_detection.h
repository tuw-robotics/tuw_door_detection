#ifndef DOOR_DETECTION_HPP
#define DOOR_DETECTION_HPP

#include <tuw_geometry/measurement_laser.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>

namespace tuw {

    class DoorDetection;

    using DoorDetectionPtr = std::shared_ptr<DoorDetection>;
    using DoorDetectionConstPtr = std::shared_ptr<DoorDetection const>;

    class DoorDetection : public Measurement {
    public:
        DoorDetection();

        double &response();

        double &responseNormalized();

        bool &validDetection();

        const double &response() const;

        const double &responseNormalized() const;

        const bool &validDetection() const;

        tuw_object_msgs::ObjectWithCovariance toMsg(int id);

        void link(DoorDetectionPtr &_this, DoorDetectionPtr &_other);

        friend std::ostream &operator<<(std::ostream &output, const DoorDetection &d);

    protected:
        double response_;
        double response_normalized_;
        bool valid_;
        MeasurementLaser::Beam
        std::weak_ptr<DoorDetection> next_;
        std::weak_ptr<DoorDetection> previous_;

        void next(DoorDetectionPtr &_other);

        DoorDetectionPtr next();

        void previous(DoorDetectionPtr &_other);

        DoorDetectionPtr previous();

    };

};

#endif
