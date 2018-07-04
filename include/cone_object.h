#ifndef CONE_OBJECT_H
#define CONE_OBJECT_H

#include "base_pub_object.h"
#include <eigen3/Eigen/Core>

namespace tuw {
    class ConeObject : public BasePubObject
    {
        public:
            ConeObject(std::string &type, std::string &file_path, std::string &publisher_topic);
            virtual ~ConeObject();
            virtual bool createMsg();
        private:
            std::vector<std::vector<double>> file_contents_parsed_;
    };
}

#endif
