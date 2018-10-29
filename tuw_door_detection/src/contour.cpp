//
// Created by felix on 29.10.18.
//

#include <contour.h>

tuw::Contour::Contour() : length_(0.0), length_cache_uptodate_{false} {}

tuw::Contour::Beam::Beam(double _range, double _angle, tuw::Point2D _end_point) {
  range = _range;
  angle = _angle;
  end_point = _end_point;
}

std::shared_ptr<tuw::Contour::Beam> tuw::Contour::Beam::make_beam(double range, double angle, tuw::Point2D end_point) {
  return std::make_shared<Beam>(range, angle, end_point);
}

void tuw::Contour::push_back(std::shared_ptr<tuw::Contour::Beam> beam) {
  if (beams_.size()) {
    length_ += beams_.back()->end_point.distanceTo(beam->end_point);
  }
  beams_.push_back(beam);
  length_cache_uptodate_ = true;
}

double tuw::Contour::length() {
  if (beams_.size() <= 2) {
    return 0;
  }

  if (length_cache_uptodate_) {
    return length_;
  } else {
    length_ = 0;
    for (size_t i = 0; i < beams_.size() - 1; i++) {
      length_ += beams_[i]->end_point.distanceTo(beams_[i + 1]->end_point);
    }
  }
}

void tuw::Contour::render(tuw::WorldScopedMaps &map, cv::Mat &img, cv::Scalar &color) {
  for (std::vector<std::shared_ptr<Contour::Beam>>::const_iterator it_beam = begin();
       it_beam != (end() - 1); ++it_beam) {
    map.line(img, (*it_beam)->end_point, (*(it_beam + 1))->end_point, color, 2);
  }
}

size_t tuw::Contour::corners()
{
  return 0;
}
