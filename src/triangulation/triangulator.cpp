//
// Created by Megan Finch on 13/02/2022.
//

#include <headers/triangulation/triangulator.h>
#include <headers/triangulation/linear_triangulator.h>
#include <headers/triangulation/midpoint_triangulator.h>

cv::Ptr<Triangulator> Triangulator::create(const TriangulatorType& type) {
    switch (type) {
        case TriangulatorType::LINEAR:
            return cv::makePtr<LinearTriangulator>();
        case TriangulatorType::MIDPOINT:
            return cv::makePtr<MidpointTriangulator>();
    }
}
