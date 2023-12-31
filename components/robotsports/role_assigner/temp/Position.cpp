#include <string>
#include <ostream>
#include <sstream>
#include "Position.h"
#include "geometry.hpp"

using namespace std;
namespace MRA {

    /**
     * Constructs a new position, with all field fully specified
     * @param aMRA::Geometry::Point Coordinates in space
     * @param aTime Time
     * @param aConfidence Confidence
     * @param aRotationZ Rotation around z axis
     */
    Position::Position(const MRA::Geometry::Point& point,
            double aTime, double aConfidence, double aRotationZ) :
            m_vector(point),
            m_time(aTime),
            m_confidence(aConfidence),
            m_rotationZ(aRotationZ) {

    }

    MRA::Geometry::Point Position::getPoint() const {
        MRA::Geometry::Point vec(m_vector);
        return vec;
    }

    void Position::set(double x, double y, double rz) {
        m_vector.x = x;
        m_vector.y = y;
        m_rotationZ = rz;
    }

    double Position::getTime() const {
        return m_time;
    }

    double Position::getRotationZ() const {
        return m_rotationZ;
    }

    double Position::getConfidence() const {
        return m_confidence;
    }
    /**
     * Moves the position.
     * Confidence and rotation are not changed.
     * @param offset Offset from present position
     * @param timespan Time it took to move.
     * @return Position after moving.
     */
    void Position::move(const MRA::Geometry::Point& offset, double timespan) {
     m_vector += offset;
     m_time += timespan;
    }
}; // namespace

