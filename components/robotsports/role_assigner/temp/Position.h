#ifndef POSITION_H
#define POSITION_H 1
#include <string>
#include "geometry.hpp"

namespace trs {

class Position  {

private:
    MRA::Geometry::Point m_vector;
    double m_time;
    double m_confidence;
    double m_rotationZ;

public:
    /**
     * Constructs a new position, with all field fully specified
     * @param aVector3d Coordinates in space
     * @param aTime Time
     * @param aConfidence Confidence
     * @param aRotationZ Rotation around z axis
     */
    Position(const MRA::Geometry::Point& aVector3d,
            double aTime = 0.0, double aConfidence = 1.0, double aRotationZ = 0.0);

    MRA::Geometry::Point getPoint() const;

    void set(double x, double y, double rz);

    double getTime() const;

    double getRotationZ() const;

    double getConfidence() const;

    /**
     * Moves the position.
     * Confidence and rotation are not changed.
     * @param offset Offset from present position
     * @param timespan Time it took to move.
     * @return Position after moving.
     */
    void move(const MRA::Geometry::Point& rOffset, double timespan);


	std::string toString() const;
};
}
#endif
