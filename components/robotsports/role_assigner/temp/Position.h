#ifndef POSITION_H
#define POSITION_H 1
#include "Vector2D.h"
#include <string>

namespace trs {

class Position  {

private:
    Vector2D m_vector;
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
    Position(const Vector2D& aVector3d,
            double aTime = 0.0, double aConfidence = 1.0, double aRotationZ = 0.0);

    Vector2D getVector2D() const;

    double getTime() const;

    double getConfidence() const;

    double getRotationZ() const;

    void set(double x, double y, double rz);

    /**
     * Moves the position.
     * Confidence and rotation are not changed.
     * @param offset Offset from present position
     * @param timespan Time it took to move.
     * @return Position after moving.
     */
    void move(const Vector2D& rOffset, double timespan);

    /**
     * Rotates this position around the z-axis with the specified
     * angle.
     * @param angle rotation angle
     * @return position which is rotated according to the specified
     * angle in comparison to this position.
     */
    Position rotateZ(double angle);

	std::string toString() const;
};
}
#endif
