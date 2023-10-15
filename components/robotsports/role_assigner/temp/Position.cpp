#include <string>
#include <ostream>
#include <sstream>
#include "Vector2D.h"
#include "Position.h"

using namespace std;
namespace trs {

    /**
     * Constructs a new position, with all field fully specified
     * @param aVector2d Coordinates in space
     * @param aTime Time
     * @param aConfidence Confidence
     * @param aRotationZ Rotation around z axis
     */
    Position::Position(const Vector2D& aVector2d,
            double aTime, double aConfidence, double aRotationZ) :
            m_vector(aVector2d),
            m_time(aTime),
            m_confidence(aConfidence),
            m_rotationZ(aRotationZ) {

    }

    void Position::set(double x, double y, double rz) {
    	m_vector.m_x = x;
    	m_vector.m_y = y;
    	m_rotationZ = rz;
    }

    Vector2D Position::getVector2D() const {
    	Vector2D vec(m_vector);
        return vec;
    }

    double Position::getTime() const {
        return m_time;
    }

    double Position::getConfidence() const {
        return m_confidence;
    }

    double Position::getRotationZ() const {
        return m_rotationZ;
    }

    /**
     * Moves the position.
     * Confidence and rotation are not changed.
     * @param offset Offset from present position
     * @param timespan Time it took to move.
     * @return Position after moving.
     */
    void Position::move(const Vector2D& offset, double timespan) {
     m_vector = m_vector.add(offset);
     m_time += timespan;
    }

    /**
     * Rotates this position around the z-axis with the specified
     * angle.
     * @param angle rotation angle
     * @return position which is rotated according to the specified
     * angle in comparison to this position.
     */
    Position Position::rotateZ(double angle) {
        return Position(m_vector, m_time,
                m_confidence, m_rotationZ + angle);
    }


	std::string Position::toString() const {
		std::stringstream buffer;
		buffer << " x:  " << m_vector.m_x << " y:  " << m_vector.m_y << " rz:  " << m_rotationZ;
		return buffer.str();
	}

	//@Override
	/*
    string toString() {
        return string.format("[vector2d=%s, time=%.3f"
                + ", confidence=%.2f, rotation=%.2f]",
                vector2D.toString(),
                time,
                confidence,
                rotationZ);
   }
   */
}; // namespace

