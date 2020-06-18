#include "RobotTypes.h"

Print &operator<<(Print &stream, const ActuatorPositionVector &vec)
{
    stream << "ActPos: ";
    for (auto e : vec)
    {
        stream << e << " ";
    }
    return stream;
}