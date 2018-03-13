#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_DTSTATUS_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_DTSTATUS_H

#include <DetourStatus.h>

#include <ostream>

namespace DetourNavigator
{
    struct WriteDtStatus
    {
        dtStatus status;
    };

    std::ostream& operator <<(std::ostream& stream, const WriteDtStatus& value);

    void checkDtStatus(dtStatus status, const char* call, int line);

    void checkDtResult(bool result, const char* call, int line);
}

#define OPENMW_CHECK_DT_STATUS(call) \
    do { DetourNavigator::checkDtStatus((call), #call, __LINE__); } while (false)

#define OPENMW_CHECK_DT_RESULT(call) \
    do { DetourNavigator::checkDtResult((call), #call, __LINE__); } while (false)

#endif
