#include "dtstatus.hpp"
#include "exceptions.hpp"
#include "debug.hpp"

#include <sstream>
#include <vector>

namespace
{
#define OPENMW_DT_STATUS(status) {status, #status}

    static const std::vector<std::pair<dtStatus, const char*>> dtStatuses {
        OPENMW_DT_STATUS(DT_FAILURE),
        OPENMW_DT_STATUS(DT_SUCCESS),
        OPENMW_DT_STATUS(DT_IN_PROGRESS),
        OPENMW_DT_STATUS(DT_WRONG_MAGIC),
        OPENMW_DT_STATUS(DT_WRONG_VERSION),
        OPENMW_DT_STATUS(DT_OUT_OF_MEMORY),
        OPENMW_DT_STATUS(DT_INVALID_PARAM),
        OPENMW_DT_STATUS(DT_BUFFER_TOO_SMALL),
        OPENMW_DT_STATUS(DT_OUT_OF_NODES),
        OPENMW_DT_STATUS(DT_PARTIAL_RESULT),
    };

#undef OPENMW_DT_STATUS
}

namespace DetourNavigator
{
    std::ostream& operator <<(std::ostream& stream, const WriteDtStatus& value)
    {
        for (const auto& status : dtStatuses)
            if (value.status & status.first)
                stream << status.second << " ";
        return stream;
    }

    void checkDtStatus(dtStatus status, const char* call, int line)
    {
        if (!dtStatusSucceed(status))
        {
            std::ostringstream message;
            message << call << " failed with status=" << WriteDtStatus {status} << " at " __FILE__ ":" << line;
            throw NavigatorException(message.str());
        }
    }

    void checkDtResult(bool result, const char* call, int line)
    {
        if (!result)
        {
            std::ostringstream message;
            message << call << " failed at " __FILE__ ":" << line;
            throw NavigatorException(message.str());
        }
    }
}
