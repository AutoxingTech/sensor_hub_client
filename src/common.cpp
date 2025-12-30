#include "common.h"
#include "nc_runtime/nc_types.h"

UserControlMode UserControlMode_fromString(const char* str)
{
    if (strcmp(str, "auto") == 0)
        return UserControlMode::automatic;
    else if (strcmp(str, "manual") == 0)
        return UserControlMode::manual;
    else if (strcmp(str, "remote") == 0)
        return UserControlMode::remote;
    else
        return UserControlMode::unknown;
}

static const char* CONTROL_MODE_TO_STR[] = {"unknown", "auto", "manual", "remote"};

const char* UserControlMode_toString(UserControlMode mode)
{
    u32 m = (u32)mode;
    if (m < countof(CONTROL_MODE_TO_STR))
    {
        return CONTROL_MODE_TO_STR[m];
    }
    else
    {
        return "unknown";
    }
}
