#pragma once

enum class UserControlMode
{
    unknown = 0,
    automatic = 1,
    manual = 2,
    remote = 3,
};

UserControlMode UserControlMode_fromString(const char* str);
const char* UserControlMode_toString(UserControlMode mode);
