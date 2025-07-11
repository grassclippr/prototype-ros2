#pragma once
#include "./rover.h"
#include "./base.h"
#include "./cli.h"

enum DeviceRole {
    ROLE_UNKNOWN = 0,
    ROLE_ROVER = 1,
    ROLE_BASESTATION = 2,
};
