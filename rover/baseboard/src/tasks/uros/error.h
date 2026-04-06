#pragma once

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

const char *rcl_ret_to_string(rcl_ret_t rc);
void log_rcl_error(const char *what, rcl_ret_t rc);
