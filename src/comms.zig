const std = @import("std");

const ParseStatus = enum { SUCCESS, BAD_START_BYTE, CONF_MISMATCH, NOT_ENOUGH_BYTES, UNHANDLED_FRAME };
