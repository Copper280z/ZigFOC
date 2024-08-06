const sine_array = [65]u16{ 0, 804, 1608, 2411, 3212, 4011, 4808, 5602, 6393, 7180, 7962, 8740, 9512, 10279, 11039, 11793, 12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531, 18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595, 23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791, 27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957, 30274, 30572, 30853, 31114, 31357, 31581, 31786, 31972, 32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758, 32768 };
pub const _PI: f32 = 3.14159274;
pub const _2PI: f32 = 2.0 * _PI;
pub const _PI_2: f32 = _PI / 2.0;
pub const _SQRT3_2: f32 = 0.86602540378;
pub const _1_SQRT3: f32 = 1.0 / @sqrt(3.0);
pub const _2_SQRT3: f32 = 2.0 / @sqrt(3.0);

pub fn _sin(a: f32) f32 {
    // 16bit integer array for sine lookup. interpolation is used for better precision
    // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
    // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
    //   int32_t t1, t2;
    const i: u32 = @intFromFloat(a * 65535.0 / _2PI);
    const frac: i32 = @bitCast(i & 0xff);
    const i_s: u32 = (i >> 8) & 0xff;
    var t1: i32 = undefined;
    var t2: i32 = undefined;
    if (i_s < 64) {
        t1 = @as(i32, sine_array[i_s]);
        t2 = @as(i32, sine_array[i_s + 1]);
    } else if (i_s < 128) {
        t1 = @as(i32, sine_array[128 - i_s]);
        t2 = @as(i32, sine_array[127 - i_s]);
    } else if (i_s < 192) {
        t1 = -@as(i32, sine_array[i_s - 128]);
        t2 = -@as(i32, sine_array[i_s - 127]);
    } else {
        t1 = -@as(i32, sine_array[256 - i_s]);
        t2 = -@as(i32, sine_array[255 - i_s]);
    }

    return @as(f32, comptime 1.0 / 32768.0) * @as(f32, @floatFromInt(t1 + (((t2 - t1) * frac) >> 8)));
}

// function approximating cosine calculation by using fixed size array
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
pub fn _cos(a: f32) f32 {
    const a_sin: f32 = a + _PI_2;
    if (a_sin > _2PI) {
        return _sin(a_sin - _2PI);
    } else {
        return _sin(a_sin);
    }
}

pub fn _sincos(a: f32, s: *f32, c: *f32) void {
    s.* = _sin(a);
    c.* = _cos(a);
}

pub fn fmax(a: f32, b: f32) f32 {
    if (a > b) {
        return a;
    } else {
        return b;
    }
}

pub fn fmin(a: f32, b: f32) f32 {
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

// // fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
// // Via Odrive project
// // https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// // This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
// // The origin for Odrive atan2 is public domain. Thanks to Odrive for making
// // it easy to borrow.
// __attribute__((weak)) float _atan2(float y, float x) {
//     // a := min (|x|, |y|) / max (|x|, |y|)
//     float abs_y = fabsf(y);
//     float abs_x = fabsf(x);
//     // inject FLT_MIN in denominator to avoid division by zero
//     float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
//     // s := a * a
//     float s = a * a;
//     // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
//     float r =
//         ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
//     // if |y| > |x| then r := 1.57079637 - r
//     if (abs_y > abs_x) r = 1.57079637f - r;
//     // if x < 0 then r := 3.14159274 - r
//     if (x < 0.0f) r = 3.14159274f - r;
//     // if y < 0 then r := -r
//     if (y < 0.0f) r = -r;

//     return r;
//   }
