const approx = @import("approx_math.zig");
// #define LOWPASS( output, input, c_lowpass)  (output += (c_lowpass) * ((input) - (output)))
// https://tomroelandts.com/articles/low-pass-single-pole-iir-filter
// https://dsp.stackexchange.com/questions/54086/single-pole-iir-low-pass-filter-which-is-the-correct-formula-for-the-decay-coe

// pub fn LPF(
//     comptime lpf: type,
//     comptime calc: fn (lpf: *lpf, input: f32, output: f32) f32,
// ) type {
//     return struct {
//         lpf: lpf,
//         pub fn Calc(L: @This(), input: f32, output: f32) f32 {
//             return calc(L.lpf, input, output);
//         }
//     };
// }

pub fn LPF_const_Ts(comptime Ts: f32) type {
    return struct {
        lpf_coeff: f32 = 0.99,

        pub fn calc(L: *@This(), input: f32, output: f32) f32 {
            return output + (L.lpf_coeff * (input - output));
        }

        pub fn set_lpf(L: *@This(), frequency: f32) void {
            // const y = 1 - approx._cos(freq * Ts);
            // L.lpf_coeff = -y + @sqrt(y * y + 2 * y);
            const y = @tan((frequency * Ts) / 2);
            L.lpf_coeff = 2 * y / (y + 1);
        }
    };
}
