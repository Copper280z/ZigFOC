const approx = @import("approx_math.zig");
const LPF = @import("lpf.zig");

pub fn PID_const_Ts(comptime Ts: f32, comptime PI: bool) type {
    const lpf_type = LPF.LPF_const_Ts(Ts);
    const _1_TS: f32 = 1 / Ts;

    return struct {
        Kp: f32 = undefined,
        Ki: f32 = undefined,
        Kd: f32 = undefined,
        lpf: lpf_type = undefined,
        output_limit: f32 = 99999,
        integral_prev: f32 = 0,
        error_prev: f32 = 0,
        output: f32 = 0,

        pub fn init(self: *@This()) void {
            const lpf = lpf_type{};
            self.lpf = lpf;
        }
        pub fn calc(self: *@This(), process_err: f32) f32 {
            const P = process_err * self.Kp;
            const I = self.integral_prev + self.Ki * Ts * 0.5 * (process_err + self.error_prev);
            var pid_out = P + I;

            if (!PI) {
                const D = self.Kd * (P - self.error_prev) * _1_TS;
                pid_out += D;
            }

            self.output = self.lpf.calc(pid_out, self.output);
            self.error_prev = process_err;
            self.output = self.constrain(self.output);

            return self.output;
        }
        pub fn set_lpf(self: *@This(), frequency: f32) void {
            self.lpf.set_lpf(frequency);
        }
        fn constrain(self: *@This(), val: f32) f32 {
            var out = val;
            if (out > self.output_limit) {
                out = self.output_limit;
            } else if (out < -self.output_limit) {
                out = -self.output_limit;
            }
            return out;
        }
    };
}
