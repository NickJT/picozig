//! Everything platform-specific about talking to the inventor board lives here;

const rp2350 = @import("rp2350");

const timer = rp2350.timer;

pub const Bus = struct {
    /// How long `waitReady` will poll SO before giving up and proceeding.
    pub const ready_timeout_us: u32 = 2_000;

    /// Set to true when `waitReady` hit its ceiling. Nothing here acts on it —
    /// it is a diagnostic for the self-test block, because a link that works but
    /// times out every transfer is a bug hiding behind a symptom-free result.
    ready_timeouts: u32 = 0,

    pub fn init(self: *Bus) void {
        self.ready_timeouts = 0;
    }

    pub fn delayUs(self: *Bus, us: u32) void {
        _ = self;
        timer.delayUs(us);
    }
};
