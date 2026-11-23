//! A do-nothing firmware root, existing only so `zig build` can prove that
//! `picozig` compiles for `thumb-freestanding` on its own.
//!
//! ## Why this file has to exist
//!
//! `boot.zig` ends its reset handler with `root.main()`, where `root` is the
//! root module of whatever compilation includes it. That is the contract
//! between the platform layer and an application: **picozig brings the chip up
//! and then calls `main`; the application provides `main`.**
//!
//! The consequence is that `boot.zig` cannot be type-checked in isolation.
//! `_reset` is an `export fn`, so it is always analysed, so `root.main()` is
//! always resolved, so *something* must supply a `main`. Without a root of its
//! own, the only way to compile picozig's startup path would be to build an
//! application — which is exactly the dependency this package is supposed not
//! to have.
//!
//! So: the smallest possible application. It satisfies the contract, forces
//! every declaration the package exports to be analysed, and does nothing.
//!
//! This is a build-time check, not a runnable image. It is never installed and
//! never flashed.
//!
//! ## It proved nothing at all until 2026-09-07
//!
//! The file used to reference its three modules from *inside* `main`:
//!
//! ```zig
//! pub fn main() noreturn {
//!     _ = &rp2350;
//!     _ = &board;
//!     _ = &boot;
//!     while (true) {}
//! }
//! ```
//!
//! **That reference to `boot` is circular, and it silently emptied the build.**
//! `_reset` is an `export fn` in a non-root module, so it is emitted only when
//! something analysed refers to that module — and here the only referrer was
//! `main`, which is itself reached only through `_reset`. Neither came into
//! existence. The linker said so every single time:
//!
//!     ld.lld: warning: cannot find entry symbol _reset; not setting start address
//!
//! It is a warning, the build went green, and nobody read it. `main` was never
//! analysed, so a smoke build whose whole purpose is analysis was compiling an
//! empty image. Every example in `example/` already had this right —
//! `blink-bare` even carries the comment "File scope, not inside `main`".
//!
//! The second half of the problem outlived the first: **referencing a module
//! does not analyse the declarations inside it.** Zig analyses a declaration
//! when something refers to *that declaration*. Only AstGen-level errors — an
//! undeclared identifier, a syntax error — surface for a whole file at once; a
//! type error inside a `pub fn` nobody calls does not. Hence `refAllRecursive`.
//!
//! Both halves are measured, not argued. With the fix in place, breaking
//! `nvic.isPending` (a free function), `i2c.base` (another) and `dma.Config.word`
//! (a method inside a struct) each fails this build; before it, none of them
//! did, and neither did a type error written directly into `main`.

const std = @import("std");
const rp2350 = @import("rp2350");
const pico_board = @import("pico_board");
const boot = @import("boot");

comptime {
    // **File scope, not inside `main`** — see the module comment above for what
    // happens otherwise, which is that nothing happens at all.
    _ = &boot;
}

/// Force every declaration reachable from `T` to be analysed.
///
/// `_ = &@field(T, name)` is what does the work. Recursing into container types
/// is what reaches the methods on `dma.Config`, `pio.Instance` and the rest,
/// which are otherwise as invisible to this file as a free function.
///
/// **Three things this deliberately does not reach**, so that a green build is
/// not read as more than it is:
///
///   * **Functions with `comptime` parameters.** There is no body to analyse
///     until the parameters are known, so `clocks.initUsb`, `pins.validate` and
///     their like are still only checked by something that instantiates them.
///     Verified by breaking `initUsb` and watching this build stay green.
///   * **Declarations behind a `comptime` branch** this board does not take.
///   * **Private declarations.** `std.meta.declarations` returns only the `pub`
///     ones, so a file-scope `const` or `fn` a module keeps to itself is
///     checked only if that module uses it somewhere. Verified both ways on the
///     same declaration: `pub` fails this build, private does not.
///   * **Anything not reachable from these three modules.**
///
/// `seen` is a comptime list rather than a depth cap: these modules refer to
/// each other's types freely — `hw.Pin` appears in half of them — so without it
/// the walk revisits the same containers combinatorially.
fn refAllRecursive(comptime T: type, comptime seen: []const type) void {
    comptime var visited = false;
    inline for (seen) |s| {
        if (s == T) visited = true;
    }
    if (visited) return;
    const next = seen ++ [_]type{T};

    inline for (comptime std.meta.declarations(T)) |decl| {
        // **Not bound to a local first.** Some declarations are runtime `var`s,
        // and binding one to a `const` here asks for a comptime value it does
        // not have. Referencing it in place is a runtime reference, which is
        // all that is wanted.
        _ = &@field(T, decl);

        // A type with no declarations is a leaf whichever kind it is, so the
        // walk simply ends there.
        if (@TypeOf(@field(T, decl)) == type) {
            switch (@typeInfo(@field(T, decl))) {
                .@"struct", .@"enum", .@"union", .@"opaque" => refAllRecursive(@field(T, decl), next),
                else => {},
            }
        }
    }
}

/// The contract `boot.zig` requires. Never called — the image is not flashed.
///
/// **The walk lives here rather than in a `comptime` block** because it has to
/// run in a runtime context: a `comptime` block would demand a comptime value
/// for every declaration it touches, and runtime `var`s do not have one. `main`
/// is reached from `_reset`, so every build still performs it.
pub fn main() noreturn {
    // Generous, and it needs to be: this walks every declaration in the chip
    // layer, the shared board helpers and the startup path in one pass.
    @setEvalBranchQuota(200_000);
    refAllRecursive(rp2350, &.{});
    refAllRecursive(pico_board, &.{});
    refAllRecursive(boot, &.{});

    while (true) {}
}
