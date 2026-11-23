//! Multi-channel ADSR Chiptune Synthesizer Engine.
//!
//! Ported from reference C++ synthesizer for Raspberry Pi Pico Audio Pack with due gratitude.

const std = @import("std");

pub const Waveform = struct {
    pub const SINE: u8 = 1 << 0;
    pub const SQUARE: u8 = 1 << 1;
    pub const TRIANGLE: u8 = 1 << 2;
    pub const NOISE: u8 = 1 << 3;
};

pub const EnvelopeState = enum {
    idle,
    attack,
    decay,
    sustain,
    release,
};

pub const AudioChannel = struct {
    waveforms: u8 = Waveform.SINE,
    attack_ms: u32 = 10,
    decay_ms: u32 = 100,
    sustain: u16 = 32767,
    release_ms: u32 = 100,
    volume: u16 = 10000,
    frequency: f32 = 440.0,

    phase: f32 = 0.0,
    env_state: EnvelopeState = .idle,
    env_level: f32 = 0.0,
    env_counter_ms: f32 = 0.0,
    lfsr: u32 = 0xACE1,

    pub fn triggerAttack(self: *AudioChannel) void {
        self.env_state = .attack;
        self.env_counter_ms = 0.0;
    }

    pub fn triggerRelease(self: *AudioChannel) void {
        if (self.env_state != .idle) {
            self.env_state = .release;
            self.env_counter_ms = 0.0;
        }
    }

    pub fn getNextSample(self: *AudioChannel, sample_rate: f32) i16 {
        if (self.env_state == .idle or sample_rate <= 0.0) return 0;

        const dt_ms = (1.0 / sample_rate) * 1000.0;
        self.env_counter_ms += dt_ms;

        // ADSR Envelope FSM
        switch (self.env_state) {
            .idle => {
                self.env_level = 0.0;
            },
            .attack => {
                if (self.attack_ms == 0) {
                    self.env_level = 1.0;
                    self.env_state = .decay;
                    self.env_counter_ms = 0.0;
                } else {
                    self.env_level = self.env_counter_ms / @as(f32, @floatFromInt(self.attack_ms));
                    if (self.env_level >= 1.0) {
                        self.env_level = 1.0;
                        self.env_state = .decay;
                        self.env_counter_ms = 0.0;
                    }
                }
            },
            .decay => {
                const sustain_norm = @as(f32, @floatFromInt(self.sustain)) / 65535.0;
                if (self.decay_ms == 0) {
                    self.env_level = sustain_norm;
                    self.env_state = .sustain;
                } else {
                    const progress = self.env_counter_ms / @as(f32, @floatFromInt(self.decay_ms));
                    if (progress >= 1.0) {
                        self.env_level = sustain_norm;
                        self.env_state = .sustain;
                    } else {
                        self.env_level = 1.0 - progress * (1.0 - sustain_norm);
                    }
                }
            },
            .sustain => {
                self.env_level = @as(f32, @floatFromInt(self.sustain)) / 65535.0;
            },
            .release => {
                const start_level = @as(f32, @floatFromInt(self.sustain)) / 65535.0;
                if (self.release_ms == 0) {
                    self.env_level = 0.0;
                    self.env_state = .idle;
                } else {
                    const progress = self.env_counter_ms / @as(f32, @floatFromInt(self.release_ms));
                    if (progress >= 1.0) {
                        self.env_level = 0.0;
                        self.env_state = .idle;
                    } else {
                        self.env_level = start_level * (1.0 - progress);
                    }
                }
            },
        }

        // Phase advance
        const phase_inc = self.frequency / sample_rate;
        self.phase += phase_inc;
        if (self.phase >= 1.0) {
            self.phase -= @floor(self.phase);
        }

        // Raw waveform generation
        var raw_wave: f32 = 0.0;
        var active_count: f32 = 0.0;

        if ((self.waveforms & Waveform.SINE) != 0) {
            raw_wave += std.math.sin(self.phase * 2.0 * std.math.pi);
            active_count += 1.0;
        }
        if ((self.waveforms & Waveform.SQUARE) != 0) {
            raw_wave += if (self.phase < 0.5) 1.0 else -1.0;
            active_count += 1.0;
        }
        if ((self.waveforms & Waveform.TRIANGLE) != 0) {
            raw_wave += if (self.phase < 0.5)
                (4.0 * self.phase - 1.0)
            else
                (3.0 - 4.0 * self.phase);
            active_count += 1.0;
        }
        if ((self.waveforms & Waveform.NOISE) != 0) {
            // Galois LFSR
            const bit = ((self.lfsr >> 0) ^ (self.lfsr >> 2) ^ (self.lfsr >> 3) ^ (self.lfsr >> 5)) & 1;
            self.lfsr = (self.lfsr >> 1) | (bit << 15);
            raw_wave += (@as(f32, @floatFromInt(self.lfsr & 0xFFFF)) / 32767.5) - 1.0;
            active_count += 1.0;
        }

        if (active_count > 0.0) {
            raw_wave /= active_count;
        }

        const vol_norm = @as(f32, @floatFromInt(self.volume)) / 65535.0;
        const out_f = raw_wave * self.env_level * vol_norm * 32767.0;
        const clamped = std.math.clamp(out_f, -32768.0, 32767.0);
        return @intFromFloat(clamped);
    }
};

pub const song_length: usize = 384;
const HAT_NOTE: i16 = 20000;
const BASS_NOTE: i16 = 500;
const SNARE_NOTE: i16 = 6000;
const SUB_NOTE: i16 = 50;

// Gadgetoid's song notes
pub const song_tracks = [_][]const i16{
    // Channel 0: Melody
    &[_]i16{
        147, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 247, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 175, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 330, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 349, 0, 0, 0, 0, 0, 0, 0, 349, 0, 330, 0, 294, 0, 220, 0, 262, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 247, 0, 0, 0, 0, 0, 0, 0, 247, 0, 220, 0, 196, 0, 147, 0, 175, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
        147, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 247, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 175, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 330, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 349, 0, 0, 0, 0, 0, 0, 0, 349, 0, 330, 0, 294, 0, 220, 0, 262, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 247, 0, 0, 0, 0, 0, 0, 0, 247, 0, 220, 0, 196, 0, 147, 0, 175, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
        147, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 247, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 175, 0, 0, 0, 0, 0, 0, 0, 175, 0, 196, 0, 220, 0, 262, 0, 330, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 349, 0, 0, 0, 0, 0, 0, 0, 349, 0, 330, 0, 294, 0, 220, 0, 262, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 247, 0, 0, 0, 0, 0, 0, 0, 247, 0, 262, 0, 294, 0, 392, 0, 440, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0,
    },
    // Channel 1: Rhythm
    &[_]i16{
        294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 392, 0, 523, 0, 659, 0, 523, 0, 392, 0, 523, 0, 659, 0, 523, 0, 698, 0, 587, 0, 440, 0, 587, 0, 698, 0, 587, 0, 440, 0, 587, 0, 523, 0, 440, 0, 330, 0, 440, 0, 523, 0, 440, 0, 330, 0, 440, 0, 349, 0, 294, 0, 220, 0, 294, 0, 349, 0, 294, 0, 220, 0, 294, 0, 262, 0, 247, 0, 220, 0, 175, 0, 165, 0, 147, 0, 131, 0, 98, 0,
        294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 392, 0, 523, 0, 659, 0, 523, 0, 392, 0, 523, 0, 659, 0, 523, 0, 698, 0, 587, 0, 440, 0, 587, 0, 698, 0, 587, 0, 440, 0, 587, 0, 523, 0, 440, 0, 330, 0, 440, 0, 523, 0, 440, 0, 330, 0, 440, 0, 349, 0, 294, 0, 220, 0, 294, 0, 349, 0, 294, 0, 220, 0, 294, 0, 262, 0, 247, 0, 220, 0, 175, 0, 165, 0, 147, 0, 131, 0, 98, 0,
        294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 294, 0, 440, 0, 587, 0, 440, 0, 392, 0, 523, 0, 659, 0, 523, 0, 392, 0, 523, 0, 659, 0, 523, 0, 698, 0, 587, 0, 440, 0, 587, 0, 698, 0, 587, 0, 440, 0, 587, 0, 523, 0, 440, 0, 330, 0, 440, 0, 523, 0, 440, 0, 330, 0, 440, 0, 349, 0, 294, 0, 220, 0, 294, 0, 349, 0, 294, 0, 220, 0, 294, 0, 262, 0, 247, 0, 220, 0, 175, 0, 165, 0, 147, 0, 131, 0, 98, 0,
    },
    // Channel 2: Drums
    &[_]i16{
        BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0,
        BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0,
        BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, BASS_NOTE, -1, BASS_NOTE, -1, 0, 0, 0, 0, 0, 0, SNARE_NOTE, 0, -1, 0, 0, 0, 0, 0,
    },
    // Channel 3: Hi-Hat
    &[_]i16{
        HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1,
        HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1,
        HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1, HAT_NOTE, -1,
    },
    // Channel 4: Sub-Bass
    &[_]i16{
        SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
        SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
        SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, SUB_NOTE, -1, SUB_NOTE, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    },
};

pub const Synth = struct {
    channels: [5]AudioChannel = @as([5]AudioChannel, @splat(.{})),
    last_beat: usize = 9999,

    pub fn init() Synth {
        var self = Synth{};

        // Track 0: Melody
        self.channels[0].waveforms = Waveform.TRIANGLE | Waveform.SQUARE;
        self.channels[0].attack_ms = 16;
        self.channels[0].decay_ms = 168;
        self.channels[0].sustain = 0x5FFF;
        self.channels[0].release_ms = 168;
        self.channels[0].volume = 10000;

        // Track 1: Rhythm
        self.channels[1].waveforms = Waveform.SINE | Waveform.SQUARE;
        self.channels[1].attack_ms = 38;
        self.channels[1].decay_ms = 300;
        self.channels[1].sustain = 0;
        self.channels[1].release_ms = 0;
        self.channels[1].volume = 12000;

        // Track 2: Drums
        self.channels[2].waveforms = Waveform.NOISE;
        self.channels[2].attack_ms = 5;
        self.channels[2].decay_ms = 10;
        self.channels[2].sustain = 16000;
        self.channels[2].release_ms = 100;
        self.channels[2].volume = 18000;

        // Track 3: Hi-Hat
        self.channels[3].waveforms = Waveform.NOISE;
        self.channels[3].attack_ms = 5;
        self.channels[3].decay_ms = 5;
        self.channels[3].sustain = 8000;
        self.channels[3].release_ms = 40;
        self.channels[3].volume = 8000;

        // Track 4: Sub-Bass
        self.channels[4].waveforms = Waveform.SQUARE;
        self.channels[4].attack_ms = 10;
        self.channels[4].decay_ms = 100;
        self.channels[4].sustain = 0;
        self.channels[4].release_ms = 500;
        self.channels[4].volume = 12000;

        return self;
    }

    pub fn updatePlayback(self: *Synth, tick_ms: u64) void {
        const beat: usize = @intCast((tick_ms / 100) % song_length);
        if (beat == self.last_beat) return;
        self.last_beat = beat;

        for (0..5) |i| {
            const track = song_tracks[i];
            if (beat < track.len) {
                const note = track[beat];
                if (note > 0) {
                    self.channels[i].frequency = @floatFromInt(note);
                    self.channels[i].triggerAttack();
                } else if (note == -1) {
                    self.channels[i].triggerRelease();
                }
            }
        }
    }

    pub fn fillBuffer(self: *Synth, out: []u32, sample_rate: f32) void {
        for (out) |*word| {
            var mix_l: i32 = 0;
            var mix_r: i32 = 0;

            for (&self.channels) |*ch| {
                const s = ch.getNextSample(sample_rate);
                mix_l += s;
                mix_r += s;
            }

            const clamped_l: i16 = @intCast(std.math.clamp(mix_l, -32768, 32767));
            const clamped_r: i16 = @intCast(std.math.clamp(mix_r, -32768, 32767));

            const u_l: u16 = @bitCast(clamped_l);
            const u_r: u16 = @bitCast(clamped_r);

            word.* = (@as(u32, u_l) << 16) | @as(u32, u_r);
        }
    }
};
