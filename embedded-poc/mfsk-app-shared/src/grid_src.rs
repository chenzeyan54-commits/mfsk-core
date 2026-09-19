//! Where the slot grid's phase comes from — the operator's choice,
//! persisted beside `boot_mode`.
//!
//! The receiver has two ways to know when a slot starts, and they do
//! not overlap: UTC from NTP, or the air itself — the DT of what
//! decodes, bootstrapped by `ft8::acquire`'s cold acquisition. Which
//! one is right is a property of *where the station is*, not of the
//! software: on a hilltop with no network there is no NTP to wait for,
//! and at home there is no reason to spend two minutes acquiring a
//! phase the clock already knows.
//!
//! Until now the code decided by itself — it starts NTP whenever WiFi
//! is up and lets `clock_is_disciplined()` hand the phase to UTC the
//! moment it succeeds. That is right at home and wrong in the field,
//! where the association may take a minute, fail, or (worse) succeed
//! against a hotspot with no route to a time server, all while the
//! band is open and the air-sync path is standing by with nothing to
//! do. So it becomes a setting.
//!
//! [`GridSource::AirDt`] **skips NTP and nothing else.** It does not
//! suppress the clock, and never did in code — this paragraph used to
//! say it called [`crate::time_sync::suppress_clock`], which only the
//! `MFSK_SIM_NO_CLOCK` harness does. Believing the doc over the code
//! cost a session: it justified skipping the sink's RTC coarse anchor
//! in this mode, and an `AIR DT` start with a good RTC then spent
//! 2 min 9 s and two 25 s captures rediscovering a phase the clock
//! already had (measured on a radio, 2026-09-19).
//!
//! The concern behind it was real but is answered elsewhere: a clock
//! disciplined mid-session would take the phase away from a working
//! air-placed grid. `clock_is_disciplined()` is raised only by
//! [`crate::time_sync::note_clock_from_ntp`], and this mode does not
//! start NTP — so the RTC is visible (the log needs the time, and the
//! grid gets a free one-shot placement from it) while the phase
//! authority stays with the air.

use esp_idf_svc::nvs::{EspNvs, NvsDefault};

/// Same namespace as `boot_mode`, so one NVS handle serves both.
const NVS_KEY: &str = "grid_src";

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum GridSource {
    /// Start NTP at boot and let UTC own the slot phase once it syncs.
    /// The default, and what every build did before this existed.
    #[default]
    Ntp,
    /// Ignore the clock entirely; the phase comes from the air —
    /// cold acquisition, then lock-and-hold.
    AirDt,
}

/// What to call the grid in a log line: the source the operator chose,
/// and — when the phase is not yet coming from it — where it is coming
/// from instead.
///
/// `air:rtc` is the state that had no name: `TIME: AIR DT` selected,
/// the sink's one-shot RTC anchor holding the phase, the air not yet
/// having placed it. Reported as plain `rtc`, it was indistinguishable
/// from an NTP build that had not synced.
pub fn grid_label(src: GridSource, lock: crate::time_sync::GridLock) -> &'static str {
    use crate::time_sync::GridLock;
    match (src, lock) {
        (_, GridLock::FreeRun) => "free-run",
        (GridSource::Ntp, GridLock::Ntp) => "ntp",
        (GridSource::Ntp, GridLock::Air) => "ntp:air",
        (GridSource::Ntp, GridLock::Rtc) => "ntp:rtc",
        (GridSource::AirDt, GridLock::Air) => "air",
        (GridSource::AirDt, GridLock::Ntp) => "air:ntp",
        (GridSource::AirDt, GridLock::Rtc) => "air:rtc",
    }
}

impl GridSource {
    pub fn as_str(self) -> &'static str {
        match self {
            GridSource::Ntp => "ntp",
            GridSource::AirDt => "air",
        }
    }

    /// What the picker shows.
    pub fn label(self) -> &'static str {
        match self {
            GridSource::Ntp => "TIME: NTP",
            GridSource::AirDt => "TIME: AIR DT",
        }
    }

    pub fn from_str(s: &str) -> Self {
        match s {
            "air" => GridSource::AirDt,
            _ => GridSource::Ntp,
        }
    }
}

/// The stored choice, or [`GridSource::Ntp`] when nothing is stored.
pub fn read(nvs: &EspNvs<NvsDefault>) -> GridSource {
    let mut buf = [0u8; 8];
    match nvs.get_str(NVS_KEY, &mut buf) {
        Ok(Some(s)) => GridSource::from_str(s),
        _ => GridSource::default(),
    }
}

pub fn write(nvs: &EspNvs<NvsDefault>, src: GridSource) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_str(NVS_KEY, src.as_str())
}
