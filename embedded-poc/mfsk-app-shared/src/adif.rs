//! ADIF records for the contacts the activator logs — pure string
//! building, so `hosttest/mfsk-app-shared` checks every field against
//! WSJT-X's own writer. The file I/O is the board's (`m5stack-cores3-app`
//! `storage.rs`), since the flash has to be written from a task whose
//! stack is in internal DRAM.
//!
//! Field order and spelling follow `LogBook::QSOToADIF`
//! (`logbook/logbook.cpp` 77-150) and the file header
//! `WorkedBefore.cpp` writes to a new `wsjtx_log.adi` (455-470), so a
//! log pulled off the board merges into a WSJT-X log, and uploads to
//! whatever takes one, without translation. Two additions WSJT-X has no
//! field for: `MY_SOTA_REF` and `MY_POTA_REF` (ADIF 3.1.4), which is
//! what the SOTA / POTA upload tools read an activation's reference
//! from.

use std::fmt::Write as _;

use crate::civil_time::civil_from_unix;

/// ADIF band names and edges, Hz — `models/Bands.cpp`'s `ADIF_bands`,
/// HF through 70 cm (the rest is past anything this board hears).
const BANDS: &[(&str, u64, u64)] = &[
    ("2190m", 135_700, 137_800),
    ("630m", 472_000, 479_000),
    ("560m", 501_000, 504_000),
    ("160m", 1_800_000, 2_000_000),
    ("80m", 3_500_000, 4_000_000),
    ("60m", 5_060_000, 5_450_000),
    ("40m", 7_000_000, 7_300_000),
    ("30m", 10_100_000, 10_150_000),
    ("20m", 14_000_000, 14_350_000),
    ("17m", 18_068_000, 18_168_000),
    ("15m", 21_000_000, 21_450_000),
    ("12m", 24_890_000, 24_990_000),
    ("10m", 28_000_000, 29_700_000),
    ("8m", 40_000_000, 45_000_000),
    ("6m", 50_000_000, 54_000_000),
    ("5m", 54_000_001, 69_900_000),
    ("4m", 70_000_000, 71_000_000),
    ("2m", 144_000_000, 148_000_000),
    ("1.25m", 222_000_000, 225_000_000),
    ("70cm", 420_000_000, 450_000_000),
];

/// The ADIF band a dial frequency lies in. `None` out of band — WSJT-X
/// writes `OOB` there, which no log importer accepts, so this leaves
/// the field out instead.
pub fn band_for_hz(hz: u64) -> Option<&'static str> {
    BANDS
        .iter()
        .find(|(_, lo, hi)| (*lo..=*hi).contains(&hz))
        .map(|(name, _, _)| *name)
}

/// One logged contact.
#[derive(Debug, Clone)]
pub struct Qso<'a> {
    pub call: &'a str,
    /// Empty when the caller never sent one.
    pub grid: &'a str,
    /// `FT8`, `FT4`, … — as WSJT-X names the mode.
    pub mode: &'a str,
    pub rst_sent: i8,
    /// `None` when their report never reached us (see
    /// `activator::QsoRecord::rst_rcvd`); the field is left out.
    pub rst_rcvd: Option<i8>,
    /// UTC, Unix seconds: when we first answered, and when RR73 went.
    pub on_unix: i64,
    pub off_unix: i64,
    /// Dial frequency. `None` when the board does not know it — the
    /// CoreS3 has no CAT link — and `FREQ` / `BAND` are left out
    /// rather than written wrong.
    pub dial_hz: Option<u64>,
    pub my_call: &'a str,
    pub my_grid: &'a str,
    pub my_sota_ref: Option<&'a str>,
    pub my_pota_ref: Option<&'a str>,
}

/// A new file's header, as WSJT-X writes one.
pub fn header(created_unix: i64) -> String {
    let (y, mo, d, h, mi, s) = civil_from_unix(created_unix);
    let ts = format!("{y:04}{mo:02}{d:02} {h:02}{mi:02}{s:02}");
    let ver = env!("CARGO_PKG_VERSION");
    format!(
        "ADIF Export\n<adif_ver:5>3.1.4\n<created_timestamp:15>{ts}\n\
         <programid:8>mfsk-app\n<programversion:{}>{ver}\n<eoh>\n",
        ver.len()
    )
}

/// `-07`, `+03`: WSJT-X's report text.
pub fn report(v: i8) -> String {
    format!("{}{:02}", if v >= 0 { '+' } else { '-' }, v.unsigned_abs())
}

fn field(out: &mut String, name: &str, value: &str) {
    if !out.is_empty() {
        out.push(' ');
    }
    let _ = write!(out, "<{name}:{}>{value}", value.len());
}

fn date_time(unix: i64) -> (String, String) {
    let (y, mo, d, h, mi, s) = civil_from_unix(unix);
    (
        format!("{y:04}{mo:02}{d:02}"),
        format!("{h:02}{mi:02}{s:02}"),
    )
}

/// One record, `<eor>` and newline included — what is appended to the
/// file.
pub fn record(q: &Qso) -> String {
    let mut t = String::new();
    field(&mut t, "call", q.call);
    field(&mut t, "gridsquare", q.grid);
    // WSJT-X: FT4 / FST4 / Q65 are submodes of MFSK in ADIF.
    if matches!(q.mode, "FT4" | "FST4" | "Q65") {
        field(&mut t, "mode", "MFSK");
        field(&mut t, "submode", q.mode);
    } else {
        field(&mut t, "mode", q.mode);
    }
    field(&mut t, "rst_sent", &report(q.rst_sent));
    if let Some(r) = q.rst_rcvd {
        field(&mut t, "rst_rcvd", &report(r));
    }
    let (d_on, t_on) = date_time(q.on_unix);
    let (d_off, t_off) = date_time(q.off_unix);
    field(&mut t, "qso_date", &d_on);
    field(&mut t, "time_on", &t_on);
    field(&mut t, "qso_date_off", &d_off);
    field(&mut t, "time_off", &t_off);
    if let Some(hz) = q.dial_hz {
        if let Some(b) = band_for_hz(hz) {
            field(&mut t, "band", b);
        }
        // `logqso.cpp:194`: MHz, six decimals.
        field(&mut t, "freq", &format!("{:.6}", hz as f64 / 1e6));
    }
    field(&mut t, "station_callsign", q.my_call);
    if !q.my_grid.is_empty() {
        field(&mut t, "my_gridsquare", q.my_grid);
    }
    if let Some(r) = q.my_sota_ref.filter(|r| !r.is_empty()) {
        field(&mut t, "my_sota_ref", r);
    }
    if let Some(r) = q.my_pota_ref.filter(|r| !r.is_empty()) {
        field(&mut t, "my_pota_ref", r);
    }
    t.push_str(" <eor>\n");
    t
}
