// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 callsign hash table for resolving `<...>` placeholders.
//!
//! Ported from WSJT-X `lib/77bit/packjt77.f90` (`ihashcall`,
//! `save_hash_call`, `hash10`, `hash12`, `hash22`).
//!
//! Three hash widths are used in FT8 messages:
//! - **22-bit** — packed inside a 28-bit callsign token (Type 1 messages)
//! - **12-bit** — Type 4 messages (one non-standard call)
//! - **10-bit** — DXpedition RR73 messages (Type 0, n3=1)
//!
//! The table is populated as callsigns are decoded and used to resolve
//! hashed callsigns in subsequent messages.

use alloc::vec::Vec;

/// Base-38 alphabet used for callsign hashing (matches WSJT-X).
const C38: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

/// Magic constant for multiplicative hash (from WSJT-X).
const HASH_MAGIC: u64 = 47_055_833_459;

/// Maximum entries in the 22-bit LRU table.
const MAX_HASH22: usize = 1000;

/// Compute the FT8 callsign hash at a given bit width.
///
/// The callsign is left-padded to 11 characters, converted to a base-38
/// number, multiplied by a magic constant, then the top `m` bits are
/// extracted.
///
/// # Arguments
/// * `call` — callsign (up to 11 chars, will be uppercased and padded)
/// * `m` — bit width: 10, 12, or 22
pub fn ihashcall(call: &str, m: u32) -> u32 {
    let call = call.to_ascii_uppercase();
    let bytes = call.as_bytes();

    let mut n64: u64 = 0;
    for i in 0..11 {
        let c = if i < bytes.len() { bytes[i] } else { b' ' };
        let j = C38.iter().position(|&x| x == c).unwrap_or(0);
        n64 = n64.wrapping_mul(38).wrapping_add(j as u64);
    }

    let hash64 = n64.wrapping_mul(HASH_MAGIC);
    (hash64 >> (64 - m)) as u32
}

/// One stored callsign, inline — WSJT-X's `character*13`.
///
/// `packjt77.f90:5-7` declares its three tables as
/// `character(len=13), dimension(...)`: the callsign text lives *in*
/// the array, not behind a pointer. This is the same thing, and the
/// reason it matters is allocation shape rather than size. The port
/// this replaces stored `String`s in two `BTreeMap`s and a `Vec`, so
/// every callsign learned cost three heap allocations plus map nodes
/// — about 130 B, measured on air 2026-09-20 — and every one of them
/// was small. On a target whose allocator sends sub-4 KB requests to
/// internal DRAM (the CoreS3's
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`), "small" means the
/// scarce pool: internal DRAM fell from 10.7 kB to 3.4 kB over 22
/// minutes of live reception until `esp-aes` could not allocate and
/// WiFi — the only console a USB-host-mode board has — went silent.
///
/// Inline entries make the whole table three large allocations
/// instead of thousands of tiny ones, which on that board puts it in
/// PSRAM where there are megabytes spare.
///
/// Zero is the empty marker: every character `ihashcall` accepts is
/// printable, so a leading NUL cannot be a callsign.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Call13([u8; 13]);

impl Call13 {
    const EMPTY: Self = Self([0; 13]);

    fn from_str(s: &str) -> Self {
        let mut out = [0u8; 13];
        let b = s.as_bytes();
        let n = b.len().min(13);
        out[..n].copy_from_slice(&b[..n]);
        Self(out)
    }

    fn is_empty(&self) -> bool {
        self.0[0] == 0
    }

    fn as_str(&self) -> &str {
        let n = self.0.iter().position(|&c| c == 0).unwrap_or(13);
        // Written only from `&str`, so the bytes are valid UTF-8.
        core::str::from_utf8(&self.0[..n]).unwrap_or("")
    }
}

impl core::fmt::Debug for Call13 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

/// Slots in the 10-bit table — the whole key space, as upstream
/// (`packjt77.f90:5`, `dimension(0:1023)`). 13 KB once allocated.
const N_HASH10: usize = 1024;

/// Slots in the 12-bit table (`packjt77.f90:6`, `dimension(0:4095)`).
/// 53 KB once allocated, and the reason allocation is lazy.
const N_HASH12: usize = 4096;

/// Runtime callsign hash lookup table.
///
/// Populated during decoding; used to resolve `<...>` placeholders in
/// messages containing hashed callsigns.
///
/// **`new()` allocates nothing, and that is load-bearing.**
/// `wsjt77::is_plausible_payload` builds a fresh empty table on every
/// call, so an eager 83 KB here would land on every `unpack77`. The
/// three blocks are allocated on the first [`Self::insert`] and not
/// before.
#[derive(Debug, Clone)]
pub struct CallsignHashTable {
    /// 10-bit hash → callsign, direct-indexed over the whole key
    /// space. `None` until the first insert.
    calls10: Option<Vec<Call13>>,
    /// 12-bit hash → callsign, direct-indexed. `None` until the first
    /// insert.
    calls12: Option<Vec<Call13>>,
    /// The 22-bit LRU: `hash22[i]` names `calls22[i]`, most recent
    /// first, exactly as `packjt77.f90:7,12`'s parallel arrays do.
    /// `None` until the first insert.
    calls22: Option<Vec<Call13>>,
    hash22: Option<Vec<u32>>,
    /// Live entries in the LRU, `<= MAX_HASH22` — upstream's `nzhash`.
    n22: usize,
}

impl CallsignHashTable {
    /// Create an empty hash table. Allocates nothing.
    pub fn new() -> Self {
        Self {
            calls10: None,
            calls12: None,
            calls22: None,
            hash22: None,
            n22: 0,
        }
    }

    /// Allocate the three blocks, if they are not there yet.
    ///
    /// `vec![Call13::EMPTY; N]` and not `Box::new([Call13::EMPTY; N])`:
    /// the latter builds the array as a temporary first, and 53 KB of
    /// temporary is an embedded task's whole stack. Two of this
    /// project's "heap corruption" investigations turned out to be
    /// stack overflows (`embedded-poc/CLAUDE.md`, "Stacks, heaps, and
    /// the space between them").
    fn ensure(&mut self) {
        if self.calls10.is_none() {
            self.calls10 = Some(alloc::vec![Call13::EMPTY; N_HASH10]);
            self.calls12 = Some(alloc::vec![Call13::EMPTY; N_HASH12]);
            self.calls22 = Some(alloc::vec![Call13::EMPTY; MAX_HASH22]);
            self.hash22 = Some(alloc::vec![0u32; MAX_HASH22]);
        }
    }

    /// Register a decoded callsign, populating all three tables.
    ///
    /// Skips empty strings, `<...>` placeholders, and strings shorter
    /// than 2 characters. Strips `<>` brackets if present.
    ///
    /// (Upstream's `save_hash_call` requires 3, not 2
    /// — `packjt77.f90:91`, `if(len(trim(cw)) .lt. 3) return`. The
    /// port has always used 2; left as it was, because changing an
    /// acceptance rule is not what this rewrite is for.)
    pub fn insert(&mut self, call: &str) {
        let call = call.trim();
        // Strip angle brackets
        let call = call.strip_prefix('<').unwrap_or(call);
        let call = call.strip_suffix('>').unwrap_or(call);
        // Strip /R or /P suffix for hashing
        let base = if call.ends_with("/R") || call.ends_with("/P") {
            &call[..call.len() - 2]
        } else {
            call
        };

        if base.len() < 2 || base == "..." || base.starts_with("CQ") {
            return;
        }

        let n10 = ihashcall(base, 10);
        let n12 = ihashcall(base, 12);
        let n22 = ihashcall(base, 22);
        let entry = Call13::from_str(base);

        self.ensure();
        // `ihashcall` returns the top `m` bits, so these are in range
        // by construction; the guard mirrors `packjt77.f90:94,97`
        // rather than trusting that across a future change.
        if let Some(t) = self.calls10.as_mut()
            && let Some(slot) = t.get_mut(n10 as usize)
        {
            *slot = entry;
        }
        if let Some(t) = self.calls12.as_mut()
            && let Some(slot) = t.get_mut(n12 as usize)
        {
            *slot = entry;
        }

        // 22-bit LRU, `packjt77.f90:99-112`: refresh in place if the
        // hash is already known, else push everything down and take
        // the front.
        let (Some(hashes), Some(calls)) = (self.hash22.as_mut(), self.calls22.as_mut()) else {
            return;
        };
        if let Some(pos) = hashes[..self.n22].iter().position(|&h| h == n22) {
            calls[pos] = entry;
            hashes[..=pos].rotate_right(1);
            calls[..=pos].rotate_right(1);
            return;
        }
        if self.n22 < MAX_HASH22 {
            self.n22 += 1;
        }
        hashes[..self.n22].rotate_right(1);
        calls[..self.n22].rotate_right(1);
        hashes[0] = n22;
        calls[0] = entry;
    }

    /// Look up a 10-bit hash. Returns the callsign if found.
    pub fn lookup10(&self, n10: u32) -> Option<&str> {
        let e = self.calls10.as_ref()?.get(n10 as usize)?;
        (!e.is_empty()).then(|| e.as_str())
    }

    /// Look up a 12-bit hash. Returns the callsign if found.
    pub fn lookup12(&self, n12: u32) -> Option<&str> {
        let e = self.calls12.as_ref()?.get(n12 as usize)?;
        (!e.is_empty()).then(|| e.as_str())
    }

    /// Look up a 22-bit hash. Returns the callsign, **unwrapped**.
    ///
    /// It used to return `<CALL>` while [`Self::lookup10`] and
    /// [`Self::lookup12`] returned the bare callsign — an asymmetry
    /// nothing announced, and one that cost a double-wrapped
    /// `<<PA3XYZ>>` during the issue #383 type-5 port before a test
    /// caught it. All three return the same shape now; the callers
    /// that want `<>` add it.
    pub fn lookup22(&self, n22: u32) -> Option<&str> {
        let hashes = self.hash22.as_ref()?;
        let pos = hashes[..self.n22].iter().position(|&h| h == n22)?;
        Some(self.calls22.as_ref()?[pos].as_str())
    }

    /// Clear all entries, keeping the blocks for reuse.
    pub fn clear(&mut self) {
        for t in [
            self.calls10.as_mut(),
            self.calls12.as_mut(),
            self.calls22.as_mut(),
        ]
        .into_iter()
        .flatten()
        {
            t.fill(Call13::EMPTY);
        }
        self.n22 = 0;
    }

    /// Number of entries in the 22-bit table (for diagnostics).
    pub fn len22(&self) -> usize {
        self.n22
    }
}

impl Default for CallsignHashTable {
    fn default() -> Self {
        Self::new()
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hash_basic() {
        // Verify hash values are deterministic and non-zero
        let h22 = ihashcall("JA1ABC", 22);
        let h12 = ihashcall("JA1ABC", 12);
        let h10 = ihashcall("JA1ABC", 10);
        assert!(h22 < (1 << 22));
        assert!(h12 < (1 << 12));
        assert!(h10 < (1 << 10));
        // Same input → same output
        assert_eq!(h22, ihashcall("JA1ABC", 22));
    }

    /// `new()` must allocate nothing — `wsjt77::is_plausible_payload`
    /// builds one per `unpack77` call, and the three blocks are 83 KB.
    #[test]
    fn an_empty_table_holds_no_blocks_and_answers_nothing() {
        let t = CallsignHashTable::new();
        assert!(t.calls10.is_none() && t.calls12.is_none() && t.calls22.is_none());
        assert_eq!(t.lookup10(0), None);
        assert_eq!(t.lookup12(0), None);
        assert_eq!(t.lookup22(0), None);
        assert_eq!(t.len22(), 0);
    }

    /// `packjt77.f90:99-112` — a repeat moves its entry to the front
    /// and does not grow the table; a new one pushes everything down.
    #[test]
    fn the_lru_orders_by_recency_like_upstream() {
        let mut t = CallsignHashTable::new();
        for c in ["JA1ABC", "3Y0Z", "W1AW"] {
            t.insert(c);
        }
        assert_eq!(t.len22(), 3);
        // Most recent first.
        assert_eq!(t.calls22.as_ref().unwrap()[0].as_str(), "W1AW");
        assert_eq!(t.calls22.as_ref().unwrap()[2].as_str(), "JA1ABC");

        // Re-inserting the oldest refreshes it in place, no growth.
        t.insert("JA1ABC");
        assert_eq!(t.len22(), 3);
        assert_eq!(t.calls22.as_ref().unwrap()[0].as_str(), "JA1ABC");
        assert_eq!(t.calls22.as_ref().unwrap()[2].as_str(), "3Y0Z");
        // And every one of them still resolves.
        for c in ["JA1ABC", "3Y0Z", "W1AW"] {
            assert_eq!(t.lookup22(ihashcall(c, 22)), Some(c), "{c}");
        }
    }

    /// The LRU is capped; the 10-/12-bit tables are bounded by their
    /// key spaces instead, a collision overwriting.
    #[test]
    fn the_tables_stay_within_their_bounds() {
        let mut t = CallsignHashTable::new();
        for i in 0..(MAX_HASH22 + 50) {
            t.insert(&alloc::format!("A{i}BC"));
        }
        assert_eq!(t.len22(), MAX_HASH22);
        assert_eq!(t.calls22.as_ref().unwrap().len(), MAX_HASH22);
        assert_eq!(t.calls10.as_ref().unwrap().len(), N_HASH10);
        assert_eq!(t.calls12.as_ref().unwrap().len(), N_HASH12);
    }

    /// A 13-character callsign is the longest `character*13` holds, and
    /// it has to come back whole.
    #[test]
    fn a_full_width_callsign_round_trips() {
        let long = "ABCDEFGHIJKLM";
        assert_eq!(long.len(), 13);
        let mut t = CallsignHashTable::new();
        t.insert(long);
        assert_eq!(t.lookup22(ihashcall(long, 22)), Some(long));
    }

    /// `clear` keeps the blocks — it is called between sessions, not
    /// between messages, and re-allocating 83 KB to answer `None` is
    /// not what it is for.
    #[test]
    fn clear_empties_without_releasing_the_blocks() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        t.clear();
        assert_eq!(t.len22(), 0);
        assert_eq!(t.lookup22(ihashcall("JA1ABC", 22)), None);
        assert_eq!(t.lookup10(ihashcall("JA1ABC", 10)), None);
        assert!(t.calls10.is_some(), "blocks are kept for reuse");
    }

    #[test]
    fn insert_and_lookup() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        t.insert("3Y0Z");

        let h22 = ihashcall("JA1ABC", 22);
        let h12 = ihashcall("JA1ABC", 12);
        let h10 = ihashcall("JA1ABC", 10);

        assert_eq!(t.lookup22(h22), Some("JA1ABC"));
        assert_eq!(t.lookup12(h12), Some("JA1ABC"));
        assert_eq!(t.lookup10(h10), Some("JA1ABC"));

        let h22z = ihashcall("3Y0Z", 22);
        assert_eq!(t.lookup22(h22z), Some("3Y0Z"));
    }

    #[test]
    fn lru_eviction() {
        let mut t = CallsignHashTable::new();
        // Fill beyond MAX_HASH22
        for i in 0..MAX_HASH22 + 10 {
            t.insert(&format!("T{:04}X", i));
        }
        assert_eq!(t.len22(), MAX_HASH22);
    }

    #[test]
    fn skip_special() {
        let mut t = CallsignHashTable::new();
        t.insert("<...>");
        t.insert("CQ");
        t.insert("CQ DX");
        t.insert("");
        t.insert("A"); // too short
        assert_eq!(t.len22(), 0);
    }

    #[test]
    fn strip_suffix() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC/P");
        let h22 = ihashcall("JA1ABC", 22);
        assert_eq!(t.lookup22(h22), Some("JA1ABC"));
    }
}
