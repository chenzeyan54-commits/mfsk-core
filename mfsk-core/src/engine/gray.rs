//! Binary-reflected Gray code of a given bit width — a port of WSJT-X
//! `lib/igray.c` (`igray(n, idir)`), both directions.
//!
//! JT65 maps its 6-bit RS symbols through this and JT9 its 3-bit data
//! symbols. Until #391 each carried its own copy (`jt65::{gray6,
//! inv_gray6}` public, `jt9::tx::gray3` / `jt9::rx::inv_gray3`
//! private); FT8/FT4/FST4 use a per-protocol `GRAY_MAP` table instead
//! and do not come through here.

/// Forward Gray code (`igray.c`, `idir > 0`): `n ^ (n >> 1)`, masked to
/// the low `bits` bits of the *result* — so an out-of-range `n` behaves
/// as the per-mode copies it replaced did.
#[inline]
pub const fn gray(n: u8, bits: u32) -> u8 {
    (n ^ (n >> 1)) & mask(bits)
}

/// Inverse Gray code (`igray.c`, `idir < 0`): XOR in successively
/// larger right shifts of the running value until nothing is left to
/// shift in. `g` is masked to `bits` first.
///
/// Computed in `u32`, as `igray.c` computes in `int`: the shift reaches
/// 8 for any value of 4 bits or more, and `u8 >> 8` is an overflow —
/// a panic in debug, and in release a shift by 0 that XORs the value
/// with itself.
#[inline]
pub const fn inv_gray(g: u8, bits: u32) -> u8 {
    let mut n = (g & mask(bits)) as u32;
    let mut sh = 1;
    let mut nn = n >> sh;
    while nn > 0 {
        n ^= nn;
        sh <<= 1;
        nn = n >> sh;
    }
    n as u8
}

#[inline]
const fn mask(bits: u32) -> u8 {
    debug_assert!(bits >= 1 && bits <= 8);
    ((1u16 << bits) - 1) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bijective_and_self_inverse_at_every_width() {
        for bits in 1..=8u32 {
            let n_vals = 1u16 << bits;
            let mut seen = [false; 256];
            for n in 0..n_vals {
                let g = gray(n as u8, bits);
                assert!(!seen[g as usize], "bits={bits}: duplicate Gray for n={n}");
                seen[g as usize] = true;
                assert_eq!(inv_gray(g, bits), n as u8, "bits={bits}, n={n}");
            }
        }
    }

    /// Adjacent values differ in exactly one bit — the property the
    /// symbol mapping exists for.
    #[test]
    fn adjacent_codes_differ_in_one_bit() {
        for n in 0u8..63 {
            assert_eq!((gray(n, 6) ^ gray(n + 1, 6)).count_ones(), 1, "n={n}");
        }
    }

    /// The values the replaced copies produced, including their masking
    /// of an out-of-range input (output-masked forward, input-masked
    /// inverse).
    #[test]
    fn matches_the_replaced_copies_on_every_u8() {
        for n in 0..=255u8 {
            assert_eq!(gray(n, 6), (n ^ (n >> 1)) & 0x3f);
            assert_eq!(gray(n, 3), (n ^ (n >> 1)) & 0x7);
            let mut m = n & 0x3f;
            m ^= m >> 1;
            m ^= m >> 2;
            m ^= m >> 4;
            assert_eq!(inv_gray(n, 6), m & 0x3f);
            let mut m = n & 0x7;
            m ^= m >> 1;
            m ^= m >> 2;
            assert_eq!(inv_gray(n, 3), m & 0x7);
        }
    }
}
