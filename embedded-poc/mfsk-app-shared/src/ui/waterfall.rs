//! Waterfall renderer — region y ∈ [14, 114), 135 × 100 px.
//!
//! Newest slot at the bottom (y=113), oldest at the top (y=14). Each
//! row is a pre-baked `WfLine` of 135 palette indices (0..15), so the
//! render pass is a flat colour-lookup + raster — no per-call FFT or
//! log work. Decode-pipeline pre-computes the row once per slot
//! (~once / 15 s) in `decode_pipeline::build_wf_row`.
//!
//! Repaint cost: 135 × 100 = 13 500 px × 16 bit = 27 KB pushed in
//! **one** `fill_contiguous` call so the SPI driver issues a single
//! `CASET`/`RASET`/`RAMWR` command + DC pulse, not 100. At 40 MHz
//! that's ~5.4 ms transfer + ~1 ms DMA / overhead = ~7-10 ms full
//! repaint, fired once per slot (~15 s). Effectively free.
//!
//! HW vertical scroll via ST7789 `VSCRDEF` / `VSCRSAD` is *also*
//! possible (TFA=14 status bar, VSA=100 waterfall, BFA=126 below)
//! and would cut the SPI cost to one row (~270 bytes = 70 µs) plus
//! the scroll-pointer write. Deferred — at one repaint per 15 s the
//! coalesced full-region path is already inside the noise floor of
//! the post-SlotEnd budget. Re-evaluate when streaming live UAC at
//! finer cadence (Phase 1 follow-up).
//!
//! Palette is a coarse NanoVNA-style 16-step gradient (black → blue
//! → cyan → green → yellow → orange → red → white).

use embedded_graphics::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};

use crate::ui::state::{WfLine, WF_DEPTH};

pub const ORIGIN_Y: i32 = 14;
pub const HEIGHT: u32 = 100;
/// Full row width. Narrow panels pass a smaller `width` to
/// [`render`] and get the leading columns.
pub const WIDTH: u32 = crate::ui::state::WF_COLS as u32;
/// The span the CoreS3's rows cover (`embedded-shared::waterfall`, which
/// builds them). The StickS3 and Core2 still take theirs from FT8's
/// stage 1, which stops at 2 700 Hz.
pub const WF_FREQ_LO_HZ: f32 = 200.0;
/// See [`WF_FREQ_LO_HZ`].
pub const WF_FREQ_HI_HZ: f32 = 3000.0;

/// 16-step palette. Indices 0..15 map magnitude bands; 0 = silence
/// (black), 15 = peak (white). RGB565 encoded inline.
const PALETTE: [Rgb565; 16] = [
    Rgb565::new(0, 0, 0),    //  0  black
    Rgb565::new(0, 0, 6),    //  1  near-black blue
    Rgb565::new(0, 0, 12),   //  2  dim blue
    Rgb565::new(0, 4, 18),   //  3  blue
    Rgb565::new(0, 12, 24),  //  4  cyan-blue
    Rgb565::new(0, 24, 24),  //  5  cyan
    Rgb565::new(0, 36, 16),  //  6  teal-green
    Rgb565::new(0, 48, 0),   //  7  green
    Rgb565::new(8, 56, 0),   //  8  yellow-green
    Rgb565::new(16, 60, 0),  //  9  lime
    Rgb565::new(24, 60, 0),  // 10  yellow-lime
    Rgb565::new(31, 56, 0),  // 11  yellow
    Rgb565::new(31, 40, 0),  // 12  orange-yellow
    Rgb565::new(31, 24, 0),  // 13  orange
    Rgb565::new(31, 8, 0),   // 14  red
    Rgb565::new(31, 31, 31), // 15  white (peak)
];

/// Repaint the waterfall region from `lines` (oldest first, newest
/// last). Lines beyond `WF_DEPTH` are ignored. Top rows are filled
/// with palette[0] (black) when fewer than `WF_DEPTH` slots have
/// arrived. Caller gates by `UiState::dirty_seq` — single
/// `fill_contiguous` over the whole `width` × 100 region so the SPI
/// driver issues one CASET/RASET/RAMWR.
pub fn render<D>(display: &mut D, lines: &[&WfLine], width: u32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    render_marked(display, lines, &[], width)
}

/// Colour of the slot-boundary rule.
const SLOT_MARK: Rgb565 = Rgb565::new(31, 0, 0);
/// Dash period of that rule, in pixels. Dashed rather than solid so
/// the row it lands on is still readable — the point is to see whether
/// the signal starts *at* the line, which a solid bar would cover.
const SLOT_MARK_DASH: usize = 6;

/// One waterfall row as the panel's wire format — RGB565, big-endian,
/// two bytes a pixel — into `out`, which must hold `width * 2` bytes.
/// `None` is a blank row. Same palette and the same dashed slot rule
/// as [`render_marked`], for a panel that sends the region as byte
/// blocks rather than through `fill_contiguous`'s per-pixel iterator.
pub fn row_rgb565_be(line: Option<&WfLine>, marked: bool, width: usize, out: &mut [u8]) {
    let width = width.min(WIDTH as usize);
    for col in 0..width {
        let c = match line {
            Some(_) if marked && col % SLOT_MARK_DASH < SLOT_MARK_DASH / 2 => SLOT_MARK,
            Some(l) => PALETTE[(l[col] & 0x0F) as usize],
            None => PALETTE[0],
        };
        let v = embedded_graphics::pixelcolor::raw::RawU16::from(c).into_inner();
        out[2 * col] = (v >> 8) as u8;
        out[2 * col + 1] = v as u8;
    }
}

/// [`render`] plus a rule across every row the caller marks.
///
/// `marks` is parallel to `lines`; a `true` draws the slot-boundary
/// rule over that row. **This is the grid the decoder decided on**,
/// not a nominal 15 s tick: `stage1_inc` stamps each `WfTick` with its
/// `pair_idx` (its `j_b`, so the slot's first tick reads 1) and that
/// row is wherever the audio sink last put the slot start. Seeing the band's transmissions begin somewhere else is the
/// difference between "the grid is off" and "nobody is transmitting",
/// which is otherwise only visible in a log this board cannot print
/// while the USB host driver owns the console.
pub fn render_marked<D>(
    display: &mut D,
    lines: &[&WfLine],
    marks: &[bool],
    width: u32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let n = lines.len().min(WF_DEPTH);
    let blank_rows = WF_DEPTH - n;
    let take = &lines[lines.len() - n..];
    // Same trailing window as `take`, and empty when the caller passed
    // none — a length mismatch marks nothing rather than panicking.
    let take_marks: &[bool] = if marks.len() == lines.len() {
        &marks[lines.len() - n..]
    } else {
        &[]
    };

    // Stream pixels top-to-bottom, left-to-right. The first
    // `blank_rows × width` pixels are palette[0]; the rest are
    // unpacked from the supplied lines.
    // `width` rather than `WIDTH`: the row carries the production
    // panel's 240 columns and a 135 px board draws the leading ones.
    let width = width.min(WIDTH);
    let rect = Rectangle::new(Point::new(0, ORIGIN_Y), Size::new(width, HEIGHT));
    let pixels = (0..HEIGHT as usize).flat_map(|row| {
        let row_pixels: &[u8] = if row < blank_rows {
            &[][..]
        } else {
            &take[row - blank_rows][..]
        };
        // For a blank row return a zero stream; otherwise
        // map each palette index to its RGB565 colour.
        let blank = row < blank_rows;
        let marked = !blank
            && take_marks
                .get(row - blank_rows)
                .copied()
                .unwrap_or(false);
        (0..width as usize).map(move |col| {
            if marked && col % SLOT_MARK_DASH < SLOT_MARK_DASH / 2 {
                return SLOT_MARK;
            }
            let idx = if blank { 0 } else { row_pixels[col] & 0x0F };
            PALETTE[idx as usize]
        })
    });
    display.fill_contiguous(&rect, pixels)
}
