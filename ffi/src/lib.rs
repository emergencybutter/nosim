//! C ABI for the nosim core.
//!
//! Conventions, chosen for an Unreal Engine 5 host but plain C otherwise:
//! - Doubles for all world quantities; `#[repr(C)]` POD structs passed and returned by value.
//! - Stateful objects ([`NosimFloatingOrigin`], [`NosimVfs`], [`NosimPackage`],
//!   [`NosimStarCatalog`]) are opaque handles with `*_new` / `*_load` and `*_free`. Passing
//!   `NULL` to any function is safe: it returns [`NosimStatus::NullPointer`], `false` or zero.
//! - Fallible calls return [`NosimStatus`]; the message behind a failure is available from
//!   [`nosim_last_error`] until the next failing call on the same thread.
//! - Strings in are NUL-terminated UTF-8. Strings out go into caller buffers; the return
//!   value is the size the string needs including the NUL, so `0` means "nothing" and a
//!   value larger than the capacity means "truncated, call again with this much".
//! - Array outputs take `(out, capacity)` and return the total count available; at most
//!   `capacity` elements are written.
//!
//! The header `include/nosim.h` is regenerated from this file on every build.

use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};
use std::io::BufReader;
use std::ptr;
use std::sync::Arc;

use nosim::arinc424::{self, RunwayRecord};
use nosim::geodesy::{self, EnuFrame, FloatingOrigin, Geodetic, Vec3};
use nosim::scenery::vfs::Vfs;
use nosim::scenery::{Bounds, ContentKind, Package};
use nosim::starfield::Catalog;
use nosim::{astro, ephem, lod, phenology, photometry, procedural, starfield, traffic};

// ---- Status and errors -----------------------------------------------------------------

/// Result of a fallible call.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NosimStatus {
    /// Success.
    Ok = 0,
    /// A required pointer was NULL.
    NullPointer = 1,
    /// An argument was out of range or not valid UTF-8.
    InvalidArgument = 2,
    /// A file could not be read.
    Io = 3,
    /// Input text did not parse.
    Parse = 4,
    /// Input parsed but failed validation.
    Validation = 5,
    /// The requested item does not exist.
    NotFound = 6,
    /// The output buffer was too small; nothing was written.
    BufferTooSmall = 7,
}

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn fail(status: NosimStatus, message: impl Into<String>) -> NosimStatus {
    let text = message.into().replace('\0', " ");
    LAST_ERROR.with(|e| *e.borrow_mut() = CString::new(text).ok());
    status
}

/// Message for the most recent failure on this thread, or NULL if none. Valid until the
/// next failing call on the same thread.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_last_error() -> *const c_char {
    LAST_ERROR.with(|e| e.borrow().as_ref().map_or(ptr::null(), |s| s.as_ptr()))
}

/// Crate version, e.g. `0.1.0`. Static storage.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr().cast()
}

/// ABI version; bumped on any incompatible change to the header.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_abi_version() -> u32 {
    1
}

// ---- Helpers ---------------------------------------------------------------------------

/// # Safety
/// `s` must be NULL or a valid NUL-terminated string.
unsafe fn read_cstr<'a>(s: *const c_char) -> Result<&'a str, NosimStatus> {
    if s.is_null() {
        return Err(fail(NosimStatus::NullPointer, "string argument is NULL"));
    }
    // SAFETY: caller guarantees a NUL-terminated string.
    unsafe { CStr::from_ptr(s) }.to_str().map_err(|_| fail(NosimStatus::InvalidArgument, "string is not UTF-8"))
}

/// Copies `s` into `buf`, always NUL-terminating when `cap > 0`; returns the size needed.
///
/// # Safety
/// `buf` must be NULL or point to `cap` writable bytes.
unsafe fn write_cstr(buf: *mut c_char, cap: usize, s: &str) -> usize {
    let needed = s.len() + 1;
    if buf.is_null() || cap == 0 {
        return needed;
    }
    let n = s.len().min(cap - 1);
    // SAFETY: caller guarantees `cap` writable bytes; we write at most `cap`.
    unsafe {
        ptr::copy_nonoverlapping(s.as_ptr().cast::<c_char>(), buf, n);
        *buf.add(n) = 0;
    }
    needed
}

fn fill_array<const N: usize>(dst: &mut [c_char; N], s: &str) {
    let bytes = s.as_bytes();
    let n = bytes.len().min(N - 1);
    for (d, b) in dst.iter_mut().zip(bytes) {
        *d = *b as c_char;
    }
    dst[n] = 0;
}

fn array_to_string<const N: usize>(src: &[c_char; N]) -> String {
    let bytes: Vec<u8> = src.iter().take_while(|&&c| c != 0).map(|&c| c as u8).collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

/// # Safety
/// `p` must be NULL or valid for reads.
unsafe fn opt_ref<'a, T>(p: *const T) -> Option<&'a T> {
    // SAFETY: caller guarantees validity when non-null.
    unsafe { p.as_ref() }
}

/// # Safety
/// `p` must be NULL or valid for writes.
unsafe fn opt_mut<'a, T>(p: *mut T) -> Option<&'a mut T> {
    // SAFETY: caller guarantees validity when non-null.
    unsafe { p.as_mut() }
}

/// # Safety
/// `p` must be NULL or valid for `n` reads.
unsafe fn slice_in<'a, T>(p: *const T, n: usize) -> Option<&'a [T]> {
    if n == 0 {
        return Some(&[]);
    }
    if p.is_null() {
        return None;
    }
    // SAFETY: caller guarantees `n` readable elements.
    Some(unsafe { std::slice::from_raw_parts(p, n) })
}

/// # Safety
/// `p` must be NULL or valid for `n` writes.
unsafe fn slice_out<'a, T>(p: *mut T, n: usize) -> Option<&'a mut [T]> {
    if n == 0 {
        return Some(&mut []);
    }
    if p.is_null() {
        return None;
    }
    // SAFETY: caller guarantees `n` writable elements.
    Some(unsafe { std::slice::from_raw_parts_mut(p, n) })
}

// ---- Basic types -----------------------------------------------------------------------

/// A 3-vector, metres or kilometres as each function states.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimVec3 {
    /// X.
    pub x: f64,
    /// Y.
    pub y: f64,
    /// Z.
    pub z: f64,
}

impl From<Vec3> for NosimVec3 {
    fn from(v: Vec3) -> Self {
        NosimVec3 { x: v.x, y: v.y, z: v.z }
    }
}

impl From<NosimVec3> for Vec3 {
    fn from(v: NosimVec3) -> Self {
        Vec3::new(v.x, v.y, v.z)
    }
}

/// WGS84 geodetic position.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimGeodetic {
    /// Latitude, degrees north.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Ellipsoidal height, metres.
    pub h_m: f64,
}

impl From<Geodetic> for NosimGeodetic {
    fn from(g: Geodetic) -> Self {
        NosimGeodetic { lat_deg: g.lat_deg, lon_deg: g.lon_deg, h_m: g.h_m }
    }
}

impl From<NosimGeodetic> for Geodetic {
    fn from(g: NosimGeodetic) -> Self {
        Geodetic::new(g.lat_deg, g.lon_deg, g.h_m)
    }
}

/// Spherical ecliptic coordinates: longitude and latitude in radians, distance as stated.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimSpherical {
    /// Longitude, radians, `[0, 2π)`.
    pub lon: f64,
    /// Latitude, radians.
    pub lat: f64,
    /// Distance.
    pub r: f64,
}

impl From<ephem::Spherical> for NosimSpherical {
    fn from(s: ephem::Spherical) -> Self {
        NosimSpherical { lon: s.lon, lat: s.lat, r: s.r }
    }
}

/// Linear sRGB colour, each channel in `[0, 1]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimRgb {
    /// Red.
    pub r: f32,
    /// Green.
    pub g: f32,
    /// Blue.
    pub b: f32,
}

impl From<[f32; 3]> for NosimRgb {
    fn from(c: [f32; 3]) -> Self {
        NosimRgb { r: c[0], g: c[1], b: c[2] }
    }
}

// ---- Geodesy ---------------------------------------------------------------------------

/// Geodetic → ECEF, metres.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_geodetic_to_ecef(g: NosimGeodetic) -> NosimVec3 {
    geodesy::geodetic_to_ecef(g.into()).into()
}

/// ECEF metres → geodetic.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ecef_to_geodetic(ecef: NosimVec3) -> NosimGeodetic {
    geodesy::ecef_to_geodetic(ecef.into()).into()
}

/// Unit ENU forward vector for a true heading in degrees clockwise from north.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_heading_to_enu(true_heading_deg: f64) -> NosimVec3 {
    geodesy::heading_to_enu(true_heading_deg).into()
}

/// ECEF → ENU metres relative to `anchor`.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ecef_to_enu(anchor: NosimGeodetic, ecef: NosimVec3) -> NosimVec3 {
    EnuFrame::at(anchor.into()).to_enu(ecef.into()).into()
}

/// ENU metres relative to `anchor` → ECEF.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_enu_to_ecef(anchor: NosimGeodetic, enu: NosimVec3) -> NosimVec3 {
    EnuFrame::at(anchor.into()).to_ecef(enu.into()).into()
}

/// Batch ECEF → ENU; `input` and `output` may alias.
///
/// # Safety
/// `input` and `output` must each point to `count` elements (or be NULL when `count` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ecef_to_enu_batch(
    anchor: NosimGeodetic,
    input: *const NosimVec3,
    output: *mut NosimVec3,
    count: usize,
) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(src), Some(dst)) = (unsafe { slice_in(input, count) }, unsafe { slice_out(output, count) }) else {
        return fail(NosimStatus::NullPointer, "batch buffers are NULL");
    };
    let frame = EnuFrame::at(anchor.into());
    for i in 0..count {
        dst[i] = frame.to_enu(src[i].into()).into();
    }
    NosimStatus::Ok
}

/// Batch ENU → ECEF; `input` and `output` may alias.
///
/// # Safety
/// `input` and `output` must each point to `count` elements (or be NULL when `count` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_enu_to_ecef_batch(
    anchor: NosimGeodetic,
    input: *const NosimVec3,
    output: *mut NosimVec3,
    count: usize,
) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(src), Some(dst)) = (unsafe { slice_in(input, count) }, unsafe { slice_out(output, count) }) else {
        return fail(NosimStatus::NullPointer, "batch buffers are NULL");
    };
    let frame = EnuFrame::at(anchor.into());
    for i in 0..count {
        dst[i] = frame.to_ecef(src[i].into()).into();
    }
    NosimStatus::Ok
}

/// Floating render origin (opaque).
pub struct NosimFloatingOrigin {
    inner: FloatingOrigin,
}

/// Creates a floating origin at the camera. `threshold_m <= 0` selects the spec's 10 km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_floating_origin_new(camera_ecef: NosimVec3, threshold_m: f64) -> *mut NosimFloatingOrigin {
    let inner = if threshold_m > 0.0 {
        FloatingOrigin::with_threshold(camera_ecef.into(), threshold_m)
    } else {
        FloatingOrigin::new(camera_ecef.into())
    };
    Box::into_raw(Box::new(NosimFloatingOrigin { inner }))
}

/// Releases a floating origin. NULL is ignored.
///
/// # Safety
/// `h` must be NULL or a handle from [`nosim_floating_origin_new`] not yet freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_free(h: *mut NosimFloatingOrigin) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw and is freed once.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Moves the camera; returns true if the origin was rebased.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_update(h: *mut NosimFloatingOrigin, camera_ecef: NosimVec3) -> bool {
    // SAFETY: documented contract.
    unsafe { opt_mut(h) }.is_some_and(|o| o.inner.update(camera_ecef.into()))
}

/// ECEF metres → render space. Zero vector for a NULL handle.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_to_render(h: *const NosimFloatingOrigin, ecef: NosimVec3) -> NosimVec3 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(NosimVec3::default(), |o| o.inner.to_render(ecef.into()).into())
}

/// Render space → ECEF metres. Zero vector for a NULL handle.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_to_ecef(h: *const NosimFloatingOrigin, render: NosimVec3) -> NosimVec3 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(NosimVec3::default(), |o| o.inner.to_ecef(render.into()).into())
}

/// Batch ECEF → render space; buffers may alias.
///
/// # Safety
/// `h` must be a live handle; `input`/`output` must point to `count` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_to_render_batch(
    h: *const NosimFloatingOrigin,
    input: *const NosimVec3,
    output: *mut NosimVec3,
    count: usize,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(o) = (unsafe { opt_ref(h) }) else { return fail(NosimStatus::NullPointer, "floating origin is NULL") };
    // SAFETY: documented contract.
    let (Some(src), Some(dst)) = (unsafe { slice_in(input, count) }, unsafe { slice_out(output, count) }) else {
        return fail(NosimStatus::NullPointer, "batch buffers are NULL");
    };
    for i in 0..count {
        dst[i] = o.inner.to_render(src[i].into()).into();
    }
    NosimStatus::Ok
}

/// Current origin in ECEF metres.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_ecef(h: *const NosimFloatingOrigin) -> NosimVec3 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(NosimVec3::default(), |o| o.inner.frame().origin_ecef.into())
}

/// Number of times the origin has been placed (1 after creation).
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_floating_origin_rebase_count(h: *const NosimFloatingOrigin) -> u32 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |o| o.inner.rebase_count())
}

// ---- Calendar and sidereal time --------------------------------------------------------

/// Julian Date of a proleptic Gregorian date and UT hours.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_julian_date(year: i32, month: u32, day: u32, ut_hours: f64) -> f64 {
    astro::julian_date(year, month, day, ut_hours)
}

/// 1-based day of year; 0 for an invalid month.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_day_of_year(year: i32, month: u32, day: u32) -> u32 {
    if !(1..=12).contains(&month) {
        return 0;
    }
    astro::day_of_year(year, month, day)
}

/// Solar declination, degrees, from the day of year.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_solar_declination_deg(doy: u32) -> f64 {
    astro::solar_declination_deg(doy)
}

/// Sea-level temperature lapsed to altitude (6.5 °C/km).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_local_temperature_c(t_sea_level_c: f64, altitude_msl_m: f64) -> f64 {
    astro::local_temperature_c(t_sea_level_c, altitude_msl_m)
}

/// Greenwich Mean Sidereal Time, degrees.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_gmst_deg(jd_ut1: f64) -> f64 {
    astro::gmst_deg(jd_ut1)
}

/// Greenwich Apparent Sidereal Time, degrees.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_gast_deg(jd_ut1: f64) -> f64 {
    ephem::gast_deg(jd_ut1)
}

/// ECI (equator of date) → ECEF rotation by a sidereal angle in degrees.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_eci_to_ecef(eci: NosimVec3, sidereal_deg: f64) -> NosimVec3 {
    astro::eci_to_ecef(eci.into(), sidereal_deg).into()
}

/// ECEF → ECI rotation by a sidereal angle in degrees.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ecef_to_eci(ecef: NosimVec3, sidereal_deg: f64) -> NosimVec3 {
    astro::ecef_to_eci(ecef.into(), sidereal_deg).into()
}

// ---- Ephemerides -----------------------------------------------------------------------

/// Geometric geocentric Sun, ecliptic and equinox of date; `r` in AU.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_sun_geocentric(jd_tdb: f64) -> NosimSpherical {
    ephem::sun_geocentric(jd_tdb).into()
}

/// Geocentric Moon, mean ecliptic and equinox of date; `r` in km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_moon_geocentric(jd_tdb: f64) -> NosimSpherical {
    ephem::moon_geocentric(jd_tdb).into()
}

/// Geocentric Moon in the J2000 ecliptic frame, km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_moon_geocentric_j2000(jd_tdb: f64) -> NosimVec3 {
    ephem::moon_geocentric_j2000(jd_tdb).into()
}

/// Apparent geocentric Sun, true equator and equinox of date, km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_sun_apparent_equatorial_km(jd_tdb: f64) -> NosimVec3 {
    ephem::sun_apparent_equatorial_km(jd_tdb).into()
}

/// Apparent geocentric Moon, true equator and equinox of date, km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_moon_apparent_equatorial_km(jd_tdb: f64) -> NosimVec3 {
    ephem::moon_apparent_equatorial_km(jd_tdb).into()
}

/// Mean obliquity of the ecliptic, radians.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_mean_obliquity(jd_tdb: f64) -> f64 {
    ephem::mean_obliquity(jd_tdb)
}

/// True obliquity of the ecliptic, radians.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_true_obliquity(jd_tdb: f64) -> f64 {
    ephem::true_obliquity(jd_tdb)
}

/// Nutation in longitude and obliquity, radians. Either output may be NULL.
///
/// # Safety
/// Non-NULL outputs must be valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_nutation(jd_tdb: f64, dpsi: *mut f64, deps: *mut f64) {
    let n = ephem::nutation(jd_tdb);
    // SAFETY: documented contract.
    if let Some(p) = unsafe { opt_mut(dpsi) } {
        *p = n.dpsi;
    }
    // SAFETY: documented contract.
    if let Some(e) = unsafe { opt_mut(deps) } {
        *e = n.deps;
    }
}

/// Rotates ecliptic coordinates to equatorial by obliquity `eps` (radians).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ecliptic_to_equatorial(v: NosimVec3, eps: f64) -> NosimVec3 {
    ephem::ecliptic_to_equatorial(v.into(), eps).into()
}

/// Rotates equatorial coordinates to ecliptic by obliquity `eps` (radians).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_equatorial_to_ecliptic(v: NosimVec3, eps: f64) -> NosimVec3 {
    ephem::equatorial_to_ecliptic(v.into(), eps).into()
}

/// Observer position in the equatorial frame of date, km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_observer_equatorial_km(observer: NosimGeodetic, gast_deg: f64) -> NosimVec3 {
    ephem::observer_equatorial_km(observer.into(), gast_deg).into()
}

/// Topocentric vector (km, equatorial of date) from a geocentric one.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_topocentric_km(
    geocentric_equatorial_km: NosimVec3,
    observer: NosimGeodetic,
    gast_deg: f64,
) -> NosimVec3 {
    ephem::topocentric_km(geocentric_equatorial_km.into(), observer.into(), gast_deg).into()
}

/// Right ascension (radians, `[0, 2π)`) and declination (radians) of an equatorial vector.
///
/// # Safety
/// Non-NULL outputs must be valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ra_dec(v: NosimVec3, ra: *mut f64, dec: *mut f64) {
    let (a, d) = ephem::ra_dec(v.into());
    // SAFETY: documented contract.
    if let Some(p) = unsafe { opt_mut(ra) } {
        *p = a;
    }
    // SAFETY: documented contract.
    if let Some(p) = unsafe { opt_mut(dec) } {
        *p = d;
    }
}

/// Phase angle at the Moon, radians; both vectors geocentric, same frame and units.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_phase_angle(sun_geocentric: NosimVec3, moon_geocentric: NosimVec3) -> f64 {
    ephem::phase_angle(sun_geocentric.into(), moon_geocentric.into())
}

/// Illuminated fraction of the lunar disc from the phase angle (radians).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_illuminated_fraction(phase_angle_rad: f64) -> f64 {
    ephem::illuminated_fraction(phase_angle_rad)
}

/// Angle between two directions, radians.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_elongation(a: NosimVec3, b: NosimVec3) -> f64 {
    ephem::elongation(a.into(), b.into())
}

/// Optical libration, degrees.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimLibration {
    /// Selenographic longitude of the sub-Earth point.
    pub l_deg: f64,
    /// Selenographic latitude of the sub-Earth point.
    pub b_deg: f64,
}

/// Optical libration of the Moon.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_optical_libration(jd_tdb: f64) -> NosimLibration {
    let l = ephem::optical_libration(jd_tdb);
    NosimLibration { l_deg: l.l_deg, b_deg: l.b_deg }
}

/// Astronomical unit in km.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_au_km() -> f64 {
    ephem::AU_KM
}

// ---- Starfield -------------------------------------------------------------------------

/// Loaded star catalogue (opaque).
pub struct NosimStarCatalog {
    inner: Catalog,
}

/// 16-byte GPU record: RA and Dec in radians (J2000), V, B−V.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimStarRecord {
    /// Right ascension, radians.
    pub ra: f32,
    /// Declination, radians.
    pub dec: f32,
    /// Visual magnitude.
    pub vmag: f32,
    /// B−V (default substituted when unmeasured).
    pub bv: f32,
}

/// Full star entry.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimStar {
    /// Harvard Revised number.
    pub hr: u16,
    /// True when `bv` is a catalogue value rather than the default.
    pub has_bv: bool,
    /// Right ascension, J2000, radians.
    pub ra_rad: f64,
    /// Declination, J2000, radians.
    pub dec_rad: f64,
    /// Visual magnitude.
    pub vmag: f32,
    /// B−V colour index.
    pub bv: f32,
    /// Proper motion in RA (with cos δ), arcsec/yr.
    pub pm_ra_arcsec_yr: f32,
    /// Proper motion in Dec, arcsec/yr.
    pub pm_dec_arcsec_yr: f32,
    /// Blackbody temperature from B−V, K.
    pub temperature_k: f64,
    /// Rendering colour.
    pub color: NosimRgb,
}

/// Loads the packed catalogue written by `compile_bsc5`. NULL on failure.
///
/// # Safety
/// `bytes` must point to `len` readable bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_from_packed(bytes: *const u8, len: usize) -> *mut NosimStarCatalog {
    // SAFETY: documented contract.
    let Some(data) = (unsafe { slice_in(bytes, len) }) else {
        fail(NosimStatus::NullPointer, "packed catalogue pointer is NULL");
        return ptr::null_mut();
    };
    match Catalog::from_packed(data) {
        Ok(inner) => Box::into_raw(Box::new(NosimStarCatalog { inner })),
        Err(e) => {
            fail(NosimStatus::Parse, e.to_string());
            ptr::null_mut()
        }
    }
}

/// Parses the CDS V/50 `catalog` text file at `path`. NULL on failure.
///
/// # Safety
/// `path` must be a NUL-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_load_bsc5(path: *const c_char) -> *mut NosimStarCatalog {
    // SAFETY: documented contract.
    let Ok(path) = (unsafe { read_cstr(path) }) else { return ptr::null_mut() };
    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(e) => {
            fail(NosimStatus::Io, format!("{path}: {e}"));
            return ptr::null_mut();
        }
    };
    match Catalog::parse_bsc5(BufReader::new(file)) {
        Ok(inner) => Box::into_raw(Box::new(NosimStarCatalog { inner })),
        Err(e) => {
            fail(NosimStatus::Parse, e.to_string());
            ptr::null_mut()
        }
    }
}

/// Releases a catalogue. NULL is ignored.
///
/// # Safety
/// `h` must be NULL or a live handle, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_free(h: *mut NosimStarCatalog) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Number of stars with positions.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_count(h: *const NosimStarCatalog) -> usize {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |c| c.inner.stars().len())
}

/// Fills up to `capacity` GPU records in HR order; returns the total available.
///
/// # Safety
/// `h` must be NULL or a live handle; `out` must point to `capacity` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_records(
    h: *const NosimStarCatalog,
    out: *mut NosimStarRecord,
    capacity: usize,
) -> usize {
    // SAFETY: documented contract.
    let Some(c) = (unsafe { opt_ref(h) }) else { return 0 };
    // SAFETY: documented contract.
    let Some(dst) = (unsafe { slice_out(out, capacity) }) else { return c.inner.stars().len() };
    for (d, s) in dst.iter_mut().zip(c.inner.records()) {
        *d = NosimStarRecord { ra: s.ra, dec: s.dec, vmag: s.vmag, bv: s.bv };
    }
    c.inner.stars().len()
}

/// Star at `index` (0-based, HR order).
///
/// # Safety
/// `h` must be NULL or a live handle; `out` must be valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_star(
    h: *const NosimStarCatalog,
    index: usize,
    out: *mut NosimStar,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(c) = (unsafe { opt_ref(h) }) else { return fail(NosimStatus::NullPointer, "catalogue is NULL") };
    // SAFETY: documented contract.
    let Some(out) = (unsafe { opt_mut(out) }) else { return fail(NosimStatus::NullPointer, "output is NULL") };
    let Some(s) = c.inner.stars().get(index) else {
        return fail(NosimStatus::NotFound, format!("star index {index} out of range"));
    };
    *out = NosimStar {
        hr: s.hr,
        has_bv: s.bv.is_some(),
        ra_rad: s.ra_rad,
        dec_rad: s.dec_rad,
        vmag: s.vmag,
        bv: s.bv.unwrap_or(starfield::DEFAULT_BV),
        pm_ra_arcsec_yr: s.pm_ra_arcsec_yr,
        pm_dec_arcsec_yr: s.pm_dec_arcsec_yr,
        temperature_k: s.temperature_k(),
        color: s.linear_srgb().into(),
    };
    NosimStatus::Ok
}

/// Index of the star with Harvard Revised number `hr`, or -1.
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_star_catalog_find_hr(h: *const NosimStarCatalog, hr: u16) -> isize {
    // SAFETY: documented contract.
    let Some(c) = (unsafe { opt_ref(h) }) else { return -1 };
    c.inner.stars().binary_search_by_key(&hr, |s| s.hr).map_or(-1, |i| i as isize)
}

/// Blackbody temperature from B−V (Ballesteros).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_bv_to_temperature_k(b_minus_v: f64) -> f64 {
    astro::bv_to_temperature_k(b_minus_v)
}

/// Normalised linear sRGB of a blackbody.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_blackbody_linear_srgb(temp_k: f64) -> NosimRgb {
    starfield::blackbody_linear_srgb(temp_k).into()
}

/// Normalised linear sRGB for a colour index.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_bv_to_linear_srgb(b_minus_v: f64) -> NosimRgb {
    starfield::bv_to_linear_srgb(b_minus_v).into()
}

// ---- Scenery packages and VFS ----------------------------------------------------------

/// Loaded scenery package (opaque). Mounting clones a reference; the handle stays valid
/// and must still be freed.
pub struct NosimPackage {
    inner: Arc<Package>,
}

/// Prioritised mount table (opaque).
pub struct NosimVfs {
    inner: Vfs,
}

/// Geographic bounding box, degrees.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimBounds {
    /// South edge.
    pub min_lat: f64,
    /// North edge.
    pub max_lat: f64,
    /// West edge.
    pub min_lon: f64,
    /// East edge.
    pub max_lon: f64,
}

impl From<NosimBounds> for Bounds {
    fn from(b: NosimBounds) -> Self {
        Bounds { min_lat: b.min_lat, max_lat: b.max_lat, min_lon: b.min_lon, max_lon: b.max_lon }
    }
}

impl From<&Bounds> for NosimBounds {
    fn from(b: &Bounds) -> Self {
        NosimBounds { min_lat: b.min_lat, max_lat: b.max_lat, min_lon: b.min_lon, max_lon: b.max_lon }
    }
}

/// Which contributed data a resolve asks for.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NosimContentKind {
    /// `content.arinc_overrides`
    ArincOverrides = 0,
    /// `content.spline_networks`
    SplineNetworks = 1,
}

/// A feature tag, key and value NUL-terminated.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NosimTag {
    /// Key, e.g. `highway`.
    pub key: *const c_char,
    /// Value, e.g. `motorway`.
    pub value: *const c_char,
}

/// A placed model: ids and paths are NUL-terminated and truncated to the array size.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NosimModelPlacement {
    /// Model id within its package.
    pub id: [c_char; 64],
    /// Owning package id.
    pub package_id: [c_char; 128],
    /// Absolute path of the mesh file.
    pub mesh_path: [c_char; 512],
    /// Anchor on the ellipsoid.
    pub anchor: NosimGeodetic,
    /// Yaw, degrees clockwise from true north.
    pub true_heading_deg: f64,
}

/// Loads and validates the package directory at `dir`. NULL on failure (see
/// [`nosim_last_error`] for the validator's report).
///
/// # Safety
/// `dir` must be a NUL-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_package_load(dir: *const c_char) -> *mut NosimPackage {
    // SAFETY: documented contract.
    let Ok(dir) = (unsafe { read_cstr(dir) }) else { return ptr::null_mut() };
    match Package::load(dir) {
        Ok(p) => Box::into_raw(Box::new(NosimPackage { inner: Arc::new(p) })),
        Err(e) => {
            let status = match &e {
                nosim::scenery::PackageError::Io { .. } | nosim::scenery::PackageError::MissingFile { .. } => {
                    NosimStatus::Io
                }
                nosim::scenery::PackageError::Json { .. } | nosim::scenery::PackageError::GeoJson { .. } => {
                    NosimStatus::Parse
                }
                nosim::scenery::PackageError::Validation(_) => NosimStatus::Validation,
            };
            fail(status, e.to_string());
            ptr::null_mut()
        }
    }
}

/// Releases a package handle. NULL is ignored.
///
/// # Safety
/// `h` must be NULL or a live handle, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_package_free(h: *mut NosimPackage) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Writes the package id; returns the size needed including the NUL (0 for NULL handle).
///
/// # Safety
/// `h` must be NULL or a live handle; `buf` must be NULL or point to `cap` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_package_id(h: *const NosimPackage, buf: *mut c_char, cap: usize) -> usize {
    // SAFETY: documented contract.
    let Some(p) = (unsafe { opt_ref(h) }) else { return 0 };
    // SAFETY: documented contract.
    unsafe { write_cstr(buf, cap, p.inner.id()) }
}

/// Manifest priority (0 for NULL).
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_package_priority(h: *const NosimPackage) -> i32 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |p| p.inner.priority())
}

/// Claimed bounds (zeros for NULL).
///
/// # Safety
/// `h` must be NULL or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_package_bounds(h: *const NosimPackage) -> NosimBounds {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(NosimBounds::default(), |p| p.inner.bounds().into())
}

/// Creates an empty mount table.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_vfs_new() -> *mut NosimVfs {
    Box::into_raw(Box::new(NosimVfs { inner: Vfs::new() }))
}

/// Releases a mount table (packages it references stay alive until their own free).
///
/// # Safety
/// `h` must be NULL or a live handle, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_free(h: *mut NosimVfs) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Mounts a package at its default tier. A newer version of an already-mounted id
/// replaces it; the same or an older version fails with `InvalidArgument`.
///
/// # Safety
/// Both handles must be live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_mount(vfs: *mut NosimVfs, package: *const NosimPackage) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(v), Some(p)) = (unsafe { opt_mut(vfs) }, unsafe { opt_ref(package) }) else {
        return fail(NosimStatus::NullPointer, "vfs or package is NULL");
    };
    match v.inner.mount(Arc::clone(&p.inner)) {
        Ok(()) => NosimStatus::Ok,
        Err(e) => fail(NosimStatus::InvalidArgument, e.to_string()),
    }
}

/// Unmounts by package id; returns whether it was mounted.
///
/// # Safety
/// `vfs` must be NULL or live; `package_id` a NUL-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_unmount(vfs: *mut NosimVfs, package_id: *const c_char) -> bool {
    // SAFETY: documented contract.
    let (Some(v), Ok(id)) = (unsafe { opt_mut(vfs) }, unsafe { read_cstr(package_id) }) else { return false };
    v.inner.unmount(id)
}

/// Number of mounted packages.
///
/// # Safety
/// `vfs` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_mount_count(vfs: *const NosimVfs) -> usize {
    // SAFETY: documented contract.
    unsafe { opt_ref(vfs) }.map_or(0, |v| v.inner.mounts().len())
}

/// Whether a baseline feature of `layer` at the point, carrying `tags`, is masked by any
/// mounted package. `tags` may be NULL when `tag_count` is 0.
///
/// # Safety
/// `vfs` must be NULL or live; `layer` and every tag string NUL-terminated; `tags` must
/// point to `tag_count` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_is_excluded(
    vfs: *const NosimVfs,
    layer: *const c_char,
    lat_deg: f64,
    lon_deg: f64,
    tags: *const NosimTag,
    tag_count: usize,
) -> bool {
    // SAFETY: documented contract.
    let (Some(v), Ok(layer)) = (unsafe { opt_ref(vfs) }, unsafe { read_cstr(layer) }) else { return false };
    // SAFETY: documented contract.
    let Some(raw) = (unsafe { slice_in(tags, tag_count) }) else { return false };
    let mut owned = Vec::with_capacity(raw.len());
    for t in raw {
        // SAFETY: documented contract.
        let (Ok(k), Ok(val)) = (unsafe { read_cstr(t.key) }, unsafe { read_cstr(t.value) }) else { return false };
        owned.push((k, val));
    }
    v.inner.is_excluded(layer, lat_deg, lon_deg, &owned).is_some()
}

/// Absolute path of the highest-priority package supplying `kind` at the point. Returns
/// the size needed including the NUL, or 0 when no package provides it.
///
/// # Safety
/// `vfs` must be NULL or live; `buf` must be NULL or point to `cap` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_resolve(
    vfs: *const NosimVfs,
    kind: NosimContentKind,
    lat_deg: f64,
    lon_deg: f64,
    buf: *mut c_char,
    cap: usize,
) -> usize {
    // SAFETY: documented contract.
    let Some(v) = (unsafe { opt_ref(vfs) }) else { return 0 };
    let kind = match kind {
        NosimContentKind::ArincOverrides => ContentKind::ArincOverrides,
        NosimContentKind::SplineNetworks => ContentKind::SplineNetworks,
    };
    match v.inner.resolve(kind, lat_deg, lon_deg) {
        // SAFETY: documented contract.
        Some(r) => unsafe { write_cstr(buf, cap, &r.path.to_string_lossy()) },
        None => 0,
    }
}

/// Models anchored inside `area`, in resolution order. Returns the total count; at most
/// `capacity` are written.
///
/// # Safety
/// `vfs` must be NULL or live; `out` must point to `capacity` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_vfs_models_in(
    vfs: *const NosimVfs,
    area: NosimBounds,
    out: *mut NosimModelPlacement,
    capacity: usize,
) -> usize {
    // SAFETY: documented contract.
    let Some(v) = (unsafe { opt_ref(vfs) }) else { return 0 };
    let found = v.inner.models_in(&area.into());
    // SAFETY: documented contract.
    if let Some(dst) = unsafe { slice_out(out, capacity) } {
        for (d, (pkg, m)) in dst.iter_mut().zip(&found) {
            let mut item = NosimModelPlacement {
                id: [0; 64],
                package_id: [0; 128],
                mesh_path: [0; 512],
                anchor: NosimGeodetic {
                    lat_deg: m.anchor_geodetic[0],
                    lon_deg: m.anchor_geodetic[1],
                    h_m: m.anchor_geodetic[2],
                },
                true_heading_deg: m.true_heading_deg,
            };
            fill_array(&mut item.id, &m.id);
            fill_array(&mut item.package_id, pkg.id());
            fill_array(&mut item.mesh_path, &pkg.mesh_path(m).to_string_lossy());
            *d = item;
        }
    }
    found.len()
}

// ---- ARINC 424 -------------------------------------------------------------------------

/// Decoded PG runway record. Optional fields come with a `has_*` flag.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NosimRunwayRecord {
    /// Airport ICAO identifier.
    pub airport_icao: [c_char; 5],
    /// ICAO region code.
    pub icao_region: [c_char; 3],
    /// Runway identifier, e.g. `RW04R`.
    pub runway_ident: [c_char; 6],
    /// Length, feet.
    pub length_ft: f64,
    /// Bearing, degrees.
    pub bearing_deg: f64,
    /// Whether the bearing is true rather than magnetic.
    pub bearing_is_true: bool,
    /// Threshold latitude, degrees.
    pub lat_deg: f64,
    /// Threshold longitude, degrees.
    pub lon_deg: f64,
    /// Whether `gradient_pct` is present.
    pub has_gradient: bool,
    /// Gradient, percent.
    pub gradient_pct: f64,
    /// Threshold elevation, feet.
    pub threshold_elev_ft: f64,
    /// Displaced threshold distance, feet.
    pub displaced_threshold_ft: f64,
    /// Whether `threshold_crossing_height_ft` is present.
    pub has_tch: bool,
    /// Threshold crossing height, feet.
    pub threshold_crossing_height_ft: f64,
    /// Width, feet.
    pub width_ft: f64,
    /// Whether `stopway_ft` is present.
    pub has_stopway: bool,
    /// Stopway, feet.
    pub stopway_ft: f64,
    /// Free-text description.
    pub description: [c_char; 24],
}

/// Runway pavement geometry in ECEF metres.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimRunwayGeometry {
    /// Physical pavement start.
    pub threshold_ecef: NosimVec3,
    /// Physical pavement end.
    pub reciprocal_ecef: NosimVec3,
    /// Landing threshold.
    pub displaced_threshold_ecef: NosimVec3,
    /// Near-left, near-right, far-right, far-left.
    pub corners_ecef: [NosimVec3; 4],
    /// Longitudinal grade, percent.
    pub grade_pct: f64,
    /// Distance between pavement ends.
    pub centerline_length_m: f64,
}

fn runway_to_ffi(r: &RunwayRecord) -> NosimRunwayRecord {
    let mut out = NosimRunwayRecord {
        airport_icao: [0; 5],
        icao_region: [0; 3],
        runway_ident: [0; 6],
        length_ft: r.length_ft,
        bearing_deg: r.bearing_deg,
        bearing_is_true: r.bearing_is_true,
        lat_deg: r.lat_deg,
        lon_deg: r.lon_deg,
        has_gradient: r.gradient_pct.is_some(),
        gradient_pct: r.gradient_pct.unwrap_or(0.0),
        threshold_elev_ft: r.threshold_elev_ft,
        displaced_threshold_ft: r.displaced_threshold_ft,
        has_tch: r.threshold_crossing_height_ft.is_some(),
        threshold_crossing_height_ft: r.threshold_crossing_height_ft.unwrap_or(0.0),
        width_ft: r.width_ft,
        has_stopway: r.stopway_ft.is_some(),
        stopway_ft: r.stopway_ft.unwrap_or(0.0),
        description: [0; 24],
    };
    fill_array(&mut out.airport_icao, &r.airport_icao);
    fill_array(&mut out.icao_region, &r.icao_region);
    fill_array(&mut out.runway_ident, &r.runway_ident);
    fill_array(&mut out.description, &r.description);
    out
}

fn runway_from_ffi(r: &NosimRunwayRecord) -> RunwayRecord {
    RunwayRecord {
        airport_icao: array_to_string(&r.airport_icao),
        icao_region: array_to_string(&r.icao_region),
        runway_ident: array_to_string(&r.runway_ident),
        length_ft: r.length_ft,
        bearing_deg: r.bearing_deg,
        bearing_is_true: r.bearing_is_true,
        lat_deg: r.lat_deg,
        lon_deg: r.lon_deg,
        gradient_pct: r.has_gradient.then_some(r.gradient_pct),
        threshold_elev_ft: r.threshold_elev_ft,
        displaced_threshold_ft: r.displaced_threshold_ft,
        threshold_crossing_height_ft: r.has_tch.then_some(r.threshold_crossing_height_ft),
        width_ft: r.width_ft,
        stopway_ft: r.has_stopway.then_some(r.stopway_ft),
        description: array_to_string(&r.description),
    }
}

/// Decodes one 132-column PG record given as `len` bytes (no NUL needed).
///
/// # Safety
/// `line` must point to `len` bytes; `out` must be valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_arinc424_parse_runway(
    line: *const c_char,
    len: usize,
    out: *mut NosimRunwayRecord,
) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(bytes), Some(out)) = (unsafe { slice_in(line.cast::<u8>(), len) }, unsafe { opt_mut(out) }) else {
        return fail(NosimStatus::NullPointer, "line or output is NULL");
    };
    let Ok(text) = std::str::from_utf8(bytes) else { return fail(NosimStatus::InvalidArgument, "record is not UTF-8") };
    match arinc424::parse_runway_record(text) {
        Ok(r) => {
            *out = runway_to_ffi(&r);
            NosimStatus::Ok
        }
        Err(e) => fail(NosimStatus::Parse, e.to_string()),
    }
}

/// Extrudes a runway along `true_heading_deg`, fitting the far end to `reciprocal_elev_m`.
///
/// # Safety
/// `record` and `out` must be valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_runway_build(
    record: *const NosimRunwayRecord,
    true_heading_deg: f64,
    reciprocal_elev_m: f64,
    out: *mut NosimRunwayGeometry,
) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(r), Some(out)) = (unsafe { opt_ref(record) }, unsafe { opt_mut(out) }) else {
        return fail(NosimStatus::NullPointer, "record or output is NULL");
    };
    let g = arinc424::build_runway(&runway_from_ffi(r), true_heading_deg, reciprocal_elev_m);
    *out = NosimRunwayGeometry {
        threshold_ecef: g.threshold_ecef.into(),
        reciprocal_ecef: g.reciprocal_ecef.into(),
        displaced_threshold_ecef: g.displaced_threshold_ecef.into(),
        corners_ecef: [
            g.corners_ecef[0].into(),
            g.corners_ecef[1].into(),
            g.corners_ecef[2].into(),
            g.corners_ecef[3].into(),
        ],
        grade_pct: g.grade_pct,
        centerline_length_m: g.centerline_length_m,
    };
    NosimStatus::Ok
}

/// Threshold bar count for a runway width in feet (FAA AC 150/5340-1).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_threshold_bar_count(width_ft: f64) -> u32 {
    arinc424::threshold_bar_count(width_ft)
}

/// Opposite-end designator for an identifier such as `RW04R`; 0 if it does not parse.
///
/// # Safety
/// `ident` must be NUL-terminated; `buf` NULL or `cap` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_reciprocal_designator(ident: *const c_char, buf: *mut c_char, cap: usize) -> usize {
    // SAFETY: documented contract.
    let Ok(ident) = (unsafe { read_cstr(ident) }) else { return 0 };
    match arinc424::parse_designator(ident) {
        // SAFETY: documented contract.
        Some(d) => unsafe { write_cstr(buf, cap, &arinc424::reciprocal_designator(&d)) },
        None => 0,
    }
}

// ---- Procedural ------------------------------------------------------------------------

/// Deterministic seed for a 1e-5° cell.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_spatial_seed(lat_deg: f64, lon_deg: f64, global_salt: u64) -> u64 {
    procedural::spatial_seed(lat_deg, lon_deg, global_salt)
}

/// Clamped Poisson building-level estimate.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_estimate_levels(seed: u64, lambda: f64, min_levels: u32, max_levels: u32) -> u32 {
    procedural::estimate_levels(
        seed,
        procedural::ZoneLevels { lambda, min_levels, max_levels: max_levels.max(min_levels) },
    )
}

/// Pier stations along a deck; returns the total count, writes at most `capacity`.
/// `spacing_m <= 0` selects the spec's 35 m.
///
/// # Safety
/// `out` must point to `capacity` doubles.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_pier_stations(
    deck_length_m: f64,
    spacing_m: f64,
    out: *mut f64,
    capacity: usize,
) -> usize {
    let spacing = if spacing_m > 0.0 { spacing_m } else { procedural::PIER_SPACING_M };
    let stations = procedural::pier_stations(deck_length_m, spacing);
    // SAFETY: documented contract.
    if let Some(dst) = unsafe { slice_out(out, capacity) } {
        for (d, s) in dst.iter_mut().zip(&stations) {
            *d = *s;
        }
    }
    stations.len()
}

/// Terrain flattening weight outside pavement; `falloff_m <= 0` selects the spec's 60 m.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_flatten_blend(distance_outside_m: f64, falloff_m: f64) -> f64 {
    procedural::flatten_blend(
        distance_outside_m,
        if falloff_m > 0.0 { falloff_m } else { procedural::RUNWAY_FALLOFF_M },
    )
}

// ---- Phenology -------------------------------------------------------------------------

/// Deciduous canopy phase.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NosimPhase {
    /// Bare branches.
    WinterDefoliation = 0,
    /// Leaves scaling up.
    SpringBudding = 1,
    /// Full canopy.
    SummerCanopy = 2,
    /// Colour transfer.
    AutumnSenescence = 3,
}

/// Local climate inputs.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimClimate {
    /// Local temperature, °C.
    pub t_local_c: f64,
    /// Day of year, 1..=365.
    pub doy: u32,
    /// Hemisphere.
    pub northern_hemisphere: bool,
    /// Precipitation falling.
    pub precipitating: bool,
}

impl From<NosimClimate> for phenology::Climate {
    fn from(c: NosimClimate) -> Self {
        phenology::Climate {
            t_local_c: c.t_local_c,
            doy: c.doy,
            northern_hemisphere: c.northern_hemisphere,
            precipitating: c.precipitating,
        }
    }
}

/// Phenology phase for a climate.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_phenology_classify(climate: NosimClimate) -> NosimPhase {
    match phenology::classify(&climate.into()) {
        phenology::Phase::WinterDefoliation => NosimPhase::WinterDefoliation,
        phenology::Phase::SpringBudding => NosimPhase::SpringBudding,
        phenology::Phase::SummerCanopy => NosimPhase::SummerCanopy,
        phenology::Phase::AutumnSenescence => NosimPhase::AutumnSenescence,
    }
}

/// Leaf geometry scale, 0..=1.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_leaf_scale(climate: NosimClimate) -> f64 {
    phenology::leaf_scale(&climate.into())
}

/// Whether snow is accumulating.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_snow_accumulates(climate: NosimClimate) -> bool {
    phenology::snow_accumulates(&climate.into())
}

/// Snow coverage mask value.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_snow_coverage(normal_dot_up: f64, slope_threshold: f64, depth: f64) -> f64 {
    phenology::snow_coverage(normal_dot_up, slope_threshold, depth)
}

// ---- Traffic ---------------------------------------------------------------------------

/// Intelligent Driver Model parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimIdmParams {
    /// Target speed, m/s.
    pub v0: f64,
    /// Minimum jam distance, m.
    pub s0: f64,
    /// Safe time headway, s.
    pub t: f64,
    /// Maximum acceleration, m/s².
    pub a: f64,
    /// Comfortable deceleration, m/s².
    pub b: f64,
    /// Free-road exponent.
    pub delta: f64,
}

impl From<NosimIdmParams> for traffic::IdmParams {
    fn from(p: NosimIdmParams) -> Self {
        traffic::IdmParams { v0: p.v0, s0: p.s0, t: p.t, a: p.a, b: p.b, delta: p.delta }
    }
}

/// The spec's IDM defaults for a target speed.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_idm_params_default(v0: f64) -> NosimIdmParams {
    let p = traffic::IdmParams::new(v0);
    NosimIdmParams { v0: p.v0, s0: p.s0, t: p.t, a: p.a, b: p.b, delta: p.delta }
}

/// IDM acceleration behind a leader (`gap` net distance, `dv = v − v_lead`).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_idm_acceleration(params: NosimIdmParams, v: f64, gap: f64, dv: f64) -> f64 {
    traffic::idm_acceleration(&params.into(), v, gap, dv)
}

/// IDM acceleration with no leader.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_idm_free_acceleration(params: NosimIdmParams, v: f64) -> f64 {
    traffic::idm_free_acceleration(&params.into(), v)
}

/// MOBIL parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimMobilParams {
    /// Politeness factor.
    pub politeness: f64,
    /// Etiquette threshold, m/s².
    pub threshold: f64,
    /// Safe braking limit for the new follower, m/s².
    pub b_safe: f64,
}

/// The spec's MOBIL defaults.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_mobil_params_default() -> NosimMobilParams {
    let m = traffic::MobilParams::default();
    NosimMobilParams { politeness: m.politeness, threshold: m.threshold, b_safe: m.b_safe }
}

/// Accelerations before and after a hypothetical lane change.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimLaneChangeContext {
    /// Own, after.
    pub self_new: f64,
    /// Own, before.
    pub self_old: f64,
    /// Target-lane follower, after.
    pub new_follower_new: f64,
    /// Target-lane follower, before.
    pub new_follower_old: f64,
    /// Current-lane follower, after.
    pub old_follower_new: f64,
    /// Current-lane follower, before.
    pub old_follower_old: f64,
}

/// MOBIL lane-change decision.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_mobil_should_change(params: NosimMobilParams, ctx: NosimLaneChangeContext) -> bool {
    let m = traffic::MobilParams { politeness: params.politeness, threshold: params.threshold, b_safe: params.b_safe };
    let c = traffic::LaneChangeContext {
        self_new: ctx.self_new,
        self_old: ctx.self_old,
        new_follower_new: ctx.new_follower_new,
        new_follower_old: ctx.new_follower_old,
        old_follower_new: ctx.old_follower_new,
        old_follower_old: ctx.old_follower_old,
    };
    traffic::mobil_should_change(&m, &c)
}

/// Vertex-animation-texture coordinate.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimVatUv {
    /// Column (vertex).
    pub u: f64,
    /// Row (frame).
    pub v: f64,
}

/// VAT texel-centre address for a vertex at a time.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_vat_uv(
    vertex_id: u32,
    total_vertices: u32,
    time_s: f64,
    speed: f64,
    playback_rate: f64,
    frame_count: u32,
) -> NosimVatUv {
    if total_vertices == 0 || frame_count == 0 {
        return NosimVatUv::default();
    }
    let uv = traffic::vat_uv(vertex_id, total_vertices, time_s, speed, playback_rate, frame_count);
    NosimVatUv { u: uv.u, v: uv.v }
}

// ---- Photometry ------------------------------------------------------------------------

/// Hapke model parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimHapkeParams {
    /// Single-scattering albedo.
    pub w: f64,
    /// Opposition surge amplitude.
    pub b0: f64,
    /// Opposition surge width.
    pub h: f64,
    /// Henyey-Greenstein asymmetry.
    pub b: f64,
    /// Backscatter weight.
    pub c: f64,
}

/// Typical lunar highland parameters.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_hapke_params_default() -> NosimHapkeParams {
    let p = photometry::HapkeParams::default();
    NosimHapkeParams { w: p.w, b0: p.b0, h: p.h, b: p.b, c: p.c }
}

/// Hapke bidirectional reflectance.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_hapke_reflectance(params: NosimHapkeParams, mu0: f64, mu: f64, alpha_rad: f64) -> f64 {
    let p = photometry::HapkeParams { w: params.w, b0: params.b0, h: params.h, b: params.b, c: params.c };
    photometry::hapke_reflectance(&p, mu0, mu, alpha_rad)
}

/// Chapman grazing-incidence function, `x = R / H`.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_chapman(x: f64, theta_rad: f64) -> f64 {
    photometry::chapman(x, theta_rad)
}

/// Single-scatter limb inscatter for Earth's Rayleigh atmosphere.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_earth_limb_inscatter(theta_rad: f64, tau_zenith: f64, intensity: f64) -> f64 {
    photometry::earth_limb_inscatter(theta_rad, tau_zenith, intensity)
}

// ---- LOD -------------------------------------------------------------------------------

/// Viewer altitude regime.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NosimAltitudeBand {
    /// 0 – 20 km.
    LowAltitude = 0,
    /// 20 – 100 km.
    Stratosphere = 1,
    /// 100 – 1,000 km.
    LowEarthOrbit = 2,
    /// Beyond 1,000 km.
    Translunar = 3,
}

/// What a band keeps resident.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimBandPolicy {
    /// Terrain quadtree streaming.
    pub stream_terrain_quadtree: bool,
    /// Roads, runways, decals.
    pub render_micro_vectors: bool,
    /// IDM / ORCA agents.
    pub simulate_microscopic_agents: bool,
    /// Volumetric atmosphere.
    pub raymarch_atmosphere: bool,
    /// Global quad-sphere bound.
    pub bind_global_quadsphere: bool,
    /// Single spheroid mesh.
    pub single_spheroid_mesh: bool,
    /// DEM resolution, metres (0 = none).
    pub dem_resolution_m: f64,
}

/// Parent inertial frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NosimParentFrame {
    /// Barycentric.
    Icrf = 0,
    /// Earth-centred.
    Eci = 1,
    /// Moon-centred.
    Mci = 2,
}

fn band_to_ffi(b: lod::AltitudeBand) -> NosimAltitudeBand {
    match b {
        lod::AltitudeBand::LowAltitude => NosimAltitudeBand::LowAltitude,
        lod::AltitudeBand::Stratosphere => NosimAltitudeBand::Stratosphere,
        lod::AltitudeBand::LowEarthOrbit => NosimAltitudeBand::LowEarthOrbit,
        lod::AltitudeBand::Translunar => NosimAltitudeBand::Translunar,
    }
}

/// Band for an altitude above the surface, metres.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_band_for(altitude_m: f64) -> NosimAltitudeBand {
    band_to_ffi(lod::band_for(altitude_m))
}

/// Residency policy for a band.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_policy_for(band: NosimAltitudeBand) -> NosimBandPolicy {
    let b = match band {
        NosimAltitudeBand::LowAltitude => lod::AltitudeBand::LowAltitude,
        NosimAltitudeBand::Stratosphere => lod::AltitudeBand::Stratosphere,
        NosimAltitudeBand::LowEarthOrbit => lod::AltitudeBand::LowEarthOrbit,
        NosimAltitudeBand::Translunar => lod::AltitudeBand::Translunar,
    };
    let p = lod::policy_for(b);
    NosimBandPolicy {
        stream_terrain_quadtree: p.stream_terrain_quadtree,
        render_micro_vectors: p.render_micro_vectors,
        simulate_microscopic_agents: p.simulate_microscopic_agents,
        raymarch_atmosphere: p.raymarch_atmosphere,
        bind_global_quadsphere: p.bind_global_quadsphere,
        single_spheroid_mesh: p.single_spheroid_mesh,
        dem_resolution_m: p.dem_resolution_m,
    }
}

/// Parent frame from distances to the Earth and Moon centres, metres.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_parent_frame_for(dist_to_earth_center_m: f64, dist_to_moon_center_m: f64) -> NosimParentFrame {
    match lod::parent_frame_for(dist_to_earth_center_m, dist_to_moon_center_m) {
        lod::ParentFrame::Icrf => NosimParentFrame::Icrf,
        lod::ParentFrame::Eci => NosimParentFrame::Eci,
        lod::ParentFrame::Mci => NosimParentFrame::Mci,
    }
}

// ---- Time scales -----------------------------------------------------------------------

/// One instant on every scale the simulator needs.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimEpoch {
    /// Civil time.
    pub jd_utc: f64,
    /// Earth-rotation time; feed to `nosim_gast_deg`.
    pub jd_ut1: f64,
    /// Terrestrial Time.
    pub jd_tt: f64,
    /// Barycentric Dynamical Time; feed to the ephemeris functions.
    pub jd_tdb: f64,
    /// ΔT = TT − UT1 used, seconds.
    pub delta_t_seconds: f64,
}

fn time_scale(dut1_seconds: f64) -> nosim::timescale::TimeScale {
    nosim::timescale::TimeScale { dut1_seconds, ..Default::default() }
}

fn epoch_to_ffi(e: nosim::timescale::Epoch) -> NosimEpoch {
    NosimEpoch {
        jd_utc: e.jd_utc,
        jd_ut1: e.jd_ut1,
        jd_tt: e.jd_tt,
        jd_tdb: e.jd_tdb,
        delta_t_seconds: e.delta_t_seconds,
    }
}

/// ΔT = TT − UT1, seconds, at a UTC instant. `dut1_seconds` is UT1 − UTC from IERS
/// Bulletin A, or 0 if unknown (then the error is below 0.9 s from 1972 onward).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_delta_t_seconds(jd_utc: f64, dut1_seconds: f64) -> f64 {
    time_scale(dut1_seconds).delta_t_seconds(jd_utc)
}

/// TAI − UTC at a UTC instant; returns false (and leaves `out` untouched) outside the
/// built-in leap-second table (before 1972 or past its validity date).
///
/// # Safety
/// `out` must be NULL or valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_tai_minus_utc(jd_utc: f64, out: *mut f64) -> bool {
    match time_scale(0.0).tai_minus_utc(jd_utc) {
        Some(v) => {
            // SAFETY: documented contract.
            if let Some(o) = unsafe { opt_mut(out) } {
                *o = v;
            }
            true
        }
        None => false,
    }
}

/// Every time scale for a UTC Julian Date.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_epoch_from_utc(jd_utc: f64, dut1_seconds: f64) -> NosimEpoch {
    epoch_to_ffi(time_scale(dut1_seconds).epoch_from_utc(jd_utc))
}

/// Every time scale for a POSIX timestamp (seconds since 1970-01-01T00:00:00 UTC).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_epoch_from_unix(unix_seconds: f64, dut1_seconds: f64) -> NosimEpoch {
    epoch_to_ffi(time_scale(dut1_seconds).epoch_from_unix(unix_seconds))
}

/// POSIX seconds → JD UTC.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_jd_from_unix(unix_seconds: f64) -> f64 {
    nosim::timescale::jd_from_unix(unix_seconds)
}

/// TT → TDB.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_tt_to_tdb(jd_tt: f64) -> f64 {
    nosim::timescale::tt_to_tdb(jd_tt)
}

// ---- Far-field traffic (CTM) -----------------------------------------------------------

use nosim::traffic::ctm;

/// Triangular fundamental diagram, per lane.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimFundamentalDiagram {
    /// Free-flow speed, m/s.
    pub free_flow_speed: f64,
    /// Capacity, veh/s/lane.
    pub capacity_per_lane: f64,
    /// Jam density, veh/m/lane.
    pub jam_density_per_lane: f64,
}

impl From<NosimFundamentalDiagram> for ctm::FundamentalDiagram {
    fn from(d: NosimFundamentalDiagram) -> Self {
        ctm::FundamentalDiagram {
            free_flow_speed: d.free_flow_speed,
            capacity_per_lane: d.capacity_per_lane,
            jam_density_per_lane: d.jam_density_per_lane,
        }
    }
}

fn diagram_to_ffi(d: ctm::FundamentalDiagram) -> NosimFundamentalDiagram {
    NosimFundamentalDiagram {
        free_flow_speed: d.free_flow_speed,
        capacity_per_lane: d.capacity_per_lane,
        jam_density_per_lane: d.jam_density_per_lane,
    }
}

/// Motorway diagram: 108 km/h, 1,800 veh/h/lane, 7.5 m jam spacing.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ctm_diagram_motorway() -> NosimFundamentalDiagram {
    diagram_to_ffi(ctm::FundamentalDiagram::MOTORWAY)
}

/// Urban diagram: 50 km/h, 1,200 veh/h/lane, 7 m jam spacing.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ctm_diagram_urban() -> NosimFundamentalDiagram {
    diagram_to_ffi(ctm::FundamentalDiagram::URBAN)
}

/// Equilibrium speed at a per-lane density, m/s.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ctm_speed_at_density(diagram: NosimFundamentalDiagram, density_per_lane: f64) -> f64 {
    ctm::FundamentalDiagram::from(diagram).speed_at_density(density_per_lane)
}

/// A far-field road segment (opaque).
pub struct NosimCtmLink {
    inner: ctm::Link,
    boundary: ctm::NearFieldBoundary,
}

/// Per-cell state for rendering density impostors.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimCtmCell {
    /// Vehicles in the cell, all lanes.
    pub vehicles: f64,
    /// Vehicles per metre, all lanes.
    pub density: f64,
    /// Equilibrium speed, m/s.
    pub speed_m_s: f64,
    /// Flow into the cell during the last step, veh/s.
    pub inflow_veh_per_s: f64,
}

/// Microscopic agents to create at the near-field boundary this step.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimSpawnBatch {
    /// Vehicles to create.
    pub count: u32,
    /// Initial speed, m/s.
    pub speed_m_s: f64,
    /// Spacing between them, metres (`INFINITY` when the boundary cell is empty).
    pub spacing_m: f64,
}

/// Creates a link; NULL (with `nosim_last_error`) for non-positive length / step, zero
/// lanes, or an inconsistent diagram.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_ctm_link_new(
    diagram: NosimFundamentalDiagram,
    lanes: u32,
    length_m: f64,
    dt_s: f64,
) -> *mut NosimCtmLink {
    match ctm::Link::new(diagram.into(), lanes, length_m, dt_s) {
        Some(inner) => Box::into_raw(Box::new(NosimCtmLink { inner, boundary: ctm::NearFieldBoundary::new() })),
        None => {
            fail(
                NosimStatus::InvalidArgument,
                "ctm link: length, dt and lanes must be positive and the diagram consistent",
            );
            ptr::null_mut()
        }
    }
}

/// Releases a link. NULL is ignored.
///
/// # Safety
/// `h` must be NULL or a live handle, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_free(h: *mut NosimCtmLink) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Number of cells (0 for NULL).
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_cell_count(h: *const NosimCtmLink) -> usize {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |l| l.inner.cell_count())
}

/// Cell length, metres (0 for NULL).
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_cell_length(h: *const NosimCtmLink) -> f64 {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0.0, |l| l.inner.cell_length_m())
}

/// Index of the cell containing a station along the link, clamped.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_cell_at(h: *const NosimCtmLink, station_m: f64) -> usize {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |l| l.inner.cell_at(station_m))
}

/// Advances one step. `demand` is upstream inflow in veh/s; `supply` is what the downstream
/// side (typically the near field) can accept, veh/s (`INFINITY` for free exit). Either
/// output may be NULL.
///
/// # Safety
/// `h` must be NULL or live; outputs NULL or valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_step(
    h: *mut NosimCtmLink,
    demand_veh_per_s: f64,
    supply_veh_per_s: f64,
    entered: *mut f64,
    exited: *mut f64,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(l) = (unsafe { opt_mut(h) }) else { return fail(NosimStatus::NullPointer, "ctm link is NULL") };
    let flows = l.inner.step(demand_veh_per_s, supply_veh_per_s);
    // SAFETY: documented contract.
    if let Some(e) = unsafe { opt_mut(entered) } {
        *e = flows.entered;
    }
    // SAFETY: documented contract.
    if let Some(x) = unsafe { opt_mut(exited) } {
        *x = flows.exited;
    }
    NosimStatus::Ok
}

/// Converts the vehicles that exited in the last `nosim_ctm_link_step` into whole agents to
/// spawn at the near-field boundary, carrying the fraction forward.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_take_spawns(h: *mut NosimCtmLink, exited_vehicles: f64) -> NosimSpawnBatch {
    // SAFETY: documented contract.
    let Some(l) = (unsafe { opt_mut(h) }) else { return NosimSpawnBatch::default() };
    let b = l.boundary.take_spawns(&l.inner, exited_vehicles);
    NosimSpawnBatch { count: b.count, speed_m_s: b.speed_m_s, spacing_m: b.spacing_m }
}

/// Adds vehicles to a cell (agents leaving the near field); returns the overflow that did
/// not fit under the jam capacity.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_inject(h: *mut NosimCtmLink, cell: usize, vehicles: f64) -> f64 {
    // SAFETY: documented contract.
    match unsafe { opt_mut(h) } {
        Some(l) if cell < l.inner.cell_count() => l.inner.inject(cell, vehicles),
        _ => vehicles.max(0.0),
    }
}

/// Removes up to `vehicles` from a cell; returns how many were removed.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_remove(h: *mut NosimCtmLink, cell: usize, vehicles: f64) -> f64 {
    // SAFETY: documented contract.
    match unsafe { opt_mut(h) } {
        Some(l) if cell < l.inner.cell_count() => l.inner.remove(cell, vehicles),
        _ => 0.0,
    }
}

/// Fills up to `capacity` cell states; returns the cell count.
///
/// # Safety
/// `h` must be NULL or live; `out` must point to `capacity` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_ctm_link_cells(
    h: *const NosimCtmLink,
    out: *mut NosimCtmCell,
    capacity: usize,
) -> usize {
    // SAFETY: documented contract.
    let Some(l) = (unsafe { opt_ref(h) }) else { return 0 };
    // SAFETY: documented contract.
    if let Some(dst) = unsafe { slice_out(out, capacity) } {
        for (i, d) in dst.iter_mut().enumerate().take(l.inner.cell_count()) {
            *d = NosimCtmCell {
                vehicles: l.inner.vehicles()[i],
                density: l.inner.density(i),
                speed_m_s: l.inner.speed(i),
                inflow_veh_per_s: l.inner.flux(i),
            };
        }
    }
    l.inner.cell_count()
}

// ---- Pedestrian crowds (ORCA) ----------------------------------------------------------

use nosim::traffic::orca;

/// A 2D vector, metres or m/s.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimVec2 {
    /// X.
    pub x: f64,
    /// Y.
    pub y: f64,
}

impl From<NosimVec2> for orca::Vec2 {
    fn from(v: NosimVec2) -> Self {
        orca::Vec2::new(v.x, v.y)
    }
}

impl From<orca::Vec2> for NosimVec2 {
    fn from(v: orca::Vec2) -> Self {
        NosimVec2 { x: v.x, y: v.y }
    }
}

/// Per-agent ORCA tuning; see `nosim_orca_agent_params_default`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NosimOrcaAgentParams {
    /// Body radius, m.
    pub radius: f64,
    /// Speed limit, m/s.
    pub max_speed: f64,
    /// Look-ahead for other agents, s.
    pub time_horizon: f64,
    /// Look-ahead for obstacles, s.
    pub time_horizon_obstacle: f64,
    /// Neighbour search radius, m.
    pub neighbor_distance: f64,
    /// Nearest-neighbour cap.
    pub max_neighbors: usize,
}

impl From<NosimOrcaAgentParams> for orca::AgentParams {
    fn from(p: NosimOrcaAgentParams) -> Self {
        orca::AgentParams {
            radius: p.radius,
            max_speed: p.max_speed,
            time_horizon: p.time_horizon,
            time_horizon_obstacle: p.time_horizon_obstacle,
            neighbor_distance: p.neighbor_distance,
            max_neighbors: p.max_neighbors,
        }
    }
}

/// Defaults for a walking adult: 0.3 m, 1.4 m/s, 5 s / 2 s horizons, 10 m, 10 neighbours.
#[unsafe(no_mangle)]
pub extern "C" fn nosim_orca_agent_params_default() -> NosimOrcaAgentParams {
    let p = orca::AgentParams::default();
    NosimOrcaAgentParams {
        radius: p.radius,
        max_speed: p.max_speed,
        time_horizon: p.time_horizon,
        time_horizon_obstacle: p.time_horizon_obstacle,
        neighbor_distance: p.neighbor_distance,
        max_neighbors: p.max_neighbors,
    }
}

/// Position and velocity of one agent.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NosimOrcaAgentState {
    /// Position, m.
    pub position: NosimVec2,
    /// Velocity, m/s.
    pub velocity: NosimVec2,
    /// Radius, m.
    pub radius: f64,
}

/// A crowd simulation (opaque).
pub struct NosimOrcaSim {
    inner: orca::Simulator,
}

/// Creates a crowd simulation stepping by `time_step` seconds (non-positive → 0.1).
#[unsafe(no_mangle)]
pub extern "C" fn nosim_orca_new(time_step: f64) -> *mut NosimOrcaSim {
    Box::into_raw(Box::new(NosimOrcaSim { inner: orca::Simulator::new(time_step) }))
}

/// Releases a simulation. NULL is ignored.
///
/// # Safety
/// `h` must be NULL or a live handle, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_free(h: *mut NosimOrcaSim) {
    if !h.is_null() {
        // SAFETY: handle came from Box::into_raw.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Adds an agent at rest; returns its id, or -1 with `nosim_last_error` set.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_add_agent(
    h: *mut NosimOrcaSim,
    position: NosimVec2,
    params: NosimOrcaAgentParams,
) -> isize {
    // SAFETY: documented contract.
    let Some(s) = (unsafe { opt_mut(h) }) else {
        fail(NosimStatus::NullPointer, "orca sim is NULL");
        return -1;
    };
    match s.inner.add_agent(position.into(), params.into()) {
        Ok(id) => id as isize,
        Err(_) => {
            fail(NosimStatus::InvalidArgument, "orca agent params must be positive and finite");
            -1
        }
    }
}

/// Removes an agent; returns whether it existed.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_remove_agent(h: *mut NosimOrcaSim, id: usize) -> bool {
    // SAFETY: documented contract.
    unsafe { opt_mut(h) }.is_some_and(|s| s.inner.remove_agent(id))
}

/// Live agent count.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_agent_count(h: *const NosimOrcaSim) -> usize {
    // SAFETY: documented contract.
    unsafe { opt_ref(h) }.map_or(0, |s| s.inner.agent_count())
}

/// Sets the velocity an agent wants (from the host's navigation).
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_set_preferred_velocity(
    h: *mut NosimOrcaSim,
    id: usize,
    velocity: NosimVec2,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(s) = (unsafe { opt_mut(h) }) else { return fail(NosimStatus::NullPointer, "orca sim is NULL") };
    match s.inner.set_preferred_velocity(id, velocity.into()) {
        Ok(()) => NosimStatus::Ok,
        Err(_) => fail(NosimStatus::NotFound, format!("no agent {id}")),
    }
}

/// Points an agent at a goal at `speed`, easing inside `slow_radius`.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_set_goal(
    h: *mut NosimOrcaSim,
    id: usize,
    goal: NosimVec2,
    speed: f64,
    slow_radius: f64,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(s) = (unsafe { opt_mut(h) }) else { return fail(NosimStatus::NullPointer, "orca sim is NULL") };
    match s.inner.set_goal(id, goal.into(), speed, slow_radius) {
        Ok(()) => NosimStatus::Ok,
        Err(_) => fail(NosimStatus::NotFound, format!("no agent {id}")),
    }
}

/// Adds a polygonal obstacle: counter-clockwise vertices keep agents outside; two
/// vertices make a wall blocking both sides.
///
/// # Safety
/// `h` must be NULL or live; `vertices` must point to `count` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_add_obstacle(
    h: *mut NosimOrcaSim,
    vertices: *const NosimVec2,
    count: usize,
) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(s) = (unsafe { opt_mut(h) }) else { return fail(NosimStatus::NullPointer, "orca sim is NULL") };
    // SAFETY: documented contract.
    let Some(v) = (unsafe { slice_in(vertices, count) }) else {
        return fail(NosimStatus::NullPointer, "vertices is NULL");
    };
    let pts: Vec<orca::Vec2> = v.iter().map(|p| (*p).into()).collect();
    match s.inner.add_obstacle(&pts) {
        Ok(()) => NosimStatus::Ok,
        Err(_) => fail(NosimStatus::InvalidArgument, "an obstacle needs at least two vertices"),
    }
}

/// Advances the crowd one time step.
///
/// # Safety
/// `h` must be NULL or live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_step(h: *mut NosimOrcaSim) -> NosimStatus {
    // SAFETY: documented contract.
    let Some(s) = (unsafe { opt_mut(h) }) else { return fail(NosimStatus::NullPointer, "orca sim is NULL") };
    s.inner.step();
    NosimStatus::Ok
}

/// Reads an agent's state.
///
/// # Safety
/// `h` must be NULL or live; `out` must be valid for writes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nosim_orca_agent_state(
    h: *const NosimOrcaSim,
    id: usize,
    out: *mut NosimOrcaAgentState,
) -> NosimStatus {
    // SAFETY: documented contract.
    let (Some(s), Some(out)) = (unsafe { opt_ref(h) }, unsafe { opt_mut(out) }) else {
        return fail(NosimStatus::NullPointer, "orca sim or output is NULL");
    };
    let Some(a) = s.inner.agent(id) else { return fail(NosimStatus::NotFound, format!("no agent {id}")) };
    *out = NosimOrcaAgentState { position: a.position.into(), velocity: a.velocity.into(), radius: a.params.radius };
    NosimStatus::Ok
}
