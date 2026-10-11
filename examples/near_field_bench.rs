//! Times the near-field vehicle simulation: 2,000 vehicles on a closed three-lane ring of
//! near edges (no far field), after a minute to settle, averaged over 600 substeps.
//!
//! ```sh
//! cargo run --release --example near_field_bench [vehicles]
//! ```

use std::time::Instant;

use nosim::traffic::ctm::{FundamentalDiagram, GraphEdge};
use nosim::traffic::hybrid::{Hybrid, HybridConfig};

fn main() {
    let n: usize = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(2000);
    // Ring of 20 edges of 1 km, three lanes: 60 km of lane, one vehicle per 30 m.
    let edges: Vec<GraphEdge> = (0..20)
        .map(|k| GraphEdge {
            from: k,
            to: (k + 1) % 20,
            length_m: 1000.0,
            lanes: 3,
            diagram: FundamentalDiagram::URBAN,
            reverse: None,
        })
        .collect();
    let cfg = HybridConfig::default();
    let mut h = Hybrid::new(20, &edges, &[true; 20], &|_, _| f64::NAN, cfg).unwrap();
    let spacing = 60_000.0 / n as f64;
    for i in 0..n {
        let along = i as f64 * spacing;
        let (lane, pos) = (i % 3, along / 3.0 * 3.0);
        let (edge, s) = (((pos / 1000.0) as usize) % 20, pos % 1000.0);
        h.insert_vehicle(edge, lane, s, 8.0);
    }
    for _ in 0..60 {
        h.step();
    }
    let steps = 120;
    let t0 = Instant::now();
    for _ in 0..steps {
        h.step();
    }
    let per_step = t0.elapsed().as_secs_f64() / f64::from(steps);
    let per_sub = per_step / f64::from(cfg.substeps);
    println!(
        "{} vehicles: {:.3} ms per {}-substep step ({:.3} ms per substep); min gap {:.2} m; clamps {}; conservation error {:.1e}",
        h.near_count(),
        per_step * 1e3,
        cfg.substeps,
        per_sub * 1e3,
        h.min_gap(),
        h.clamps,
        h.totals().conservation_error()
    );
}
