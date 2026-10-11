//! Command-line front end for the world compiler.

use std::process::ExitCode;

use nosim_compiler::{
    Command, USAGE, parse_args, run_areas, run_arinc, run_atmosphere, run_bridges, run_graph, run_landcover, run_osm,
    run_raster, run_tiles, run_validate, validate,
};

fn main() -> ExitCode {
    let command = match parse_args(std::env::args().skip(1)) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match command {
        Command::Validate(args) => {
            let (reports, ok) = run_validate(&args);
            if args.json {
                println!("{}", validate::render_json(&reports, args.strict));
            } else {
                for r in &reports {
                    print!("{}", validate::render(r, args.strict));
                }
                let failed = reports.iter().filter(|r| !r.is_ok() || (args.strict && !r.warnings.is_empty())).count();
                println!("{} package(s) audited, {} failed", reports.len(), failed);
            }
            if ok { ExitCode::SUCCESS } else { ExitCode::FAILURE }
        }
        Command::Graph(args) => match run_graph(&args) {
            Ok(nosim_compiler::GraphRun { summary: s, sim, hybrid, walk, .. }) => {
                println!(
                    "{}: {} spline(s) used ({} other class, {} closed to this mode) → {} segment(s)",
                    args.input.display(),
                    s.splines_used,
                    s.splines_other,
                    s.splines_no_access,
                    s.segments
                );
                println!(
                    "{}: {} node(s), {} directed edge(s), {} boundary node(s); {} strongly connected component(s), the largest with {} node(s){}",
                    args.output.display(),
                    s.nodes,
                    s.edges,
                    s.boundary_nodes,
                    s.components,
                    s.largest_component_nodes,
                    if args.largest_component { " (kept alone)" } else { "" }
                );
                if let Some(m) = &sim
                    && m.conservation_error.abs() > 1e-6 * m.entered.max(1.0)
                {
                    eprintln!(
                        "error: CTM conservation violated: {:.6} vehicles unaccounted for ({:.1} entered, {:.1} exited, {:.1} on the network)",
                        m.conservation_error, m.entered, m.exited, m.on_network
                    );
                    return ExitCode::FAILURE;
                }
                if let (Some(m), Some(min)) = (&sim, args.min_exit_ratio)
                    && m.exit_ratio < min
                {
                    eprintln!(
                        "error: steady-state exit ratio {:.1}% is below the required {:.1}%",
                        100.0 * m.exit_ratio,
                        100.0 * min
                    );
                    return ExitCode::FAILURE;
                }
                if let Some(w) = &walk {
                    println!(
                        "walk: route of {} edge(s) from OSM node {} to {}, {:.2} m ({:.0} m straight); {} pedestrian(s) both ways, {} arrived; mean {:.0} s, slowest {:.0} s, alone {:.0} s; closest approach {:.3} m between bodies",
                        w.route_edges,
                        w.route_ends_osm.0.map_or("-".to_owned(), |n| n.to_string()),
                        w.route_ends_osm.1.map_or("-".to_owned(), |n| n.to_string()),
                        w.route_length_m,
                        w.straight_m,
                        w.walkers,
                        w.arrived,
                        w.mean_time_s,
                        w.max_time_s,
                        w.free_time_s,
                        w.min_clearance_m
                    );
                    if w.min_clearance_m < -1e-6 {
                        eprintln!("error: two pedestrians overlapped by {:.3} m", -w.min_clearance_m);
                        return ExitCode::FAILURE;
                    }
                    if w.arrived < w.walkers {
                        eprintln!(
                            "error: {} pedestrian(s) did not arrive within {} s",
                            w.walkers - w.arrived,
                            args.walk_seconds
                        );
                        return ExitCode::FAILURE;
                    }
                }
                if let Some(h) = &hybrid {
                    println!(
                        "hybrid {} s: {} near-field node(s), {} near-field edge(s); {:.1} veh entered, {:.1} exited; worst conservation error {:.2e}; steady-state exit ratio {:.1}%",
                        h.steps as f64 * args.dt_s,
                        h.near_nodes,
                        h.near_edges,
                        h.entered,
                        h.exited,
                        h.worst_conservation_error,
                        100.0 * h.exit_ratio
                    );
                    println!(
                        "  near field: {} vehicle(s) now, {} at most, {} driven through; min gap {:.2} m; {} clamp(s), {} merge hold(s); {:.3} ms per step",
                        h.near_vehicles,
                        h.peak_near_vehicles,
                        h.near_vehicles_seen,
                        h.min_gap_m,
                        h.clamps,
                        h.holds,
                        h.mean_step_ms
                    );
                    if h.worst_conservation_error > 1e-6 * h.entered.max(1.0) {
                        eprintln!("error: hybrid conservation violated by {:.6} vehicles", h.worst_conservation_error);
                        return ExitCode::FAILURE;
                    }
                    if let Some(min) = args.min_exit_ratio
                        && h.exit_ratio < min
                    {
                        eprintln!(
                            "error: steady-state exit ratio {:.1}% is below the required {:.1}%",
                            100.0 * h.exit_ratio,
                            100.0 * min
                        );
                        return ExitCode::FAILURE;
                    }
                }
                if let Some(m) = sim {
                    println!(
                        "CTM {} s at Δt {} s, {}: {} source(s){}, {} sink(s), {} junction(s), {} link(s) shorter than one step",
                        m.steps as f64 * args.dt_s,
                        args.dt_s,
                        if m.routed { "routed demand" } else { "capacity split" },
                        m.sources,
                        if m.stranded_sources > 0 {
                            format!(" ({} with no reachable exit, idle)", m.stranded_sources)
                        } else {
                            String::new()
                        },
                        m.sinks,
                        m.junctions,
                        m.short_links
                    );
                    println!(
                        "  {:.1} veh entered, {:.1} exited, {:.1} on the network (conservation error {:.2e}); steady-state exit ratio {:.1}%; {} link(s) jammed at the end; peak density {:.0}% of jam",
                        m.entered,
                        m.exited,
                        m.on_network,
                        m.conservation_error,
                        100.0 * m.exit_ratio,
                        m.jammed_links,
                        100.0 * m.peak_density_ratio
                    );
                    for c in m.per_class.iter().take(8) {
                        println!(
                            "  {:<16} {:>9.1} veh-km  {:>8.2} veh-h  {:>6.1} km/h  delay {:>5.1}%",
                            c.class,
                            c.vehicle_km,
                            c.vehicle_hours,
                            c.mean_speed_kmh(),
                            100.0 * c.delay_share()
                        );
                    }
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Osm(args) => match run_osm(&args) {
            Ok((_, s)) => {
                println!(
                    "{}: {} block(s), {} node(s), {} way(s), {} relation(s) skipped; written by {}",
                    args.input.display(),
                    s.blocks,
                    s.nodes,
                    s.ways,
                    s.relations_skipped,
                    s.writing_program.as_deref().unwrap_or("an unnamed program")
                );
                println!(
                    "{}: {} spline(s) ({} road, {} aeroway) from {} matching way(s); {} with a tagged speed",
                    args.output.display(),
                    s.rows,
                    s.road_rows,
                    s.aeroway_rows,
                    s.ways_matched,
                    s.tagged_speeds
                );
                if s.missing_nodes > 0 {
                    eprintln!(
                        "warning: {} reference(s) to nodes not in the file; {} way(s) split, {} dropped",
                        s.missing_nodes, s.ways_split, s.ways_unresolved
                    );
                }
                if s.outside_bbox > 0 {
                    println!("{} spline(s) outside the bounding box dropped", s.outside_bbox);
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Areas(args) => match run_areas(&args) {
            Ok((set, s)) => {
                println!(
                    "{}: {} block(s), {} node(s), {} way(s), {} relation(s); written by {}",
                    args.input.display(),
                    s.blocks,
                    s.nodes,
                    s.ways,
                    s.relations,
                    s.writing_program.as_deref().unwrap_or("an unnamed program")
                );
                println!(
                    "candidates: {} closed way(s), {} multipolygon / boundary relation(s); {} area(s) from relations",
                    s.candidate_ways, s.candidate_relations, s.relations_built
                );
                let count = |l: &str| s.built.get(l).copied().unwrap_or(0);
                println!(
                    "{}: {} building(s), {} water, {} land use, {} coastline piece(s)",
                    args.output.display(),
                    count("building"),
                    count("water"),
                    count("landuse"),
                    s.coastline_rows
                );
                let heights: Vec<String> = s.heights.iter().map(|(k, v)| format!("{v} {k}")).collect();
                if !heights.is_empty() {
                    println!("building heights: {}", heights.join(", "));
                }
                for (package, n) in &s.excluded {
                    println!("{n} building(s) removed by {package}'s procedural_buildings exclusion");
                }
                if s.area_no > 0 {
                    println!("{} way(s) skipped: tagged area=no, or pier and breakwater lines", s.area_no);
                }
                if s.outside_bbox > 0 {
                    println!("{} area(s) outside the bounding box dropped", s.outside_bbox);
                }
                if !set.problems.is_empty() {
                    let kinds: Vec<String> = s.failed.iter().map(|(k, v)| format!("{v} {k}")).collect();
                    eprintln!(
                        "warning: {} area(s) could not be built ({}); listed in {}",
                        set.problems.len(),
                        kinds.join(", "),
                        args.output.join("problems.csv").display()
                    );
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Bridges(args) => match run_bridges(&args) {
            Ok((found, s)) => {
                println!(
                    "{} spline(s), {} tagged bridge spline(s), {} tunnel(s); water: {} polygon(s), {} coastline piece(s)",
                    s.splines, s.tagged_splines, s.tunnels, s.water_polygons, s.coastline_pieces
                );
                println!(
                    "{}: {} tagged deck(s) ({} over water), {} untagged span(s) over water, {} over DEM gorges; {} pier(s)",
                    args.output.display(),
                    s.tagged_decks,
                    s.tagged_over_water,
                    s.water_spans,
                    s.dem_spans,
                    s.piers
                );
                let mut longest: Vec<&nosim_compiler::bridges::Bridge> =
                    found.iter().filter(|b| b.over_water_m > 0.0).collect();
                longest.sort_by(|a, b| b.over_water_m.total_cmp(&a.over_water_m));
                for b in longest.iter().take(5) {
                    println!(
                        "  {} {} (way {}{}): {:.0} m deck, {:.0} m over water, {} pier(s){}",
                        b.source.as_str(),
                        b.name.as_deref().unwrap_or("(unnamed)"),
                        b.osm_ids[0],
                        if b.osm_ids.len() > 1 { format!(" +{}", b.osm_ids.len() - 1) } else { String::new() },
                        b.length_m,
                        b.over_water_m,
                        b.piers.len(),
                        b.max_pier_m.map(|h| format!(", tallest {h:.1} m")).unwrap_or_default()
                    );
                }
                let missed = nosim_compiler::bridges::unmet(&found, &args.expect);
                for m in &missed {
                    eprintln!("error: expected bridge not found over water: {m}");
                }
                for (name, metres) in &args.expect {
                    if !missed.iter().any(|m| m.starts_with(&format!("{name}:"))) {
                        println!("expected: {name} spans at least {metres:.0} m of water");
                    }
                }
                if missed.is_empty() { ExitCode::SUCCESS } else { ExitCode::FAILURE }
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Landcover(args) => match run_landcover(&args) {
            Ok(s) => {
                let (c0, r0, w, h) = s.window;
                println!(
                    "{}: window {w} × {h} pixel(s) at column {c0}, row {r0}; {} unknown code pixel(s)",
                    args.input.display(),
                    s.unknown_pixels
                );
                let total: u64 = s.histogram.values().sum();
                let names: std::collections::HashMap<u8, &str> =
                    nosim_compiler::landcover::WORLDCOVER.iter().copied().collect();
                for (code, n) in &s.histogram {
                    println!(
                        "  {code:>3} {:<24} {n:>10}  {:>5.1}%",
                        names.get(code).copied().unwrap_or(if *code == 0 { "no data" } else { "unknown" }),
                        100.0 * *n as f64 / total as f64
                    );
                }
                let per_zoom: Vec<String> = s.tiles.iter().map(|(z, n)| format!("z{z}: {n}")).collect();
                println!(
                    "{}: class tiles ({}), {} mask tile(s), {} empty tile(s) skipped{}",
                    args.output.display(),
                    per_zoom.join(", "),
                    s.mask_tiles,
                    s.empty_tiles,
                    if s.excluded_pixels > 0 {
                        format!("; vegetation excluded at {} output pixel(s)", s.excluded_pixels)
                    } else {
                        String::new()
                    }
                );
                if s.unknown_pixels > 0 {
                    eprintln!("error: the raster holds codes that are not WorldCover classes");
                    return ExitCode::FAILURE;
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Atmosphere(args) => {
            let start = std::time::Instant::now();
            match run_atmosphere(&args) {
                Ok((t, h)) => {
                    let s = &t.sizes;
                    println!(
                        "{}: transmittance {}x{}, scattering {}x{}x{}x{}, irradiance {}x{}, {} orders, in {:.1} s",
                        args.output.display(),
                        s.transmittance.0,
                        s.transmittance.1,
                        s.scattering_nu,
                        s.scattering_mu_s,
                        s.scattering_mu,
                        s.scattering_r,
                        s.irradiance.0,
                        s.irradiance.1,
                        t.orders,
                        start.elapsed().as_secs_f64()
                    );
                    let (zen, _) = t.sky_radiance(
                        [0.0, 0.0, t.params.bottom_radius + 0.01],
                        [0.0, 0.0, 1.0],
                        [0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2],
                    );
                    println!("zenith sky radiance, sun 45° up: {:.5} {:.5} {:.5} W/m²/sr/nm", zen[0], zen[1], zen[2]);
                    let pct =
                        |v: [f64; 3]| format!("{:.1}% / {:.1}% / {:.1}%", v[0] * 100.0, v[1] * 100.0, v[2] * 100.0);
                    println!(
                        "limb hand-over at {} km, {} rays: shell vs tables RMS {}, integrated over each limb {} (worst), worst single ray {}",
                        h.altitude_km,
                        h.samples,
                        pct(h.rms),
                        pct(h.worst_integrated),
                        pct(h.worst_ray)
                    );
                    println!(
                        "  shell: tau {:.4} {:.4} {:.4}, scale height {:.2} {:.2} {:.2} km",
                        h.shell.tau[0],
                        h.shell.tau[1],
                        h.shell.tau[2],
                        h.shell.scale_height_km[0],
                        h.shell.scale_height_km[1],
                        h.shell.scale_height_km[2]
                    );
                    ExitCode::SUCCESS
                }
                Err(e) => {
                    eprintln!("error: {e}");
                    ExitCode::FAILURE
                }
            }
        }
        Command::Raster(args) => match run_raster(&args) {
            Ok((storage, s, patch)) => {
                if let Some(p) = &patch {
                    println!(
                        "patch: {} runway pavement(s), {} road(s) ({} skipped: bridges, tunnels, other layers or off the DEM), {} strip(s){}",
                        p.runways,
                        p.roads,
                        p.roads_skipped,
                        p.strips,
                        args.patch
                            .patched_dem
                            .as_ref()
                            .map_or(String::new(), |d| format!("; patched DEM written to {}", d.display()))
                    );
                }
                println!(
                    "{}: DEM {}×{} ({} no-data pixel(s); compression {}, predictor {}, sample format {} / {} bits, {}), bounds {:.5} {:.5} {:.5} {:.5}",
                    args.input.display(),
                    s.dem_size.0,
                    s.dem_size.1,
                    s.nodata_pixels,
                    storage.compression,
                    storage.predictor,
                    storage.sample_format,
                    storage.bits,
                    if storage.tiled { "tiled" } else { "stripped" },
                    s.bounds.0,
                    s.bounds.1,
                    s.bounds.2,
                    s.bounds.3
                );
                println!(
                    "{}: {} terrain-rgb tile(s), {} normal-map tile(s), {} mesh tile(s) with {} vertices / {} triangles (max error {:.3} m), {} empty tile(s) skipped",
                    args.output.display(),
                    s.terrain_rgb_tiles,
                    s.normal_tiles,
                    s.mesh_tiles,
                    s.mesh_vertices,
                    s.mesh_triangles,
                    s.mesh_max_error_m,
                    s.empty_tiles_skipped
                );
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Tiles(args) => match run_tiles(&args) {
            Ok((source, summary)) => {
                println!(
                    "{}: {} tile(s) at z{}..={} from {} feature(s) ({} instance(s) written, {} dropped as degenerate)",
                    args.output.display(),
                    summary.tiles,
                    args.options.min_zoom,
                    args.options.max_zoom,
                    summary.features,
                    summary.feature_instances,
                    summary.dropped_instances
                );
                println!(
                    "geometry column {:?}; properties: {}",
                    source.geometry_column,
                    if source.property_columns.is_empty() {
                        "none".to_owned()
                    } else {
                        source.property_columns.join(", ")
                    }
                );
                if !source.skipped_columns.is_empty() {
                    eprintln!("warning: non-scalar column(s) skipped: {}", source.skipped_columns.join(", "));
                }
                if source.rows_without_geometry > 0 {
                    eprintln!("warning: {} row(s) without a decodable geometry", source.rows_without_geometry);
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Command::Arinc(args) => match run_arinc(&args) {
            Ok((rows, summary, overrides)) => {
                println!(
                    "{}: {} runway ends from {} airports ({} PG records read, {} continuations skipped, {} other records)",
                    args.output.display(),
                    rows.len(),
                    summary.airports,
                    summary.runways_read,
                    summary.continuations_skipped,
                    summary.other_records
                );
                for (line, why) in &summary.unparsed {
                    eprintln!("warning: line {line}: {why}");
                }
                if !summary.unpaired.is_empty() {
                    eprintln!(
                        "warning: {} runway(s) without a reciprocal record: {}",
                        summary.unpaired.len(),
                        summary.unpaired.join(", ")
                    );
                }
                if !summary.no_variation.is_empty() {
                    eprintln!(
                        "warning: {} runway(s) at airports without a PA record; magnetic bearing used as true: {}",
                        summary.no_variation.len(),
                        summary.no_variation.join(", ")
                    );
                }
                for id in &overrides.packages_mounted {
                    println!("mounted {id}");
                }
                for (airport, package, n) in &overrides.airports_overridden {
                    println!("{airport}: {n} runway end(s) from {package}");
                }
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("error: {e}");
                if matches!(e, nosim_compiler::CompileError::Usage(_)) {
                    eprintln!("{USAGE}");
                }
                ExitCode::FAILURE
            }
        },
    }
}
