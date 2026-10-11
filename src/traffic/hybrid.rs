//! Hybrid traffic (spec §7): individual vehicles near the camera, the Cell Transmission Model
//! beyond, on one road graph, with vehicles handed between them without loss.
//!
//! Graph nodes inside the near field are marked by the caller. Edges between two near nodes
//! carry individual vehicles: IDM car following with the edge's free-flow speed as `v₀`, and
//! MOBIL lane changes on multi-lane edges. Every other edge is a CTM link. At a near node, an
//! incoming CTM link ends in a sink and an outgoing one starts at a source, so the far field
//! never routes through the near field:
//!
//! - **Far to near.** Flow leaving an incoming boundary link accumulates; each whole vehicle
//!   is placed at the start of the near edge its turning choice picks, once there is room.
//!   While a vehicle waits for room, the link's exit closes, so congestion spills back into
//!   the far field.
//! - **Near to far.** A vehicle reaching the end of a near edge whose next edge is an outgoing
//!   boundary link goes into that link's entry buffer, which the CTM drains as its receiving
//!   flow allows. While a whole vehicle is still waiting there, the edge end acts as a
//!   stopped obstacle for the vehicles behind.
//!
//! At every near node a vehicle picks its next edge at random in proportion to the same
//! turning weights the CTM junctions use, from a seeded generator, so runs are reproducible.
//! Intersection control (signals, yielding to crossing traffic) is not modelled: vehicles enter
//! a junction's exit whenever there is a safe gap on it.

use super::ctm::{self, GraphEdge, Network};
use super::{IdmParams, LaneChangeContext, MobilParams, idm_acceleration, idm_free_acceleration, mobil_should_change};

/// Hybrid parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HybridConfig {
    /// CTM time step, seconds.
    pub dt_s: f64,
    /// Near-field substeps per CTM step.
    pub substeps: u32,
    /// IDM parameters; `v0` is replaced by each edge's free-flow speed.
    pub idm: IdmParams,
    /// MOBIL parameters.
    pub mobil: MobilParams,
    /// Vehicle length, metres.
    pub vehicle_length_m: f64,
    /// Minimum time between two lane changes of one vehicle, seconds.
    pub lane_change_cooldown_s: f64,
    /// Seed of the turning-choice generator.
    pub seed: u64,
}

impl Default for HybridConfig {
    fn default() -> Self {
        Self {
            dt_s: 1.0,
            substeps: 5,
            idm: IdmParams::new(13.9),
            mobil: MobilParams::default(),
            vehicle_length_m: 4.5,
            lane_change_cooldown_s: 3.0,
            seed: 1,
        }
    }
}

/// Why a hybrid network could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HybridError {
    /// The far-field network could not be built.
    Far(ctm::BuildError),
    /// The near-node mask does not cover every node.
    MaskLength,
    /// The configuration has a non-positive step, substep count or vehicle length.
    BadConfig,
}

/// Where a near-field vehicle goes when it reaches the end of its edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Next {
    /// Another near edge.
    Edge(u32),
    /// The entry buffer of an outgoing boundary link (index into `outgoing`).
    Out(u32),
    /// A graph sink: the vehicle leaves the network.
    Exit,
}

/// One near-field vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vehicle {
    /// Stable id.
    pub id: u64,
    /// Distance along its edge from the upstream end, metres (front bumper).
    pub s: f64,
    /// Speed, m/s.
    pub v: f64,
    next: Next,
    cooldown: f64,
}

/// An incoming boundary link: CTM link whose downstream end is a near node.
#[derive(Clone, Debug, PartialEq)]
struct Incoming {
    edge: u32,
    link: usize,
    sink_node: usize,
    /// Fractional vehicles that left the link and are not yet whole.
    pending: f64,
    /// Whole vehicles waiting for room on their first near edge, with that edge chosen.
    queue: Vec<Next>,
}

/// An outgoing boundary link: CTM link whose upstream end is a near node.
#[derive(Clone, Debug, PartialEq)]
struct Outgoing {
    link: usize,
    source_node: usize,
    /// Vehicles that left the near field and have not yet entered the link.
    buffer: f64,
}

/// A graph source inside the near field.
#[derive(Clone, Debug, PartialEq)]
struct NearSource {
    node: u32,
    rate: f64,
    pending: f64,
    queue: Vec<Next>,
}

/// Vehicle totals, for conservation checks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Totals {
    /// Vehicles on CTM links.
    pub far: f64,
    /// Vehicles on near edges.
    pub near: f64,
    /// Vehicles between the two: fractional and queued arrivals, and entry buffers.
    pub in_transfer: f64,
    /// Vehicles that entered at any source so far.
    pub entered: f64,
    /// Vehicles that left at any sink so far.
    pub exited: f64,
}

impl Totals {
    /// `entered − exited − (far + near + in_transfer)`: zero up to rounding.
    pub fn conservation_error(&self) -> f64 {
        self.entered - self.exited - (self.far + self.near + self.in_transfer)
    }
}

/// A running sum with Kahan compensation, so totals over millions of small terms stay exact
/// to rounding of the final value.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Sum {
    total: f64,
    carry: f64,
}

impl Sum {
    fn add(&mut self, x: f64) {
        let y = x - self.carry;
        let t = self.total + y;
        self.carry = (t - self.total) - y;
        self.total = t;
    }
}

/// The hybrid network.
pub struct Hybrid {
    cfg: HybridConfig,
    edges: Vec<GraphEdge>,
    /// Near edge → its lanes (vehicles sorted by ascending `s`); empty for CTM edges.
    lanes: Vec<Vec<Vec<Vehicle>>>,
    is_near: Vec<bool>,
    far: Network,
    /// CTM link of each edge, `None` for near edges.
    link_of: Vec<Option<usize>>,
    incoming: Vec<Incoming>,
    outgoing: Vec<Outgoing>,
    /// Outgoing boundary index of each edge.
    outgoing_of: Vec<Option<u32>>,
    near_sources: Vec<NearSource>,
    /// Far-field `(graph node, network source node, link)` for each far source.
    far_sources: Vec<(u32, usize, usize)>,
    /// Network sink nodes and links of the far field's own sinks.
    far_sinks: Vec<(usize, usize)>,
    /// Edges leaving each node, with turning weights from each incoming edge.
    outs: Vec<Vec<u32>>,
    weights: std::collections::HashMap<(u32, u32), f64>,
    rng: u64,
    next_id: u64,
    entered: Sum,
    exited: Sum,
    /// Positions clamped to avoid an overlap (should stay zero).
    pub clamps: u64,
    /// Arrivals held at the end of their edge because the merge had no room.
    pub holds: u64,
    /// Front vehicle of every near lane heading into each `(edge, lane)`, as
    /// `(distance to the merge, speed, from edge, from lane)`; rebuilt every substep.
    merging: std::collections::HashMap<(u32, u32), Vec<Rival>>,
}

/// A vehicle approaching a merge: distance to it, speed, and the edge and lane it is on.
type Rival = (f64, f64, u32, u32);

fn capacity(e: &GraphEdge) -> f64 {
    e.diagram.capacity_per_lane * f64::from(e.lanes)
}

impl Hybrid {
    /// Builds the hybrid network. `near_node[v]` marks the near field; `weight(i, o)` gives the
    /// turning weight from edge `i` to edge `o` (as for [`ctm::build_network_with_turns`]).
    pub fn new(
        node_count: usize,
        edges: &[GraphEdge],
        near_node: &[bool],
        weight: &dyn Fn(usize, usize) -> f64,
        cfg: HybridConfig,
    ) -> Result<Hybrid, HybridError> {
        if near_node.len() != node_count {
            return Err(HybridError::MaskLength);
        }
        if !(cfg.dt_s > 0.0 && cfg.substeps > 0 && cfg.vehicle_length_m > 0.0) {
            return Err(HybridError::BadConfig);
        }
        let is_near: Vec<bool> = edges.iter().map(|e| near_node[e.from] && near_node[e.to]).collect();
        // Far field: every non-near edge, with boundary ends at near nodes given fresh nodes so
        // they become sinks (incoming) and sources (outgoing) instead of junctions.
        let mut far_edges = Vec::new();
        let mut far_index = Vec::new();
        let mut next_node = node_count;
        for (k, e) in edges.iter().enumerate() {
            if is_near[k] {
                continue;
            }
            let mut f = *e;
            if near_node[e.to] {
                f.to = next_node;
                next_node += 1;
            }
            if near_node[e.from] {
                f.from = next_node;
                next_node += 1;
            }
            f.reverse = None;
            far_index.push(k);
            far_edges.push(f);
        }
        // Keep reverse links among far edges so the U-turn rule still applies there.
        let mut far_pos = vec![None; edges.len()];
        for (j, &k) in far_index.iter().enumerate() {
            far_pos[k] = Some(j);
        }
        for (j, &k) in far_index.iter().enumerate() {
            far_edges[j].reverse = edges[k].reverse.and_then(|r| far_pos[r]);
        }
        let far_weight = |i: usize, o: usize| weight(far_index[i], far_index[o]);
        let built =
            ctm::build_network_with_turns(next_node, &far_edges, cfg.dt_s, &far_weight).map_err(HybridError::Far)?;
        let mut link_of = vec![None; edges.len()];
        for (j, &k) in far_index.iter().enumerate() {
            link_of[k] = Some(j);
        }
        let (mut incoming, mut outgoing, mut far_sources, mut far_sinks) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut outgoing_of = vec![None; edges.len()];
        for &(node, link) in &built.sinks {
            let k = far_index[link];
            if near_node[edges[k].to] {
                incoming.push(Incoming { edge: k as u32, link, sink_node: node, pending: 0.0, queue: Vec::new() });
            } else {
                far_sinks.push((node, link));
            }
        }
        for &(node, link) in &built.sources {
            let k = far_index[link];
            if near_node[edges[k].from] {
                outgoing_of[k] = Some(outgoing.len() as u32);
                outgoing.push(Outgoing { link, source_node: node, buffer: 0.0 });
            } else {
                far_sources.push((edges[k].from as u32, node, link));
            }
        }
        let mut outs: Vec<Vec<u32>> = vec![Vec::new(); node_count];
        let mut ins = vec![0u32; node_count];
        for (k, e) in edges.iter().enumerate() {
            outs[e.from].push(k as u32);
            ins[e.to] += 1;
        }
        let near_sources = (0..node_count)
            .filter(|&v| near_node[v] && ins[v] == 0 && !outs[v].is_empty())
            .map(|v| NearSource { node: v as u32, rate: 0.0, pending: 0.0, queue: Vec::new() })
            .collect();
        // Turning weights for every movement through a near node, with the same fallback as
        // the CTM (capacity-proportional, no U-turn unless it is the only way on).
        let mut weights = std::collections::HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            if !near_node[e.to] {
                continue;
            }
            let options = &outs[e.to];
            let given: Vec<f64> = options.iter().map(|&o| weight(i, o as usize)).collect();
            let row: Vec<f64> = if given.iter().all(|w| w.is_finite() && *w >= 0.0) && given.iter().sum::<f64>() > 0.0 {
                given
            } else {
                let no_uturn = options.iter().any(|&o| e.reverse != Some(o as usize));
                options
                    .iter()
                    .map(
                        |&o| if no_uturn && e.reverse == Some(o as usize) { 0.0 } else { capacity(&edges[o as usize]) },
                    )
                    .collect()
            };
            for (&o, w) in options.iter().zip(row) {
                weights.insert((i as u32, o), w);
            }
        }
        let lanes = edges
            .iter()
            .enumerate()
            .map(|(k, e)| if is_near[k] { vec![Vec::new(); e.lanes.max(1) as usize] } else { Vec::new() })
            .collect();
        Ok(Hybrid {
            cfg,
            edges: edges.to_vec(),
            lanes,
            is_near,
            far: built.network,
            link_of,
            incoming,
            outgoing,
            outgoing_of,
            near_sources,
            far_sources,
            far_sinks,
            outs,
            weights,
            rng: cfg.seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            next_id: 0,
            entered: Sum::default(),
            exited: Sum::default(),
            clamps: 0,
            holds: 0,
            merging: std::collections::HashMap::new(),
        })
    }

    /// Graph nodes that are sources (far field and near field).
    pub fn source_nodes(&self) -> Vec<u32> {
        let mut v: Vec<u32> =
            self.far_sources.iter().map(|s| s.0).chain(self.near_sources.iter().map(|s| s.node)).collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// Sets the demand of every source at a graph node, vehicles per second.
    pub fn set_source_rate(&mut self, node: u32, veh_per_s: f64) {
        for &(n, net_node, _) in &self.far_sources {
            if n == node {
                self.far.set_rate(net_node, veh_per_s);
            }
        }
        for s in &mut self.near_sources {
            if s.node == node {
                s.rate = veh_per_s.max(0.0);
            }
        }
    }

    /// Whether edge `k` is simulated vehicle by vehicle.
    pub fn is_near_edge(&self, k: usize) -> bool {
        self.is_near[k]
    }

    /// The far-field network (read-only).
    pub fn far(&self) -> &Network {
        &self.far
    }

    /// CTM link of an edge, if it is in the far field.
    pub fn link_of(&self, edge: usize) -> Option<usize> {
        self.link_of[edge]
    }

    /// Near-field vehicles as `(edge, lane, vehicle)`.
    pub fn vehicles(&self) -> impl Iterator<Item = (usize, usize, &Vehicle)> {
        self.lanes
            .iter()
            .enumerate()
            .flat_map(|(k, lanes)| lanes.iter().enumerate().flat_map(move |(l, vs)| vs.iter().map(move |v| (k, l, v))))
    }

    /// Number of near-field vehicles.
    pub fn near_count(&self) -> usize {
        self.lanes.iter().flatten().map(Vec::len).sum()
    }

    /// Current totals.
    pub fn totals(&self) -> Totals {
        let mut t = Totals { entered: self.entered.total, exited: self.exited.total, ..Totals::default() };
        t.far = self.far.total_vehicles();
        t.near = self.near_count() as f64;
        t.in_transfer = self.incoming.iter().map(|i| i.pending + i.queue.len() as f64).sum::<f64>()
            + self.outgoing.iter().map(|o| o.buffer).sum::<f64>()
            + self.near_sources.iter().map(|s| s.pending + s.queue.len() as f64).sum::<f64>();
        t
    }

    fn random(&mut self) -> f64 {
        // xorshift64*
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Where a vehicle arriving at the end of `edge` goes next.
    fn choose_after(&mut self, edge: u32) -> Next {
        let node = self.edges[edge as usize].to;
        let options = self.outs[node].clone();
        if options.is_empty() {
            return Next::Exit;
        }
        let w: Vec<f64> = options.iter().map(|&o| self.weights.get(&(edge, o)).copied().unwrap_or(0.0)).collect();
        let total: f64 = w.iter().sum();
        let pick = if total > 0.0 {
            let mut r = self.random() * total;
            let mut chosen = options[options.len() - 1];
            for (&o, &x) in options.iter().zip(&w) {
                if r < x {
                    chosen = o;
                    break;
                }
                r -= x;
            }
            chosen
        } else {
            options[0]
        };
        self.target(pick)
    }

    /// The first move from a source node inside the near field: by capacity.
    fn choose_from_source(&mut self, node: u32) -> Next {
        let options = self.outs[node as usize].clone();
        let total: f64 = options.iter().map(|&o| capacity(&self.edges[o as usize])).sum();
        let mut r = self.random() * total;
        for &o in &options {
            let c = capacity(&self.edges[o as usize]);
            if r < c {
                return self.target(o);
            }
            r -= c;
        }
        self.target(options[options.len() - 1])
    }

    fn target(&self, edge: u32) -> Next {
        match self.outgoing_of[edge as usize] {
            Some(o) => Next::Out(o),
            None if self.is_near[edge as usize] => Next::Edge(edge),
            None => Next::Exit, // unreachable: a near node's exit is near or outgoing
        }
    }

    /// Room for a new vehicle at the start of a near edge: the lane with the most space whose
    /// last vehicle is clear of the entry by at least the jam gap. Returns `(lane, entry speed)`.
    fn entry_room(&self, edge: u32) -> Option<(usize, f64)> {
        let e = &self.edges[edge as usize];
        let need = self.cfg.idm.s0 + self.cfg.vehicle_length_m;
        let mut best: Option<(usize, f64, f64)> = None;
        for (l, vs) in self.lanes[edge as usize].iter().enumerate() {
            let (gap, v) = match vs.first() {
                Some(last) => (last.s - self.cfg.vehicle_length_m, last.v),
                None => (e.length_m, e.diagram.free_flow_speed),
            };
            if gap >= need && best.is_none_or(|b| gap > b.1) {
                best = Some((l, gap, v.min(e.diagram.free_flow_speed)));
            }
        }
        best.map(|(l, _, v)| (l, v))
    }

    fn place(&mut self, edge: u32, lane: usize, v: f64) {
        let next = self.choose_after(edge);
        let veh = Vehicle { id: self.next_id, s: 0.0, v, next, cooldown: 0.0 };
        self.next_id += 1;
        self.lanes[edge as usize][lane].insert(0, veh);
    }

    /// Places a vehicle directly (tests and engine-side spawns). It counts as entered.
    pub fn insert_vehicle(&mut self, edge: usize, lane: usize, s: f64, v: f64) {
        assert!(self.is_near[edge], "edge {edge} is not a near-field edge");
        let next = self.choose_after(edge as u32);
        let veh = Vehicle { id: self.next_id, s, v, next, cooldown: 0.0 };
        self.next_id += 1;
        let vs = &mut self.lanes[edge][lane];
        let at = vs.partition_point(|x| x.s < s);
        vs.insert(at, veh);
        self.entered.add(1.0);
    }

    /// Gap and speed of whatever is ahead of a vehicle at `s` in `lane` of `edge` whose next
    /// move is `next`, with `ahead` the next vehicle in the same lane if any.
    fn leader(&self, edge: usize, lane: usize, s: f64, ahead: Option<&Vehicle>, next: Next) -> Option<(f64, f64)> {
        let len = self.cfg.vehicle_length_m;
        if let Some(a) = ahead {
            return Some((a.s - len - s, a.v));
        }
        let to_end = self.edges[edge].length_m - s;
        match next {
            Next::Exit => None,
            Next::Out(o) => (self.outgoing[o as usize].buffer >= 1.0).then_some((to_end, 0.0)),
            Next::Edge(n) => {
                let nl = lane.min(self.lanes[n as usize].len() - 1);
                let mut best = self.lanes[n as usize][nl].first().map(|b| (to_end + b.s - len, b.v));
                // Zipper merge: a vehicle on another approach to the same lane that is nearer the
                // merge point goes first, so it is treated as the leader.
                if let Some(rivals) = self.merging.get(&(n, nl as u32)) {
                    for &(d, v, e, l) in rivals {
                        let ahead_of_us = d < to_end || (d == to_end && (e, l) < (edge as u32, lane as u32));
                        if (e, l) != (edge as u32, lane as u32) && ahead_of_us {
                            let gap = to_end - d - len;
                            if best.is_none_or(|(g, _)| gap < g) {
                                best = Some((gap, v));
                            }
                        }
                    }
                }
                best
            }
        }
    }

    /// Rebuilds the merge index from the current front vehicles.
    fn index_merges(&mut self) {
        self.merging.clear();
        for k in 0..self.lanes.len() {
            for l in 0..self.lanes[k].len() {
                if let Some(front) = self.lanes[k][l].last()
                    && let Next::Edge(n) = front.next
                {
                    let nl = l.min(self.lanes[n as usize].len() - 1) as u32;
                    let d = self.edges[k].length_m - front.s;
                    self.merging.entry((n, nl)).or_default().push((d, front.v, k as u32, l as u32));
                }
            }
        }
    }

    fn accel(&self, edge: usize, v: f64, lead: Option<(f64, f64)>) -> f64 {
        let p = IdmParams { v0: self.edges[edge].diagram.free_flow_speed, ..self.cfg.idm };
        match lead {
            Some((gap, lv)) => idm_acceleration(&p, v, gap, v - lv),
            None => idm_free_acceleration(&p, v),
        }
    }

    /// One near-field substep; lane changes are considered only when `lane_changes` is set
    /// (once per CTM step).
    fn substep(&mut self, dt: f64, lane_changes: bool) {
        let len = self.cfg.vehicle_length_m;
        self.index_merges();
        let mut changed = false;
        // 1. MOBIL lane changes on multi-lane edges, decided on the current state.
        for k in 0..self.lanes.len() {
            if !lane_changes || self.lanes[k].len() < 2 {
                continue;
            }
            let mut moves: Vec<(usize, usize, usize)> = Vec::new(); // (from lane, index, to lane)
            for l in 0..self.lanes[k].len() {
                for i in 0..self.lanes[k][l].len() {
                    let me = self.lanes[k][l][i];
                    if me.cooldown > 0.0 {
                        continue;
                    }
                    let ahead = self.lanes[k][l].get(i + 1);
                    let self_old = self.accel(k, me.v, self.leader(k, l, me.s, ahead, me.next));
                    for t in [l.wrapping_sub(1), l + 1] {
                        if t >= self.lanes[k].len() {
                            continue;
                        }
                        let target = &self.lanes[k][t];
                        let j = target.partition_point(|x| x.s < me.s);
                        let new_leader = target.get(j);
                        let new_follower = if j > 0 { target.get(j - 1) } else { None };
                        // Safe gaps both ways, including a leader just past the end of this edge.
                        let lead = self.leader(k, t, me.s, new_leader, me.next);
                        if lead.is_some_and(|(gap, _)| gap < self.cfg.idm.s0)
                            || new_follower.is_some_and(|nf| me.s - len - nf.s < self.cfg.idm.s0)
                        {
                            continue;
                        }
                        let self_new = self.accel(k, me.v, lead);
                        let (nf_new, nf_old) = match new_follower {
                            Some(nf) => (
                                self.accel(k, nf.v, Some((me.s - len - nf.s, me.v))),
                                self.accel(k, nf.v, self.leader(k, t, nf.s, new_leader, nf.next)),
                            ),
                            None => (0.0, 0.0),
                        };
                        let (of_new, of_old) = if i > 0 {
                            let of = &self.lanes[k][l][i - 1];
                            (
                                self.accel(k, of.v, self.leader(k, l, of.s, ahead, of.next)),
                                self.accel(k, of.v, Some((me.s - len - of.s, me.v))),
                            )
                        } else {
                            (0.0, 0.0)
                        };
                        let ctx = LaneChangeContext {
                            self_new,
                            self_old,
                            new_follower_new: nf_new,
                            new_follower_old: nf_old,
                            old_follower_new: of_new,
                            old_follower_old: of_old,
                        };
                        if mobil_should_change(&self.cfg.mobil, &ctx) {
                            moves.push((l, i, t));
                            break;
                        }
                    }
                }
            }
            // Apply, highest index first so indices stay valid; re-check the gaps since earlier
            // moves may have filled them.
            moves.sort_by_key(|m| std::cmp::Reverse((m.0, m.1)));
            for (l, i, t) in moves {
                let mut me = self.lanes[k][l][i];
                let target = &self.lanes[k][t];
                let j = target.partition_point(|x| x.s < me.s);
                let lead_ok =
                    self.leader(k, t, me.s, target.get(j), me.next).is_none_or(|(gap, _)| gap >= self.cfg.idm.s0);
                let follow_ok = j == 0 || me.s - len - target[j - 1].s >= self.cfg.idm.s0;
                if !(lead_ok && follow_ok) {
                    continue;
                }
                self.lanes[k][l].remove(i);
                me.cooldown = self.cfg.lane_change_cooldown_s;
                self.lanes[k][t].insert(j, me);
                changed = true;
            }
        }
        // 2. Accelerations after the lane changes, so every vehicle brakes for its real leader.
        if changed {
            self.index_merges();
        }
        let mut acc: Vec<Vec<Vec<f64>>> = Vec::with_capacity(self.lanes.len());
        for k in 0..self.lanes.len() {
            let mut per_lane = Vec::with_capacity(self.lanes[k].len());
            for l in 0..self.lanes[k].len() {
                let vs = &self.lanes[k][l];
                let a: Vec<f64> = (0..vs.len())
                    .map(|i| {
                        let lead = self.leader(k, l, vs[i].s, vs.get(i + 1), vs[i].next);
                        self.accel(k, vs[i].v, lead)
                    })
                    .collect();
                per_lane.push(a);
            }
            acc.push(per_lane);
        }
        // 3. Ballistic update, never backwards, never through the leader.
        let mut clamps = 0;
        for (edge_lanes, edge_acc) in self.lanes.iter_mut().zip(&acc) {
            for (vs, lane_acc) in edge_lanes.iter_mut().zip(edge_acc) {
                for i in (0..vs.len()).rev() {
                    let a = lane_acc[i];
                    let limit = vs.get(i + 1).map(|x| x.s - len - 0.1);
                    let me = &mut vs[i];
                    let v_new = (me.v + a * dt).max(0.0);
                    let ds = if v_new == 0.0 && me.v + a * dt < 0.0 {
                        // Stops within the step: travel to the stop, not past it.
                        if a < 0.0 { -me.v * me.v / (2.0 * a) } else { 0.0 }
                    } else {
                        (me.v + v_new) * 0.5 * dt
                    };
                    let mut s_new = me.s + ds.max(0.0);
                    if let Some(lim) = limit
                        && s_new > lim
                    {
                        s_new = lim.max(me.s);
                        clamps += 1;
                    }
                    me.s = s_new;
                    me.v = v_new;
                    me.cooldown = (me.cooldown - dt).max(0.0);
                }
            }
        }
        self.clamps += clamps;
        // 4. Vehicles past the end of their edge move on.
        let mut arrivals: Vec<(u32, u32, usize, Vehicle)> = Vec::new();
        for k in 0..self.lanes.len() {
            let edge_len = self.edges[k].length_m;
            for l in 0..self.lanes[k].len() {
                while self.lanes[k][l].last().is_some_and(|v| v.s >= edge_len) {
                    let mut veh = self.lanes[k][l].pop().expect("checked");
                    match veh.next {
                        Next::Exit => self.exited.add(1.0),
                        Next::Out(o) => self.outgoing[o as usize].buffer += 1.0,
                        Next::Edge(n) => {
                            veh.s -= edge_len;
                            arrivals.push((k as u32, n, l, veh));
                        }
                    }
                }
            }
        }
        for (from, n, l, mut veh) in arrivals {
            let nl = l.min(self.lanes[n as usize].len() - 1);
            let room = {
                let vs = &self.lanes[n as usize][nl];
                let at = vs.partition_point(|x| x.s < veh.s);
                // Room if the vehicle ahead on that lane is clear of the entry by a car length.
                vs.get(at).is_none_or(|a| a.s - len - 0.1 >= 0.0)
            };
            if !room {
                // The merge is occupied right at its entry: wait, stopped, at the end of the edge.
                let back = &mut self.lanes[from as usize][l];
                let edge_len = self.edges[from as usize].length_m;
                veh.s = edge_len - 0.01;
                veh.v = 0.0;
                let at = back.partition_point(|x| x.s < veh.s);
                back.insert(at, veh);
                self.holds += 1;
                continue;
            }
            veh.next = self.choose_after(n);
            let vs = &mut self.lanes[n as usize][nl];
            let at = vs.partition_point(|x| x.s < veh.s);
            // Keep behind whatever is already ahead in that lane.
            if let Some(a) = vs.get(at)
                && veh.s > a.s - len - 0.1
            {
                veh.s = (a.s - len - 0.1).max(0.0);
                self.clamps += 1;
            }
            vs.insert(at, veh);
        }
        // 5. Spawns waiting for room.
        for i in 0..self.incoming.len() {
            while let Some(&Next::Edge(e)) = self.incoming[i].queue.first() {
                let Some((lane, v)) = self.entry_room(e) else { break };
                self.incoming[i].queue.remove(0);
                self.place(e, lane, v);
            }
            // A queued vehicle bound straight for an outgoing link or a sink skips the near field.
            while let Some(&first) = self.incoming[i].queue.first() {
                match first {
                    Next::Out(o) => self.outgoing[o as usize].buffer += 1.0,
                    Next::Exit => self.exited.add(1.0),
                    Next::Edge(_) => break,
                }
                self.incoming[i].queue.remove(0);
            }
        }
        for i in 0..self.near_sources.len() {
            self.near_sources[i].pending += self.near_sources[i].rate * dt;
            self.entered.add(self.near_sources[i].rate * dt);
            while self.near_sources[i].pending >= 1.0 {
                self.near_sources[i].pending -= 1.0;
                let node = self.near_sources[i].node;
                let first = self.choose_from_source(node);
                self.near_sources[i].queue.push(first);
            }
            while let Some(&first) = self.near_sources[i].queue.first() {
                match first {
                    Next::Edge(e) => {
                        let Some((lane, v)) = self.entry_room(e) else { break };
                        self.place(e, lane, v);
                    }
                    Next::Out(o) => self.outgoing[o as usize].buffer += 1.0,
                    Next::Exit => self.exited.add(1.0),
                }
                self.near_sources[i].queue.remove(0);
            }
        }
    }

    /// Advances one CTM step (`dt_s`), with `substeps` near-field substeps after it.
    pub fn step(&mut self) {
        let dt = self.cfg.dt_s;
        // Boundary rates: outgoing links take from their buffers; incoming links close their
        // exit while a vehicle is still waiting for room in the near field.
        for o in &self.outgoing {
            self.far.set_rate(o.source_node, o.buffer / dt);
        }
        for i in &self.incoming {
            self.far.set_rate(i.sink_node, if i.queue.is_empty() { f64::INFINITY } else { 0.0 });
        }
        let flows = self.far.step();
        for &(_, _, link) in &self.far_sources {
            self.entered.add(flows[link].entered);
        }
        for &(_, link) in &self.far_sinks {
            self.exited.add(flows[link].exited);
        }
        for o in &mut self.outgoing {
            o.buffer = (o.buffer - flows[o.link].entered).max(0.0);
        }
        for i in 0..self.incoming.len() {
            self.incoming[i].pending += flows[self.incoming[i].link].exited;
            while self.incoming[i].pending >= 1.0 {
                self.incoming[i].pending -= 1.0;
                let first = self.choose_after(self.incoming[i].edge);
                self.incoming[i].queue.push(first);
            }
        }
        let sub = dt / f64::from(self.cfg.substeps);
        for i in 0..self.cfg.substeps {
            self.substep(sub, i == 0);
        }
    }

    /// Smallest bumper-to-bumper gap between consecutive vehicles in any near lane, metres
    /// (`INFINITY` with fewer than two vehicles in every lane).
    pub fn min_gap(&self) -> f64 {
        let len = self.cfg.vehicle_length_m;
        self.lanes
            .iter()
            .flatten()
            .flat_map(|vs| vs.windows(2).map(move |w| w[1].s - len - w[0].s))
            .fold(f64::INFINITY, f64::min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traffic::ctm::FundamentalDiagram;

    fn edge(from: usize, to: usize, length_m: f64, lanes: u32) -> GraphEdge {
        GraphEdge { from, to, length_m, lanes, diagram: FundamentalDiagram::URBAN, reverse: None }
    }

    /// Straight road 0 → 1 → 2 → 3 → 4 → 5; nodes 2 and 3 are near, so edge 2→3 is the near
    /// field, 1→2 enters it and 3→4 leaves it.
    fn corridor(lanes: u32) -> (Vec<GraphEdge>, Vec<bool>) {
        let edges = vec![
            edge(0, 1, 800.0, lanes),
            edge(1, 2, 800.0, lanes),
            edge(2, 3, 1200.0, lanes),
            edge(3, 4, 800.0, lanes),
            edge(4, 5, 800.0, lanes),
        ];
        (edges, vec![false, false, true, true, false, false])
    }

    #[test]
    fn corridor_hands_off_without_loss() {
        let (edges, near) = corridor(2);
        let mut h = Hybrid::new(6, &edges, &near, &|_, _| f64::NAN, HybridConfig::default()).unwrap();
        assert!(h.is_near_edge(2) && !h.is_near_edge(1) && !h.is_near_edge(3));
        assert_eq!(h.source_nodes(), vec![0]);
        h.set_source_rate(0, 0.25);
        let mut max_near = 0;
        for step in 0..1800 {
            h.step();
            let t = h.totals();
            assert!(t.conservation_error().abs() < 1e-9, "step {step}: {t:?}");
            assert!(h.min_gap() >= 0.0, "overlap at step {step}");
            max_near = max_near.max(h.near_count());
        }
        let t = h.totals();
        assert!((t.entered - 0.25 * 1800.0).abs() < 1e-6, "{t:?}");
        assert!(t.exited > 0.8 * t.entered, "{t:?}");
        assert!(max_near > 5, "the near field carried {max_near} vehicles at most");
        assert_eq!(h.clamps, 0);
        // Free-flowing near-field vehicles drive close to the edge's free-flow speed.
        let v0 = FundamentalDiagram::URBAN.free_flow_speed;
        let speeds: Vec<f64> = h.vehicles().map(|(_, _, v)| v.v).collect();
        let mean = speeds.iter().sum::<f64>() / speeds.len() as f64;
        assert!(mean > 0.8 * v0 && speeds.iter().all(|&v| v <= v0 + 1e-9), "mean {mean}");
    }

    #[test]
    fn blocked_exit_queues_at_jam_spacing_and_spills_back() {
        // Close the far-field exit: the near field fills and the queue reaches back upstream.
        let (edges, near) = corridor(1);
        let mut h = Hybrid::new(6, &edges, &near, &|_, _| f64::NAN, HybridConfig::default()).unwrap();
        h.set_source_rate(0, 0.3);
        let sink = h
            .far
            .nodes()
            .iter()
            .position(|n| matches!(n, ctm::Node::Sink { link, .. } if *link == h.link_of(4).unwrap()))
            .unwrap();
        h.far.set_rate(sink, 0.0);
        for _ in 0..2400 {
            h.step();
            assert!(h.totals().conservation_error().abs() < 1e-9);
            assert!(h.min_gap() >= 0.0);
        }
        // The near edge holds a standing queue at about the jam gap (s0 = 2 m).
        let gaps: Vec<f64> = h.lanes[2][0].windows(2).map(|w| w[1].s - 4.5 - w[0].s).collect();
        assert!(gaps.len() > 100, "{} vehicles queued", gaps.len() + 1);
        let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
        assert!((1.9..3.0).contains(&mean), "mean jam gap {mean}");
        assert!(h.vehicles().all(|(_, _, v)| v.v < 0.5));
        // Spillback: the incoming far-field link is congested too.
        let incoming = &h.far.links()[h.link_of(1).unwrap()];
        assert!(incoming.total_vehicles() > 0.5 * incoming.jam_capacity() * incoming.cell_count() as f64);
        assert_eq!(h.totals().exited, 0.0);
    }

    #[test]
    fn turning_shares_follow_the_weights() {
        // Near node 1 splits edge 0→1 into 1→2 (weight 3) and 1→3 (weight 1); both lead to sinks.
        let edges = vec![edge(0, 1, 600.0, 1), edge(1, 2, 300.0, 1), edge(1, 3, 300.0, 1)];
        let near = vec![false, true, true, true];
        let w = |i: usize, o: usize| match (i, o) {
            (0, 1) => 3.0,
            (0, 2) => 1.0,
            _ => f64::NAN,
        };
        let mut h = Hybrid::new(4, &edges, &near, &w, HybridConfig::default()).unwrap();
        h.set_source_rate(0, 0.3);
        let mut seen = std::collections::HashSet::new();
        let mut counts = [0usize; 3];
        for _ in 0..3600 {
            h.step();
            for (k, _, v) in h.vehicles() {
                if seen.insert(v.id) {
                    counts[k] += 1;
                }
            }
            assert!(h.totals().conservation_error().abs() < 1e-9);
        }
        let share = counts[1] as f64 / (counts[1] + counts[2]) as f64;
        assert!((share - 0.75).abs() < 0.05, "{counts:?}");
    }

    #[test]
    fn lane_changes_spread_a_packed_lane_safely() {
        // A two-lane near ring with ten vehicles packed into lane 0 at 10 m spacing and lane 1
        // empty: the followers brake hard behind their leaders, so moving over pays (MOBIL),
        // and every change must keep a safe gap.
        let edges = vec![edge(0, 1, 2000.0, 2), edge(1, 0, 2000.0, 2)];
        let near = vec![true, true];
        let mut h = Hybrid::new(2, &edges, &near, &|_, _| f64::NAN, HybridConfig::default()).unwrap();
        for k in 0..10 {
            h.insert_vehicle(0, 0, 100.0 + 10.0 * f64::from(k), 10.0);
        }
        let in_lane_1 = |h: &Hybrid| h.vehicles().filter(|(_, l, _)| *l == 1).count();
        let mut most = 0;
        for _ in 0..120 {
            h.step();
            assert!(h.min_gap() >= 0.0);
            assert!(h.totals().conservation_error().abs() < 1e-9);
            most = most.max(in_lane_1(&h));
        }
        assert!(most >= 3, "only {most} vehicle(s) moved to the free lane");
        assert_eq!(h.near_count(), 10);
        assert_eq!(h.clamps, 0);
    }

    #[test]
    fn dense_multi_lane_ring_never_overlaps() {
        // 240 vehicles on a 3-lane, 4 km ring with lane changes: no position is ever clamped.
        let edges: Vec<GraphEdge> = (0..4).map(|k| edge(k, (k + 1) % 4, 1000.0, 3)).collect();
        let mut h = Hybrid::new(4, &edges, &[true; 4], &|_, _| f64::NAN, HybridConfig::default()).unwrap();
        for i in 0..240 {
            let pos = f64::from(i) * 4000.0 / 240.0 * 3.0 % 4000.0;
            h.insert_vehicle((pos / 1000.0) as usize, (i % 3) as usize, pos % 1000.0, 6.0);
        }
        for _ in 0..600 {
            h.step();
            assert!(h.min_gap() >= 0.0);
        }
        assert_eq!((h.clamps, h.near_count()), (0, 240));
        assert!(h.totals().conservation_error().abs() < 1e-9);
    }

    #[test]
    fn runs_are_deterministic() {
        let run = |seed| {
            let edges = vec![edge(0, 1, 600.0, 2), edge(1, 2, 300.0, 1), edge(1, 3, 300.0, 2)];
            let near = vec![false, true, true, true];
            let mut h =
                Hybrid::new(4, &edges, &near, &|_, _| f64::NAN, HybridConfig { seed, ..HybridConfig::default() })
                    .unwrap();
            h.set_source_rate(0, 0.4);
            for _ in 0..900 {
                h.step();
            }
            h.vehicles().map(|(k, l, v)| (k, l, v.id, v.s.to_bits())).collect::<Vec<_>>()
        };
        assert_eq!(run(7), run(7));
        assert_ne!(run(7), run(8));
    }

    #[test]
    fn rejects_bad_input() {
        let (edges, near) = corridor(1);
        assert_eq!(
            Hybrid::new(6, &edges, &near[..3], &|_, _| f64::NAN, HybridConfig::default()).err(),
            Some(HybridError::MaskLength)
        );
        let bad = HybridConfig { substeps: 0, ..HybridConfig::default() };
        assert_eq!(Hybrid::new(6, &edges, &near, &|_, _| f64::NAN, bad).err(), Some(HybridError::BadConfig));
    }
}
