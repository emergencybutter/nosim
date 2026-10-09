//! Far-field traffic: the 1D Cell Transmission Model and its handoff to the near field
//! (spec §7). Beyond the microscopic radius, roads carry aggregated vehicle counts per cell
//! under the fluid conservation law; density and speed per cell are what the far-field
//! renderer draws, and the flux across the near-field boundary is what it spawns.
//!
//! The scheme is Daganzo's CTM generalised to cells longer than `v_f·Δt` (a Godunov
//! finite-volume update on a triangular fundamental diagram). Choosing the cell length at
//! least `v_f·Δt` keeps the CFL condition satisfied, so the update is unconditionally
//! stable and never produces negative or over-jam counts.

/// Triangular fundamental diagram, per lane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FundamentalDiagram {
    /// Free-flow speed, m/s.
    pub free_flow_speed: f64,
    /// Capacity (maximum flow), vehicles per second per lane.
    pub capacity_per_lane: f64,
    /// Jam density, vehicles per metre per lane.
    pub jam_density_per_lane: f64,
}

impl FundamentalDiagram {
    /// Motorway: 108 km/h, 1,800 veh/h/lane, one vehicle per 7.5 m when jammed.
    pub const MOTORWAY: FundamentalDiagram =
        FundamentalDiagram { free_flow_speed: 30.0, capacity_per_lane: 0.5, jam_density_per_lane: 1.0 / 7.5 };
    /// Urban arterial: 50 km/h, 1,200 veh/h/lane, one vehicle per 7 m when jammed.
    pub const URBAN: FundamentalDiagram =
        FundamentalDiagram { free_flow_speed: 13.9, capacity_per_lane: 1.0 / 3.0, jam_density_per_lane: 1.0 / 7.0 };

    /// Density at which flow reaches capacity, veh/m/lane.
    pub fn critical_density(&self) -> f64 {
        self.capacity_per_lane / self.free_flow_speed
    }

    /// Backward (congested) wave speed, m/s, positive.
    pub fn wave_speed(&self) -> f64 {
        self.capacity_per_lane / (self.jam_density_per_lane - self.critical_density())
    }

    /// Equilibrium flow at a per-lane density, veh/s/lane.
    pub fn flow_at_density(&self, density_per_lane: f64) -> f64 {
        let rho = density_per_lane.clamp(0.0, self.jam_density_per_lane);
        (self.free_flow_speed * rho)
            .min(self.wave_speed() * (self.jam_density_per_lane - rho))
            .min(self.capacity_per_lane)
    }

    /// Equilibrium speed at a per-lane density, m/s (free-flow speed when empty).
    pub fn speed_at_density(&self, density_per_lane: f64) -> f64 {
        if density_per_lane <= 1e-12 {
            return self.free_flow_speed;
        }
        (self.flow_at_density(density_per_lane) / density_per_lane).min(self.free_flow_speed)
    }

    fn is_valid(&self) -> bool {
        self.free_flow_speed > 0.0
            && self.capacity_per_lane > 0.0
            && self.jam_density_per_lane > self.critical_density()
            && self.free_flow_speed.is_finite()
            && self.capacity_per_lane.is_finite()
            && self.jam_density_per_lane.is_finite()
    }
}

/// Vehicles that crossed a link's boundaries during one step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepFlows {
    /// Vehicles that entered at the upstream end.
    pub entered: f64,
    /// Vehicles that left at the downstream end.
    pub exited: f64,
}

/// One directed road segment discretised into cells.
#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    diagram: FundamentalDiagram,
    lanes: u32,
    length_m: f64,
    dt_s: f64,
    cell_length_m: f64,
    /// Vehicles in each cell (all lanes).
    cells: Vec<f64>,
    /// Vehicles that crossed each cell face during the last step: `faces[i]` is the flow
    /// into cell `i`; `faces[cells.len()]` is the outflow.
    faces: Vec<f64>,
}

impl Link {
    /// Builds a link. Cells are as close to `v_f·Δt` as the length allows while staying at
    /// least that long (CFL). Returns `None` for non-positive length / time step or an
    /// inconsistent diagram.
    pub fn new(diagram: FundamentalDiagram, lanes: u32, length_m: f64, dt_s: f64) -> Option<Link> {
        if !(length_m > 0.0 && dt_s > 0.0 && lanes > 0 && diagram.is_valid()) {
            return None;
        }
        let min_cell = diagram.free_flow_speed * dt_s;
        let count = ((length_m / min_cell).floor() as usize).max(1);
        let cell_length_m = length_m / count as f64;
        Some(Link {
            diagram,
            lanes,
            length_m,
            dt_s,
            cell_length_m,
            cells: vec![0.0; count],
            faces: vec![0.0; count + 1],
        })
    }

    /// The diagram in use.
    pub fn diagram(&self) -> &FundamentalDiagram {
        &self.diagram
    }
    /// Lane count.
    pub fn lanes(&self) -> u32 {
        self.lanes
    }
    /// Link length, metres.
    pub fn length_m(&self) -> f64 {
        self.length_m
    }
    /// Time step, seconds.
    pub fn dt_s(&self) -> f64 {
        self.dt_s
    }
    /// Cell length, metres.
    pub fn cell_length_m(&self) -> f64 {
        self.cell_length_m
    }
    /// Number of cells.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }
    /// Vehicles per cell, all lanes.
    pub fn vehicles(&self) -> &[f64] {
        &self.cells
    }
    /// Vehicles on the whole link.
    pub fn total_vehicles(&self) -> f64 {
        self.cells.iter().sum()
    }
    /// Index of the cell containing a station (metres from the upstream end), clamped.
    pub fn cell_at(&self, station_m: f64) -> usize {
        ((station_m / self.cell_length_m).floor().max(0.0) as usize).min(self.cells.len() - 1)
    }
    /// Vehicles per metre in a cell, all lanes.
    pub fn density(&self, cell: usize) -> f64 {
        self.cells[cell] / self.cell_length_m
    }
    /// Equilibrium speed in a cell, m/s.
    pub fn speed(&self, cell: usize) -> f64 {
        self.diagram.speed_at_density(self.density(cell) / f64::from(self.lanes))
    }
    /// Flow through a cell face during the last step, vehicles per second. Face `0` is
    /// the upstream boundary; face `cell_count()` the downstream one.
    pub fn flux(&self, face: usize) -> f64 {
        self.faces[face] / self.dt_s
    }

    /// Maximum vehicles a cell holds.
    pub fn jam_capacity(&self) -> f64 {
        self.diagram.jam_density_per_lane * self.cell_length_m * f64::from(self.lanes)
    }

    /// Maximum vehicles that can cross a face in one step.
    fn capacity_per_step(&self) -> f64 {
        self.diagram.capacity_per_lane * f64::from(self.lanes) * self.dt_s
    }

    /// Vehicles cell `i` can send downstream this step.
    pub fn sending(&self, cell: usize) -> f64 {
        let advance = self.diagram.free_flow_speed * self.dt_s / self.cell_length_m;
        (self.cells[cell] * advance).min(self.capacity_per_step())
    }

    /// Vehicles cell `i` can receive this step.
    pub fn receiving(&self, cell: usize) -> f64 {
        let retreat = self.diagram.wave_speed() * self.dt_s / self.cell_length_m;
        (retreat * (self.jam_capacity() - self.cells[cell])).max(0.0).min(self.capacity_per_step())
    }

    /// Adds vehicles to a cell (e.g. agents leaving the near field); returns the overflow
    /// that did not fit under the jam capacity.
    pub fn inject(&mut self, cell: usize, vehicles: f64) -> f64 {
        let room = (self.jam_capacity() - self.cells[cell]).max(0.0);
        let added = vehicles.max(0.0).min(room);
        self.cells[cell] += added;
        vehicles.max(0.0) - added
    }

    /// Removes up to `vehicles` from a cell; returns how many were removed.
    pub fn remove(&mut self, cell: usize, vehicles: f64) -> f64 {
        let taken = vehicles.max(0.0).min(self.cells[cell]);
        self.cells[cell] -= taken;
        taken
    }

    /// Advances one step with boundary conditions given as rates: upstream demand and
    /// downstream supply in vehicles per second (`f64::INFINITY` for an unrestricted exit).
    pub fn step(&mut self, demand_veh_per_s: f64, supply_veh_per_s: f64) -> StepFlows {
        let inflow = (demand_veh_per_s.max(0.0) * self.dt_s).min(self.receiving(0));
        let outflow = self.sending(self.cells.len() - 1).min(supply_veh_per_s.max(0.0) * self.dt_s);
        self.step_with_boundary_flows(inflow, outflow)
    }

    /// Advances one step with the boundary flows already decided (vehicles this step), as a
    /// [`Network`] does after resolving its nodes. Flows are clamped to what the end cells
    /// can actually receive and send.
    pub fn step_with_boundary_flows(&mut self, inflow_veh: f64, outflow_veh: f64) -> StepFlows {
        let n = self.cells.len();
        self.faces[0] = inflow_veh.max(0.0).min(self.receiving(0));
        for i in 1..n {
            self.faces[i] = self.sending(i - 1).min(self.receiving(i));
        }
        self.faces[n] = outflow_veh.max(0.0).min(self.sending(n - 1));
        for i in 0..n {
            self.cells[i] = (self.cells[i] + self.faces[i] - self.faces[i + 1]).max(0.0);
        }
        StepFlows { entered: self.faces[0], exited: self.faces[n] }
    }
}

/// How links join at a node.
#[derive(Clone, Debug, PartialEq)]
pub enum Node {
    /// External demand feeding a link's upstream end, veh/s.
    Source {
        /// Fed link.
        link: usize,
        /// Demand, veh/s.
        demand_veh_per_s: f64,
    },
    /// External supply draining a link's downstream end, veh/s (`INFINITY` = free exit).
    Sink {
        /// Drained link.
        link: usize,
        /// Supply, veh/s.
        supply_veh_per_s: f64,
    },
    /// Several links feeding one, sharing its capacity by priority (Daganzo's merge).
    Merge {
        /// `(link, priority)`; priorities are normalised.
        from: Vec<(usize, f64)>,
        /// Receiving link.
        to: usize,
    },
    /// One link splitting by fixed turning fractions, FIFO: a blocked branch holds all
    /// (Daganzo's diverge).
    Diverge {
        /// Sending link.
        from: usize,
        /// `(link, fraction)`; fractions are normalised.
        to: Vec<(usize, f64)>,
    },
}

/// Links joined by nodes, stepped together.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Network {
    links: Vec<Link>,
    nodes: Vec<Node>,
}

fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

/// Why a node could not be added.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetworkError {
    /// A node names a link index that does not exist.
    UnknownLink(usize),
    /// A link end already has a node attached.
    EndAlreadyConnected(usize),
    /// A merge or diverge has no branches, or weights are not positive.
    BadWeights,
}

impl Network {
    /// Empty network.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a link; returns its index.
    pub fn add_link(&mut self, link: Link) -> usize {
        self.links.push(link);
        self.links.len() - 1
    }

    /// Adds a node after checking every referenced link exists and no end is doubly fed.
    pub fn add_node(&mut self, node: Node) -> Result<usize, NetworkError> {
        let (ups, downs): (Vec<usize>, Vec<usize>) = match &node {
            Node::Source { link, .. } => (vec![*link], vec![]),
            Node::Sink { link, .. } => (vec![], vec![*link]),
            Node::Merge { from, to } => {
                if from.is_empty() || !from.iter().all(|(_, p)| positive(*p)) {
                    return Err(NetworkError::BadWeights);
                }
                (vec![*to], from.iter().map(|(l, _)| *l).collect())
            }
            Node::Diverge { from, to } => {
                if to.is_empty() || !to.iter().all(|(_, f)| positive(*f)) {
                    return Err(NetworkError::BadWeights);
                }
                (to.iter().map(|(l, _)| *l).collect(), vec![*from])
            }
        };
        for &l in ups.iter().chain(&downs) {
            if l >= self.links.len() {
                return Err(NetworkError::UnknownLink(l));
            }
        }
        for &l in &ups {
            if self.upstream_node(l).is_some() {
                return Err(NetworkError::EndAlreadyConnected(l));
            }
        }
        for &l in &downs {
            if self.downstream_node(l).is_some() {
                return Err(NetworkError::EndAlreadyConnected(l));
            }
        }
        self.nodes.push(node);
        Ok(self.nodes.len() - 1)
    }

    fn upstream_node(&self, link: usize) -> Option<usize> {
        self.nodes.iter().position(|n| match n {
            Node::Source { link: l, .. } => *l == link,
            Node::Merge { to, .. } => *to == link,
            Node::Diverge { to, .. } => to.iter().any(|(l, _)| *l == link),
            Node::Sink { .. } => false,
        })
    }

    fn downstream_node(&self, link: usize) -> Option<usize> {
        self.nodes.iter().position(|n| match n {
            Node::Sink { link: l, .. } => *l == link,
            Node::Merge { from, .. } => from.iter().any(|(l, _)| *l == link),
            Node::Diverge { from, .. } => *from == link,
            Node::Source { .. } => false,
        })
    }

    /// Links in index order.
    pub fn links(&self) -> &[Link] {
        &self.links
    }
    /// Mutable link access (for injections and removals).
    pub fn link_mut(&mut self, index: usize) -> &mut Link {
        &mut self.links[index]
    }
    /// Nodes in index order.
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }
    /// Replace a node's external rate (source demand or sink supply); other kinds ignore it.
    pub fn set_rate(&mut self, node: usize, veh_per_s: f64) {
        match &mut self.nodes[node] {
            Node::Source { demand_veh_per_s, .. } => *demand_veh_per_s = veh_per_s,
            Node::Sink { supply_veh_per_s, .. } => *supply_veh_per_s = veh_per_s,
            _ => {}
        }
    }
    /// Vehicles on every link.
    pub fn total_vehicles(&self) -> f64 {
        self.links.iter().map(Link::total_vehicles).sum()
    }

    /// Advances every link one step; returns per-link boundary flows. Link ends without a
    /// node are closed.
    pub fn step(&mut self) -> Vec<StepFlows> {
        let n = self.links.len();
        let mut inflow = vec![0.0; n];
        let mut outflow = vec![0.0; n];
        let send: Vec<f64> = self.links.iter().map(|l| l.sending(l.cell_count() - 1)).collect();
        let recv: Vec<f64> = self.links.iter().map(|l| l.receiving(0)).collect();

        for node in &self.nodes {
            match node {
                Node::Source { link, demand_veh_per_s } => {
                    inflow[*link] = (demand_veh_per_s.max(0.0) * self.links[*link].dt_s).min(recv[*link]);
                }
                Node::Sink { link, supply_veh_per_s } => {
                    outflow[*link] = (supply_veh_per_s.max(0.0) * self.links[*link].dt_s).min(send[*link]);
                }
                Node::Merge { from, to } => {
                    let flows = merge_flows(&from.iter().map(|(l, p)| (send[*l], *p)).collect::<Vec<_>>(), recv[*to]);
                    for ((l, _), f) in from.iter().zip(flows) {
                        outflow[*l] = f;
                    }
                    inflow[*to] =
                        outflow.iter().zip(0..).filter(|(_, i)| from.iter().any(|(l, _)| l == i)).map(|(f, _)| f).sum();
                }
                Node::Diverge { from, to } => {
                    let total: f64 = to.iter().map(|(_, f)| f).sum();
                    let mut y = send[*from];
                    for (l, f) in to {
                        let beta = f / total;
                        y = y.min(recv[*l] / beta);
                    }
                    outflow[*from] = y;
                    for (l, f) in to {
                        inflow[*l] = y * f / total;
                    }
                }
            }
        }
        (0..n).map(|i| self.links[i].step_with_boundary_flows(inflow[i], outflow[i])).collect()
    }
}

/// Daganzo's priority merge for any number of inputs: each input gets its sending flow if
/// everything fits; otherwise capacity is shared by priority, with shares that inputs do not
/// use handed to the ones still constrained.
pub fn merge_flows(inputs: &[(f64, f64)], receiving: f64) -> Vec<f64> {
    let total_send: f64 = inputs.iter().map(|(s, _)| s).sum();
    if total_send <= receiving {
        return inputs.iter().map(|(s, _)| *s).collect();
    }
    let mut flows = vec![0.0; inputs.len()];
    let mut remaining = receiving;
    let mut open: Vec<usize> = (0..inputs.len()).collect();
    while !open.is_empty() && remaining > 1e-12 {
        let weight: f64 = open.iter().map(|&i| inputs[i].1).sum();
        let mut next_open = Vec::new();
        let mut used = 0.0;
        for &i in &open {
            let share = remaining * inputs[i].1 / weight;
            let want = inputs[i].0 - flows[i];
            if want <= share {
                flows[i] += want;
                used += want;
            } else {
                flows[i] += share;
                used += share;
                next_open.push(i);
            }
        }
        remaining -= used;
        if next_open.len() == open.len() {
            break; // everyone is constrained: shares were exact
        }
        open = next_open;
    }
    flows
}

/// Turns the fractional flux crossing into the near field into whole vehicles to spawn,
/// preserving the rate exactly on average.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NearFieldBoundary {
    /// Fractional vehicle carried to the next step.
    pub pending: f64,
}

/// A request to instantiate microscopic agents at the boundary.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpawnBatch {
    /// Vehicles to create this step.
    pub count: u32,
    /// Speed to give them, m/s (the boundary cell's equilibrium speed).
    pub speed_m_s: f64,
    /// Spacing between them, metres (from the boundary cell's density; `INFINITY` when empty).
    pub spacing_m: f64,
}

impl NearFieldBoundary {
    /// Fresh accumulator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Converts this step's exit flow from `link` (its downstream end faces the near field)
    /// into a spawn batch.
    pub fn take_spawns(&mut self, link: &Link, exited_vehicles: f64) -> SpawnBatch {
        self.pending += exited_vehicles.max(0.0);
        let count = self.pending.floor();
        self.pending -= count;
        let last = link.cell_count() - 1;
        let density_per_lane = link.density(last) / f64::from(link.lanes());
        SpawnBatch {
            count: count as u32,
            speed_m_s: link.speed(last),
            spacing_m: if density_per_lane > 1e-12 { 1.0 / density_per_lane } else { f64::INFINITY },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn fundamental_diagram_shape() {
        let d = FundamentalDiagram::MOTORWAY;
        assert!(near(d.critical_density(), 0.5 / 30.0, 1e-12));
        let w = d.wave_speed();
        assert!(w > 3.0 && w < 7.0, "{w}"); // 11–25 km/h backward wave
        assert_eq!(d.speed_at_density(0.0), 30.0);
        assert!(near(d.speed_at_density(d.critical_density()), 30.0, 1e-9));
        assert!(near(d.flow_at_density(d.critical_density()), 0.5, 1e-12));
        assert!(near(d.flow_at_density(d.jam_density_per_lane), 0.0, 1e-12));
        assert!(near(d.speed_at_density(d.jam_density_per_lane), 0.0, 1e-12));
        assert!(d.speed_at_density(0.1) < 10.0 && d.speed_at_density(0.1) > 0.0);
        assert!(Link::new(FundamentalDiagram { free_flow_speed: 0.0, ..d }, 1, 100.0, 1.0).is_none());
        assert!(Link::new(d, 0, 100.0, 1.0).is_none());
        assert!(Link::new(d, 1, -1.0, 1.0).is_none());
    }

    #[test]
    fn cells_satisfy_cfl() {
        let d = FundamentalDiagram::MOTORWAY;
        let l = Link::new(d, 2, 1000.0, 1.0).unwrap();
        assert_eq!(l.cell_count(), 33);
        assert!(l.cell_length_m() >= d.free_flow_speed * l.dt_s());
        assert!(near(l.cell_length_m() * l.cell_count() as f64, 1000.0, 1e-9));
        assert_eq!(l.cell_at(0.0), 0);
        assert_eq!(l.cell_at(999.9), 32);
        assert_eq!(l.cell_at(5000.0), 32);
        let short = Link::new(d, 1, 10.0, 1.0).unwrap(); // shorter than v_f·Δt: one cell
        assert_eq!(short.cell_count(), 1);
    }

    #[test]
    fn conservation_and_bounds() {
        let mut l = Link::new(FundamentalDiagram::MOTORWAY, 1, 3000.0, 1.0).unwrap();
        for c in 0..l.cell_count() {
            l.inject(c, 1.5);
        }
        let start = l.total_vehicles();
        for _ in 0..500 {
            l.step(0.0, 0.0); // closed both ends
            assert!(near(l.total_vehicles(), start, 1e-9));
            assert!(l.vehicles().iter().all(|&n| n >= 0.0 && n <= l.jam_capacity() + 1e-9));
        }
        // Overflowing injection is reported, not silently created.
        let overflow = l.inject(0, 1e6);
        assert!(overflow > 0.0 && near(l.vehicles()[0], l.jam_capacity(), 1e-9));
        assert!(near(l.remove(0, 1e6), l.jam_capacity(), 1e-9));
        assert_eq!(l.vehicles()[0], 0.0);
    }

    /// With cells exactly v_f·Δt, free-flow transport is exact: a platoon moves one cell per step.
    #[test]
    fn free_flow_moves_platoon_at_free_flow_speed() {
        let mut l = Link::new(FundamentalDiagram::MOTORWAY, 1, 3000.0, 1.0).unwrap(); // cells = 30 m = v_f·Δt
        assert!(near(l.cell_length_m(), 30.0, 1e-9));
        l.inject(0, 0.4); // below per-step capacity (0.5), so nothing is held back
        for k in 1..50 {
            l.step(0.0, f64::INFINITY);
            assert!(near(l.vehicles()[k], 0.4, 1e-12), "step {k}");
            assert!(near(l.speed(k), 30.0, 1e-9));
        }
    }

    #[test]
    fn steady_demand_reaches_equilibrium_density() {
        let d = FundamentalDiagram::MOTORWAY;
        let mut l = Link::new(d, 2, 3000.0, 1.0).unwrap();
        let q = 0.6; // veh/s on two lanes, below capacity 1.0
        for _ in 0..600 {
            l.step(q, f64::INFINITY);
        }
        let expected_density = q / d.free_flow_speed; // veh/m, all lanes
        for c in 0..l.cell_count() {
            assert!(near(l.density(c), expected_density, 1e-6), "cell {c}: {}", l.density(c));
            assert!(near(l.speed(c), 30.0, 1e-6));
        }
        for f in 0..=l.cell_count() {
            assert!(near(l.flux(f), q, 1e-6));
        }
        // Demand above capacity is capped at capacity.
        let mut over = Link::new(d, 2, 3000.0, 1.0).unwrap();
        for _ in 0..600 {
            over.step(5.0, f64::INFINITY);
        }
        assert!(near(over.flux(over.cell_count()), 1.0, 1e-6));
    }

    /// Close the exit under heavy inflow: the queue tail propagates upstream at the wave speed.
    #[test]
    fn bottleneck_shockwave_travels_at_wave_speed() {
        let d = FundamentalDiagram::MOTORWAY;
        let mut l = Link::new(d, 1, 3000.0, 1.0).unwrap();
        for _ in 0..200 {
            l.step(0.5, f64::INFINITY); // fill at capacity
        }
        let jam = l.jam_capacity();
        let mut reached: Vec<(usize, usize)> = Vec::new(); // (cell, step at which it jammed)
        for step in 1..=800 {
            l.step(0.5, 0.0);
            for c in (0..l.cell_count()).rev() {
                if l.vehicles()[c] > 0.99 * jam && !reached.iter().any(|(cc, _)| *cc == c) {
                    reached.push((c, step));
                }
            }
        }
        let last = l.cell_count() - 1;
        let (c_a, t_a) = *reached.iter().find(|(c, _)| *c == last - 10).expect("cell jammed");
        let (c_b, t_b) = *reached.iter().find(|(c, _)| *c == last - 40).expect("cell jammed");
        let measured = (c_a - c_b) as f64 * l.cell_length_m() / (t_b - t_a) as f64;
        assert!(
            near(measured, d.wave_speed(), 0.3 * d.wave_speed()),
            "measured {measured} m/s vs w = {}",
            d.wave_speed()
        );
        assert!(l.speed(last) < 1.0 && l.flux(last + 1) == 0.0);
    }

    #[test]
    fn merge_shares_capacity_by_priority() {
        // Everything fits: inputs pass unchanged.
        assert_eq!(merge_flows(&[(0.2, 1.0), (0.1, 1.0)], 1.0), vec![0.2, 0.1]);
        // Both saturated, equal priority: half each.
        let f = merge_flows(&[(0.6, 1.0), (0.6, 1.0)], 0.5);
        assert!(near(f[0], 0.25, 1e-12) && near(f[1], 0.25, 1e-12));
        // Both saturated, 70/30.
        let f = merge_flows(&[(0.6, 0.7), (0.6, 0.3)], 0.5);
        assert!(near(f[0], 0.35, 1e-12) && near(f[1], 0.15, 1e-12));
        // One input smaller than its share: the other takes the rest (Daganzo's median rule).
        let f = merge_flows(&[(0.1, 0.5), (0.6, 0.5)], 0.5);
        assert!(near(f[0], 0.1, 1e-12) && near(f[1], 0.4, 1e-12));
        assert!(near(f.iter().sum::<f64>(), 0.5, 1e-12));
    }

    #[test]
    fn network_merge_and_diverge() {
        let d = FundamentalDiagram::MOTORWAY;
        let mut net = Network::new();
        let a = net.add_link(Link::new(d, 1, 600.0, 1.0).unwrap());
        let b = net.add_link(Link::new(d, 1, 600.0, 1.0).unwrap());
        let m = net.add_link(Link::new(d, 1, 600.0, 1.0).unwrap()); // one lane: capacity 0.5
        let x = net.add_link(Link::new(d, 1, 600.0, 1.0).unwrap());
        let y = net.add_link(Link::new(d, 1, 600.0, 1.0).unwrap());
        net.add_node(Node::Source { link: a, demand_veh_per_s: 0.4 }).unwrap();
        net.add_node(Node::Source { link: b, demand_veh_per_s: 0.4 }).unwrap();
        net.add_node(Node::Merge { from: vec![(a, 0.5), (b, 0.5)], to: m }).unwrap();
        net.add_node(Node::Diverge { from: m, to: vec![(x, 0.7), (y, 0.3)] }).unwrap();
        net.add_node(Node::Sink { link: x, supply_veh_per_s: f64::INFINITY }).unwrap();
        let y_sink = net.add_node(Node::Sink { link: y, supply_veh_per_s: f64::INFINITY }).unwrap();
        // Validation.
        assert_eq!(
            net.add_node(Node::Sink { link: x, supply_veh_per_s: 1.0 }),
            Err(NetworkError::EndAlreadyConnected(x))
        );
        assert_eq!(net.add_node(Node::Source { link: 99, demand_veh_per_s: 1.0 }), Err(NetworkError::UnknownLink(99)));
        assert_eq!(net.add_node(Node::Merge { from: vec![], to: m }), Err(NetworkError::BadWeights));

        for _ in 0..400 {
            net.step();
        }
        // Merge: 0.8 demanded, 0.5 fits → 0.25 each; downstream split 0.35 / 0.15.
        let flows = net.step();
        assert!(near(flows[a].exited, 0.25, 1e-6) && near(flows[b].exited, 0.25, 1e-6));
        assert!(near(flows[m].entered, 0.5, 1e-6));
        assert!(near(flows[x].entered, 0.35, 1e-6) && near(flows[y].entered, 0.15, 1e-6));

        // Block the minor branch: FIFO holds the whole diverge once y is jammed. Jamming y
        // (80 veh at 0.15 veh/s), then m, then a and b takes about 900 s.
        net.set_rate(y_sink, 0.0);
        for _ in 0..2500 {
            net.step();
        }
        let flows = net.step();
        assert!(flows[m].exited < 1e-6, "{}", flows[m].exited);
        assert!(flows[x].entered < 1e-6);
        // Upstream links back up: sources are throttled below demand.
        assert!(flows[a].entered < 0.1 && flows[b].entered < 0.1);
        let total_before = net.total_vehicles();
        net.step();
        assert!(net.total_vehicles() <= total_before + 1e-9); // nothing enters a jammed system
    }

    #[test]
    fn near_field_handoff_preserves_rate() {
        let d = FundamentalDiagram::MOTORWAY;
        let mut l = Link::new(d, 2, 1500.0, 0.5).unwrap(); // 0.5 s steps
        let mut boundary = NearFieldBoundary::new();
        let q = 0.7;
        let mut spawned = 0u32;
        let mut exited = 0.0;
        let mut last_batch = SpawnBatch::default();
        for _ in 0..2000 {
            let flows = l.step(q, f64::INFINITY);
            exited += flows.exited;
            last_batch = boundary.take_spawns(&l, flows.exited);
            spawned += last_batch.count;
        }
        assert!(exited > 500.0);
        // The accumulator may hold just under one whole vehicle.
        assert!((f64::from(spawned) - exited).abs() < 1.0 + 1e-6, "spawned {spawned} vs exited {exited}");
        assert!(boundary.pending >= 0.0 && boundary.pending < 1.0);
        assert!(near(last_batch.speed_m_s, 30.0, 1e-6));
        // Spacing per lane at q = 0.7 veh/s over 2 lanes at 30 m/s: 30 / 0.35 ≈ 85.7 m.
        assert!(near(last_batch.spacing_m, 30.0 / 0.35, 0.5), "{}", last_batch.spacing_m);
        let empty = Link::new(d, 1, 300.0, 1.0).unwrap();
        assert_eq!(NearFieldBoundary::new().take_spawns(&empty, 0.0).spacing_m, f64::INFINITY);
    }
}
