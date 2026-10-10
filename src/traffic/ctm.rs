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

    /// Vehicles cell `i` can send downstream this step. The free-flow advance is capped at
    /// one cell per step: on a link shorter than `v_f·Δt` (CFL not met) a cell can still never
    /// send more than it holds, so conservation stays exact.
    pub fn sending(&self, cell: usize) -> f64 {
        let advance = (self.diagram.free_flow_speed * self.dt_s / self.cell_length_m).min(1.0);
        (self.cells[cell] * advance).min(self.capacity_per_step())
    }

    /// Vehicles cell `i` can receive this step; likewise capped at its free room, so a short
    /// cell is never filled past jam.
    pub fn receiving(&self, cell: usize) -> f64 {
        let retreat = (self.diagram.wave_speed() * self.dt_s / self.cell_length_m).min(1.0);
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
    /// A general intersection: any number of inputs and outputs, with a turning fraction for
    /// every input–output pair (Tampère et al. 2011). See [`junction_flows`].
    Junction {
        /// `(link, priority)`; usually the input's capacity.
        from: Vec<(usize, f64)>,
        /// Output links.
        to: Vec<usize>,
        /// `turning[i][j]`: share of input `i`'s traffic bound for output `j`. Rows are
        /// normalised; zero entries are turns that are not allowed.
        turning: Vec<Vec<f64>>,
    },
}

/// Links joined by nodes, stepped together.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Network {
    links: Vec<Link>,
    nodes: Vec<Node>,
    /// Node attached to each link's upstream end.
    upstream: Vec<Option<usize>>,
    /// Node attached to each link's downstream end.
    downstream: Vec<Option<usize>>,
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
    /// A merge, diverge or junction has no branches, or weights are not positive.
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
        self.upstream.push(None);
        self.downstream.push(None);
        self.links.len() - 1
    }

    /// Adds a node after checking every referenced link exists and no end is doubly fed.
    pub fn add_node(&mut self, node: Node) -> Result<usize, NetworkError> {
        // `ups`: links whose upstream end this node feeds; `downs`: links it drains.
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
            Node::Junction { from, to, turning } => {
                let rows_ok = turning.len() == from.len()
                    && turning.iter().all(|row| {
                        row.len() == to.len()
                            && row.iter().all(|b| b.is_finite() && *b >= 0.0)
                            && positive(row.iter().sum())
                    });
                if from.is_empty() || to.is_empty() || !from.iter().all(|(_, p)| positive(*p)) || !rows_ok {
                    return Err(NetworkError::BadWeights);
                }
                (to.clone(), from.iter().map(|(l, _)| *l).collect())
            }
        };
        for &l in ups.iter().chain(&downs) {
            if l >= self.links.len() {
                return Err(NetworkError::UnknownLink(l));
            }
        }
        let mut seen = std::collections::HashSet::new();
        for &l in &ups {
            if self.upstream[l].is_some() || !seen.insert(l) {
                return Err(NetworkError::EndAlreadyConnected(l));
            }
        }
        seen.clear();
        for &l in &downs {
            if self.downstream[l].is_some() || !seen.insert(l) {
                return Err(NetworkError::EndAlreadyConnected(l));
            }
        }
        let index = self.nodes.len();
        for &l in &ups {
            self.upstream[l] = Some(index);
        }
        for &l in &downs {
            self.downstream[l] = Some(index);
        }
        self.nodes.push(node);
        Ok(index)
    }

    /// Node attached to a link's upstream end.
    pub fn upstream_node(&self, link: usize) -> Option<usize> {
        self.upstream.get(link).copied().flatten()
    }

    /// Node attached to a link's downstream end.
    pub fn downstream_node(&self, link: usize) -> Option<usize> {
        self.downstream.get(link).copied().flatten()
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
                    let mut total = 0.0;
                    for ((l, _), f) in from.iter().zip(flows) {
                        outflow[*l] = f;
                        total += f;
                    }
                    inflow[*to] = total;
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
                Node::Junction { from, to, turning } => {
                    let sending: Vec<f64> = from.iter().map(|(l, _)| send[*l]).collect();
                    let priority: Vec<f64> = from.iter().map(|(_, p)| *p).collect();
                    let receiving: Vec<f64> = to.iter().map(|l| recv[*l]).collect();
                    let q = junction_flows(&sending, &priority, turning, &receiving);
                    for (i, (l, _)) in from.iter().enumerate() {
                        outflow[*l] = q[i].iter().sum();
                    }
                    for (j, l) in to.iter().enumerate() {
                        inflow[*l] = q.iter().map(|row| row[j]).sum();
                    }
                }
            }
        }
        (0..n).map(|i| self.links[i].step_with_boundary_flows(inflow[i], outflow[i])).collect()
    }
}

/// The general first-order node model of Tampère, Corthout, Cattrysse & Immers (2011):
/// flows `q[i][j]` from each input to each output, given input sending flows, input
/// priorities (normally capacities), row-normalised turning fractions and output receiving
/// flows.
///
/// It satisfies the requirements that model sets out: no input sends more than it has and
/// no output receives more than it can take; flows are conserved; each input's turns move
/// in fixed proportion (FIFO), so a blocked exit holds back that whole approach; an input
/// that is supply-constrained gets a share of the scarce output proportional to its
/// priority; and an input whose demand fits is never cut. With one output it is Daganzo's
/// priority merge ([`merge_flows`]); with one input it is Daganzo's diverge.
pub fn junction_flows(sending: &[f64], priority: &[f64], turning: &[Vec<f64>], receiving: &[f64]) -> Vec<Vec<f64>> {
    let (ni, nj) = (sending.len(), receiving.len());
    let beta: Vec<Vec<f64>> = turning
        .iter()
        .map(|row| {
            let total: f64 = row.iter().sum();
            row.iter().map(|b| if total > 0.0 { b / total } else { 0.0 }).collect()
        })
        .collect();
    let mut q = vec![vec![0.0; nj]; ni];
    let mut remaining: Vec<f64> = receiving.iter().map(|r| r.max(0.0)).collect();
    // Undetermined inputs competing for each output.
    let mut competing: Vec<Vec<usize>> =
        (0..nj).map(|j| (0..ni).filter(|&i| sending[i] > 0.0 && beta[i][j] > 0.0).collect()).collect();
    let mut determined = vec![false; ni];
    loop {
        // Most restrictive output: the smallest supply per unit of competing priority.
        let mut best: Option<(usize, f64)> = None;
        for (j, inputs) in competing.iter().enumerate() {
            if inputs.is_empty() {
                continue;
            }
            let weight: f64 = inputs.iter().map(|&i| priority[i] * beta[i][j]).sum();
            let a = remaining[j].max(0.0) / weight;
            if best.is_none_or(|(_, b)| a < b) {
                best = Some((j, a));
            }
        }
        let Some((jstar, a)) = best else { break };
        let demand_bound: Vec<usize> =
            competing[jstar].iter().copied().filter(|&i| sending[i] <= a * priority[i]).collect();
        // Inputs whose demand fits get all of it; otherwise every input at j* is held to its
        // priority share of j*.
        let fixed: Vec<(usize, f64)> = if demand_bound.is_empty() {
            competing[jstar].iter().map(|&i| (i, a * priority[i])).collect()
        } else {
            demand_bound.iter().map(|&i| (i, sending[i])).collect()
        };
        for (i, total) in fixed {
            determined[i] = true;
            for j in 0..nj {
                q[i][j] = total * beta[i][j];
                remaining[j] -= q[i][j];
            }
        }
        for inputs in &mut competing {
            inputs.retain(|&i| !determined[i]);
        }
    }
    q
}

/// A directed road segment for [`build_network`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraphEdge {
    /// Graph node at the upstream end.
    pub from: usize,
    /// Graph node at the downstream end.
    pub to: usize,
    /// Length, metres.
    pub length_m: f64,
    /// Lanes in this direction.
    pub lanes: u32,
    /// Fundamental diagram.
    pub diagram: FundamentalDiagram,
    /// The opposite-direction edge of the same road, if two-way (its U-turn).
    pub reverse: Option<usize>,
}

/// A network built from a graph, with the external boundary nodes listed.
#[derive(Clone, Debug, PartialEq)]
pub struct GraphNetwork {
    /// The network; link `k` is edge `k`.
    pub network: Network,
    /// `(network node, link)` for every edge leaving a graph node that nothing enters.
    pub sources: Vec<(usize, usize)>,
    /// `(network node, link)` for every edge entering a graph node that nothing leaves.
    pub sinks: Vec<(usize, usize)>,
    /// Intersections built as [`Node::Junction`].
    pub junctions: usize,
    /// Links shorter than one free-flow step (`v_f·Δt`): they hold a single cell, so traffic
    /// crosses them at most one cell per step, slower than free flow. Shorten `Δt` to remove.
    pub short_links: usize,
}

/// Why a graph could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BuildError {
    /// Edge `k` has a non-positive length, no lanes or an invalid diagram.
    BadEdge(usize),
    /// Edge `k` names a node at or beyond the node count.
    UnknownNode(usize),
    /// The network rejected a node (a bug in the graph, such as a malformed reverse link).
    Network(NetworkError),
}

/// Builds a CTM network from a directed graph. Every edge becomes a link; every graph node
/// with both entering and leaving edges becomes a [`Node::Junction`] whose turning fractions
/// split each input over its outputs in proportion to their capacity, never back onto its
/// own reverse (no U-turns) unless that is the only way on; priorities are input
/// capacities. Nodes only left become sources (demand 0, set it with
/// [`Network::set_rate`]); nodes only entered become free-exit sinks.
pub fn build_network(node_count: usize, edges: &[GraphEdge], dt_s: f64) -> Result<GraphNetwork, BuildError> {
    let mut network = Network::new();
    let mut short_links = 0;
    for (k, e) in edges.iter().enumerate() {
        if e.from >= node_count || e.to >= node_count {
            return Err(BuildError::UnknownNode(k));
        }
        let link = Link::new(e.diagram, e.lanes, e.length_m, dt_s).ok_or(BuildError::BadEdge(k))?;
        if e.length_m < e.diagram.free_flow_speed * dt_s {
            short_links += 1;
        }
        network.add_link(link);
    }
    let capacity = |e: &GraphEdge| e.diagram.capacity_per_lane * f64::from(e.lanes);
    let mut ins: Vec<Vec<usize>> = vec![Vec::new(); node_count];
    let mut outs: Vec<Vec<usize>> = vec![Vec::new(); node_count];
    for (k, e) in edges.iter().enumerate() {
        outs[e.from].push(k);
        ins[e.to].push(k);
    }
    let (mut sources, mut sinks, mut junctions) = (Vec::new(), Vec::new(), 0);
    let net_err = BuildError::Network;
    for v in 0..node_count {
        match (ins[v].is_empty(), outs[v].is_empty()) {
            (true, true) => {}
            (true, false) => {
                for &l in &outs[v] {
                    let n = network.add_node(Node::Source { link: l, demand_veh_per_s: 0.0 }).map_err(net_err)?;
                    sources.push((n, l));
                }
            }
            (false, true) => {
                for &l in &ins[v] {
                    let n =
                        network.add_node(Node::Sink { link: l, supply_veh_per_s: f64::INFINITY }).map_err(net_err)?;
                    sinks.push((n, l));
                }
            }
            (false, false) => {
                let turning: Vec<Vec<f64>> = ins[v]
                    .iter()
                    .map(|&i| {
                        let no_uturn = outs[v].iter().any(|&o| edges[i].reverse != Some(o));
                        outs[v]
                            .iter()
                            .map(|&o| if no_uturn && edges[i].reverse == Some(o) { 0.0 } else { capacity(&edges[o]) })
                            .collect()
                    })
                    .collect();
                let from = ins[v].iter().map(|&i| (i, capacity(&edges[i]))).collect();
                network.add_node(Node::Junction { from, to: outs[v].clone(), turning }).map_err(net_err)?;
                junctions += 1;
            }
        }
    }
    Ok(GraphNetwork { network, sources, sinks, junctions, short_links })
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

    /// Deterministic pseudo-random numbers in [0, 1).
    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        (*seed >> 11) as f64 / (1u64 << 53) as f64
    }

    #[test]
    fn junction_reduces_to_merge_and_diverge() {
        let mut seed = 7u64;
        for _ in 0..500 {
            // One output: Daganzo's priority merge.
            let n = 1 + (lcg(&mut seed) * 4.0) as usize;
            let inputs: Vec<(f64, f64)> = (0..n).map(|_| (lcg(&mut seed) * 5.0, 0.1 + lcg(&mut seed))).collect();
            let r = lcg(&mut seed) * 8.0;
            let sending: Vec<f64> = inputs.iter().map(|x| x.0).collect();
            let priority: Vec<f64> = inputs.iter().map(|x| x.1).collect();
            let q = junction_flows(&sending, &priority, &vec![vec![1.0]; n], &[r]);
            for (got, want) in q.iter().map(|row| row[0]).zip(merge_flows(&inputs, r)) {
                assert!((got - want).abs() < 1e-9, "merge {got} vs {want}");
            }
            // One input: Daganzo's diverge, y = min(S, min_j R_j / β_j).
            let m = 1 + (lcg(&mut seed) * 4.0) as usize;
            let beta: Vec<f64> = (0..m).map(|_| 0.05 + lcg(&mut seed)).collect();
            let total: f64 = beta.iter().sum();
            let recv: Vec<f64> = (0..m).map(|_| lcg(&mut seed) * 3.0).collect();
            let s = lcg(&mut seed) * 5.0;
            let q = junction_flows(&[s], &[1.0], std::slice::from_ref(&beta), &recv);
            let y = beta.iter().zip(&recv).fold(s, |y, (b, r)| y.min(r / (b / total)));
            assert!((q[0].iter().sum::<f64>() - y).abs() < 1e-9);
            for (j, b) in beta.iter().enumerate() {
                assert!((q[0][j] - y * b / total).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn junction_invariants_on_random_intersections() {
        let mut seed = 99u64;
        for _ in 0..2000 {
            let (ni, nj) = (1 + (lcg(&mut seed) * 4.0) as usize, 1 + (lcg(&mut seed) * 4.0) as usize);
            let sending: Vec<f64> =
                (0..ni).map(|_| if lcg(&mut seed) < 0.1 { 0.0 } else { lcg(&mut seed) * 4.0 }).collect();
            let priority: Vec<f64> = (0..ni).map(|_| 0.2 + lcg(&mut seed)).collect();
            let turning: Vec<Vec<f64>> = (0..ni)
                .map(|_| {
                    let mut row: Vec<f64> =
                        (0..nj).map(|_| if lcg(&mut seed) < 0.3 { 0.0 } else { lcg(&mut seed) }).collect();
                    if row.iter().sum::<f64>() == 0.0 {
                        row[0] = 1.0;
                    }
                    row
                })
                .collect();
            let receiving: Vec<f64> = (0..nj).map(|_| lcg(&mut seed) * 5.0).collect();
            let q = junction_flows(&sending, &priority, &turning, &receiving);
            let eps = 1e-9;
            for j in 0..nj {
                let into: f64 = q.iter().map(|row| row[j]).sum();
                assert!(into <= receiving[j] + eps, "output {j} over capacity");
            }
            for i in 0..ni {
                let total: f64 = turning[i].iter().sum();
                let out: f64 = q[i].iter().sum();
                assert!(q[i].iter().all(|&x| x >= -eps));
                assert!(out <= sending[i] + eps, "input {i} sends more than it has");
                // FIFO: every turn of an input carries its share of that input's flow.
                for j in 0..nj {
                    assert!((q[i][j] - out * turning[i][j] / total).abs() < 1e-9);
                }
                // Maximal: an input held below its demand is held by a full output it uses.
                if out < sending[i] - 1e-7 {
                    let blocked = (0..nj)
                        .any(|j| turning[i][j] > 0.0 && q.iter().map(|row| row[j]).sum::<f64>() >= receiving[j] - 1e-7);
                    assert!(blocked, "input {i} held back with no full output");
                }
            }
        }
    }

    #[test]
    fn blocked_exit_holds_the_whole_approach() {
        // Input 0 splits evenly to a wide-open exit and a nearly full one; input 1 only uses
        // the nearly full one. Exit 1 admits 6: shared by priority (equal), input 0 gets 4 in
        // total, so only 2 reach the open exit although it could take 100 (FIFO).
        let q = junction_flows(&[10.0, 10.0], &[1.0, 1.0], &[vec![1.0, 1.0], vec![0.0, 1.0]], &[100.0, 6.0]);
        let near = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(near(q[0][0], 2.0) && near(q[0][1], 2.0) && near(q[1][0], 0.0) && near(q[1][1], 4.0), "{q:?}");
        // With room everywhere, everything flows.
        let q = junction_flows(&[10.0, 10.0], &[1.0, 1.0], &[vec![1.0, 1.0], vec![0.0, 1.0]], &[100.0, 100.0]);
        assert!(near(q[0][0], 5.0) && near(q[0][1], 5.0) && near(q[1][1], 10.0));
        // A higher-priority input gets the larger share of a contested exit.
        let q = junction_flows(&[10.0, 10.0], &[3.0, 1.0], &[vec![1.0], vec![1.0]], &[8.0]);
        assert!(near(q[0][0], 6.0) && near(q[1][0], 2.0));
    }

    #[test]
    fn graph_network_routes_and_conserves() {
        // A two-way street 0–1–2 (dead end at 2) and a one-way feeder 3 → 1.
        let d = FundamentalDiagram::URBAN;
        let e = |from, to, reverse| GraphEdge { from, to, length_m: 300.0, lanes: 1, diagram: d, reverse };
        let edges = vec![e(0, 1, Some(1)), e(1, 0, Some(0)), e(1, 2, Some(3)), e(2, 1, Some(2)), e(3, 1, None)];
        let g = build_network(4, &edges, 1.0).unwrap();
        // Node 0: entered by 1→0 and left by 0→1 → a junction (with the U-turn as its only way on).
        // Node 1: a three-in / two-out intersection. Node 2: dead end, U-turn junction. Node 3: source.
        assert_eq!((g.junctions, g.sources.len(), g.sinks.len(), g.short_links), (3, 1, 0, 0));
        let at1 = g.network.nodes().iter().find_map(|n| match n {
            Node::Junction { from, to, turning } if from.len() == 3 => {
                Some((from.clone(), to.clone(), turning.clone()))
            }
            _ => None,
        });
        let (from, to, turning) = at1.unwrap();
        for (row, (input, _)) in turning.iter().zip(&from) {
            for (b, out) in row.iter().zip(&to) {
                if edges[*input].reverse == Some(*out) {
                    assert_eq!(*b, 0.0, "U-turn {input} → {out} allowed at a through node");
                }
            }
        }
        // Feed 0.2 veh/s at the source for ten minutes: nothing leaves (no sinks), so every
        // vehicle that entered is on the network, and no cell ever exceeds jam.
        let mut g = g;
        g.network.set_rate(g.sources[0].0, 0.2);
        let (mut entered, mut exited) = (0.0, 0.0);
        for _ in 0..600 {
            let flows = g.network.step();
            entered += flows[g.sources[0].1].entered;
            exited += g.sinks.iter().map(|&(_, l)| flows[l].exited).sum::<f64>();
            for l in g.network.links() {
                assert!(l.vehicles().iter().all(|&v| v >= -1e-9 && v <= l.jam_capacity() + 1e-9));
            }
        }
        assert!((entered - 0.2 * 600.0).abs() < 1e-6, "{entered}");
        assert!((entered - exited - g.network.total_vehicles()).abs() < 1e-6);
        assert!(g.network.links()[0].total_vehicles() > 0.0, "traffic reached the far street");
        // Malformed input is refused.
        assert_eq!(build_network(2, &[e(0, 5, None)], 1.0), Err(BuildError::UnknownNode(0)));
        let mut bad = e(0, 1, None);
        bad.length_m = 0.0;
        assert_eq!(build_network(2, &[bad], 1.0), Err(BuildError::BadEdge(0)));
    }

    #[test]
    fn links_shorter_than_one_step_conserve_and_stay_bounded() {
        // 5 m at 30 m/s with Δt = 1 s: six cells' worth of free-flow travel per step. Before the
        // advance was capped, a full cell sent six times its content and the clamp at zero
        // created vehicles.
        let mut l = Link::new(FundamentalDiagram::MOTORWAY, 2, 5.0, 1.0).unwrap();
        assert_eq!(l.cell_count(), 1);
        let jam = l.jam_capacity();
        let (mut total_in, mut total_out) = (0.0, 0.0);
        for step in 0..200 {
            // Alternate a hard push and a blocked exit, then a free exit.
            let supply = if step % 20 < 10 { 0.0 } else { f64::INFINITY };
            let f = l.step(5.0, supply);
            total_in += f.entered;
            total_out += f.exited;
            let n = l.total_vehicles();
            assert!((0.0..=jam + 1e-12).contains(&n), "{n} outside [0, {jam}]");
            assert!(f.exited <= n + f.exited - f.entered + 1e-12);
        }
        assert!((total_in - total_out - l.total_vehicles()).abs() < 1e-9);
        assert!(total_out > 0.0);
    }
}
