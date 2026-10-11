//! Pedestrian crowds: Optimal Reciprocal Collision Avoidance in 2D (spec §7B).
//!
//! A port of the RVO2 formulation (van den Berg, Guy, Lin & Manocha 2011). Each agent turns
//! every nearby agent and obstacle segment into a half-plane of permitted velocities, then
//! picks the permitted velocity closest to its preferred one by a 2D linear program. When
//! the half-planes have no common point (a crush), a 3D fallback chooses the velocity that
//! violates the agent constraints least while still honouring the obstacles.
//!
//! Conventions: y up, obstacles are polygons listed **counter-clockwise** (agents stay on
//! their outside); a two-vertex obstacle is a wall blocking both sides. The caller owns
//! routing and sets each agent's preferred velocity (or a goal through
//! [`Simulator::set_goal`]); this module only keeps them from walking through each other.

use std::ops::{Add, Mul, Neg, Sub};

const EPSILON: f64 = 1e-5;

/// A 2D vector in metres (or m/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    /// X.
    pub x: f64,
    /// Y.
    pub y: f64,
}

impl Vec2 {
    /// Builds a vector.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    /// Dot product.
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    /// 2D cross product (z of the 3D cross product).
    pub fn det(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    /// Squared length.
    pub fn length_sq(self) -> f64 {
        self.dot(self)
    }
    /// Length.
    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }
    /// Unit vector; zero stays zero.
    pub fn normalized(self) -> Vec2 {
        let l = self.length();
        if l > 0.0 { self * (1.0 / l) } else { Vec2::default() }
    }
    /// Rotated +90°.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

/// Signed area test: positive when `c` is left of the directed line `a → b`.
fn left_of(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    (a - c).det(b - a)
}

/// Per-agent tuning. Defaults suit a walking adult.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AgentParams {
    /// Body radius, m.
    pub radius: f64,
    /// Speed limit, m/s.
    pub max_speed: f64,
    /// Look-ahead for other agents, s. Larger reacts earlier and gentler.
    pub time_horizon: f64,
    /// Look-ahead for obstacles, s.
    pub time_horizon_obstacle: f64,
    /// Only agents within this distance are considered, m.
    pub neighbor_distance: f64,
    /// At most this many nearest agents are considered.
    pub max_neighbors: usize,
}

impl Default for AgentParams {
    fn default() -> Self {
        Self {
            radius: 0.3,
            max_speed: 1.4,
            time_horizon: 5.0,
            time_horizon_obstacle: 2.0,
            neighbor_distance: 10.0,
            max_neighbors: 10,
        }
    }
}

/// One pedestrian.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Agent {
    /// Position, m.
    pub position: Vec2,
    /// Current velocity, m/s.
    pub velocity: Vec2,
    /// Velocity the agent would take if alone, m/s.
    pub preferred_velocity: Vec2,
    /// Tuning.
    pub params: AgentParams,
}

/// One directed obstacle edge, as RVO2 stores them.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ObstacleEdge {
    point: Vec2,
    unit_dir: Vec2,
    is_convex: bool,
    prev: usize,
    next: usize,
}

/// Half-plane of permitted velocities: points `v` with `det(direction, v − point) ≥ 0`
/// (the left of the directed line).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Line {
    point: Vec2,
    direction: Vec2,
}

/// The crowd.
#[derive(Clone, Debug, PartialEq)]
pub struct Simulator {
    time_step: f64,
    agents: Vec<Option<Agent>>,
    free: Vec<usize>,
    edges: Vec<ObstacleEdge>,
}

/// Stable handle to an agent slot.
pub type AgentId = usize;

/// Why an agent operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrcaError {
    /// No live agent at that id.
    NoSuchAgent,
    /// A polygon needs at least two vertices.
    DegenerateObstacle,
    /// Parameters must be positive and finite.
    BadParams,
}

impl Simulator {
    /// Creates a simulator stepping by `time_step` seconds (must be positive).
    pub fn new(time_step: f64) -> Simulator {
        Simulator {
            time_step: if time_step > 0.0 { time_step } else { 0.1 },
            agents: Vec::new(),
            free: Vec::new(),
            edges: Vec::new(),
        }
    }

    /// Step length, s.
    pub fn time_step(&self) -> f64 {
        self.time_step
    }

    /// Adds an agent at rest; returns its id. Ids of removed agents are reused.
    pub fn add_agent(&mut self, position: Vec2, params: AgentParams) -> Result<AgentId, OrcaError> {
        let ok = params.radius > 0.0
            && params.max_speed > 0.0
            && params.time_horizon > 0.0
            && params.time_horizon_obstacle > 0.0
            && params.neighbor_distance >= 0.0
            && [
                params.radius,
                params.max_speed,
                params.time_horizon,
                params.time_horizon_obstacle,
                params.neighbor_distance,
            ]
            .iter()
            .all(|v| v.is_finite());
        if !ok {
            return Err(OrcaError::BadParams);
        }
        let agent = Agent { position, velocity: Vec2::default(), preferred_velocity: Vec2::default(), params };
        if let Some(id) = self.free.pop() {
            self.agents[id] = Some(agent);
            Ok(id)
        } else {
            self.agents.push(Some(agent));
            Ok(self.agents.len() - 1)
        }
    }

    /// Removes an agent; its id may be handed out again.
    pub fn remove_agent(&mut self, id: AgentId) -> bool {
        match self.agents.get_mut(id) {
            Some(slot @ Some(_)) => {
                *slot = None;
                self.free.push(id);
                true
            }
            _ => false,
        }
    }

    /// Live agent count.
    pub fn agent_count(&self) -> usize {
        self.agents.iter().filter(|a| a.is_some()).count()
    }

    /// Number of slots (highest id + 1).
    pub fn slot_count(&self) -> usize {
        self.agents.len()
    }

    /// The agent, if live.
    pub fn agent(&self, id: AgentId) -> Option<&Agent> {
        self.agents.get(id).and_then(Option::as_ref)
    }

    /// Mutable access to a live agent.
    pub fn agent_mut(&mut self, id: AgentId) -> Option<&mut Agent> {
        self.agents.get_mut(id).and_then(Option::as_mut)
    }

    /// Sets the velocity the agent wants.
    pub fn set_preferred_velocity(&mut self, id: AgentId, v: Vec2) -> Result<(), OrcaError> {
        self.agent_mut(id).map(|a| a.preferred_velocity = v).ok_or(OrcaError::NoSuchAgent)
    }

    /// Points the agent at `goal` at `speed`, easing off inside `slow_radius` so it stops
    /// on the goal instead of orbiting it.
    pub fn set_goal(&mut self, id: AgentId, goal: Vec2, speed: f64, slow_radius: f64) -> Result<(), OrcaError> {
        let a = self.agent_mut(id).ok_or(OrcaError::NoSuchAgent)?;
        a.preferred_velocity = preferred_velocity_toward(a.position, goal, speed, slow_radius);
        Ok(())
    }

    /// Adds a polygonal obstacle (counter-clockwise vertices; two vertices make a wall).
    pub fn add_obstacle(&mut self, vertices: &[Vec2]) -> Result<(), OrcaError> {
        if vertices.len() < 2 {
            return Err(OrcaError::DegenerateObstacle);
        }
        let first = self.edges.len();
        let n = vertices.len();
        for i in 0..n {
            let point = vertices[i];
            let next_point = vertices[(i + 1) % n];
            let is_convex = if n == 2 { true } else { left_of(vertices[(i + n - 1) % n], point, next_point) >= 0.0 };
            self.edges.push(ObstacleEdge {
                point,
                unit_dir: (next_point - point).normalized(),
                is_convex,
                prev: first + (i + n - 1) % n,
                next: first + (i + 1) % n,
            });
        }
        Ok(())
    }

    /// Number of obstacle edges.
    pub fn obstacle_edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Advances every agent one step: new velocities are computed from the current state of
    /// all agents (so the order of agents does not matter), then positions are integrated.
    pub fn step(&mut self) {
        let grid = Grid::build(&self.agents);
        let mut new_velocities: Vec<Option<Vec2>> = vec![None; self.agents.len()];
        for (id, slot) in self.agents.iter().enumerate() {
            if let Some(agent) = slot {
                let neighbors = grid.neighbors(&self.agents, id, agent);
                let (v, _) = self.compute_new_velocity(agent, &neighbors);
                new_velocities[id] = Some(v);
            }
        }
        let dt = self.time_step;
        for (slot, v) in self.agents.iter_mut().zip(new_velocities) {
            if let (Some(agent), Some(v)) = (slot, v) {
                agent.velocity = v;
                agent.position = agent.position + v * dt;
            }
        }
    }

    /// ORCA for one agent given its neighbours. Returns the velocity and whether the full
    /// constraint set was feasible (`false` means the 3D fallback chose the safest velocity).
    pub fn compute_new_velocity(&self, agent: &Agent, neighbors: &[&Agent]) -> (Vec2, bool) {
        let mut lines: Vec<Line> = Vec::new();
        self.obstacle_lines(agent, &mut lines);
        let num_obstacle_lines = lines.len();
        agent_lines(agent, neighbors, self.time_step, &mut lines);

        let mut result = Vec2::default();
        let fail = linear_program2(&lines, agent.params.max_speed, agent.preferred_velocity, false, &mut result);
        let feasible = fail == lines.len();
        if !feasible {
            linear_program3(&lines, num_obstacle_lines, fail, agent.params.max_speed, &mut result);
        }
        (result, feasible)
    }

    /// Obstacle half-planes, following RVO2's `Agent::computeNewVelocity` case by case.
    fn obstacle_lines(&self, agent: &Agent, lines: &mut Vec<Line>) {
        let radius = agent.params.radius;
        let radius_sq = radius * radius;
        let inv_t = 1.0 / agent.params.time_horizon_obstacle;
        let range = agent.params.time_horizon_obstacle * agent.params.max_speed + radius;
        let range_sq = range * range;
        let pos = agent.position;
        let vel = agent.velocity;

        // Edges within range whose exterior (right) side faces the agent, nearest first so
        // the "already covered" pruning sees the strong constraints first. The side test is
        // RVO2's: an edge seen from behind would push the agent through it.
        let mut candidates: Vec<(f64, usize)> = self
            .edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| {
                let next = self.edges[e.next].point;
                if left_of(e.point, next, pos) >= 0.0 {
                    return None;
                }
                let d = dist_sq_point_segment(e.point, next, pos);
                (d < range_sq).then_some((d, i))
            })
            .collect();
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        for (_, idx) in candidates {
            let mut obstacle1 = self.edges[idx];
            let mut obstacle2 = self.edges[obstacle1.next];
            let rel1 = obstacle1.point - pos;
            let rel2 = obstacle2.point - pos;

            let already_covered = lines.iter().any(|l| {
                (rel1 * inv_t - l.point).det(l.direction) - inv_t * radius >= -EPSILON
                    && (rel2 * inv_t - l.point).det(l.direction) - inv_t * radius >= -EPSILON
            });
            if already_covered {
                continue;
            }

            let dist_sq1 = rel1.length_sq();
            let dist_sq2 = rel2.length_sq();
            let obstacle_vector = obstacle2.point - obstacle1.point;
            let s = (-rel1).dot(obstacle_vector) / obstacle_vector.length_sq();
            let dist_sq_line = (-rel1 - obstacle_vector * s).length_sq();

            if s < 0.0 && dist_sq1 <= radius_sq {
                // Collision with left vertex; ignore if non-convex.
                if obstacle1.is_convex {
                    lines.push(Line { point: Vec2::default(), direction: Vec2::new(-rel1.y, rel1.x).normalized() });
                }
                continue;
            } else if s > 1.0 && dist_sq2 <= radius_sq {
                // Collision with right vertex; ignore if non-convex or it will be taken care of
                // by the neighbouring obstacle.
                if obstacle2.is_convex && rel2.det(obstacle2.unit_dir) >= 0.0 {
                    lines.push(Line { point: Vec2::default(), direction: Vec2::new(-rel2.y, rel2.x).normalized() });
                }
                continue;
            } else if (0.0..1.0).contains(&s) && dist_sq_line <= radius_sq {
                // Collision with the segment itself.
                lines.push(Line { point: Vec2::default(), direction: -obstacle1.unit_dir });
                continue;
            }

            // No collision: compute the legs of the velocity obstacle.
            let (left_leg, right_leg);
            if s < 0.0 && dist_sq_line <= radius_sq {
                // Seen obliquely: the left vertex defines the VO alone.
                if !obstacle1.is_convex {
                    continue;
                }
                obstacle2 = obstacle1;
                let leg1 = (dist_sq1 - radius_sq).sqrt();
                left_leg =
                    Vec2::new(rel1.x * leg1 - rel1.y * radius, rel1.x * radius + rel1.y * leg1) * (1.0 / dist_sq1);
                right_leg =
                    Vec2::new(rel1.x * leg1 + rel1.y * radius, -rel1.x * radius + rel1.y * leg1) * (1.0 / dist_sq1);
            } else if s > 1.0 && dist_sq_line <= radius_sq {
                // Seen obliquely: the right vertex defines the VO alone.
                if !obstacle2.is_convex {
                    continue;
                }
                obstacle1 = obstacle2;
                let leg2 = (dist_sq2 - radius_sq).sqrt();
                left_leg =
                    Vec2::new(rel2.x * leg2 - rel2.y * radius, rel2.x * radius + rel2.y * leg2) * (1.0 / dist_sq2);
                right_leg =
                    Vec2::new(rel2.x * leg2 + rel2.y * radius, -rel2.x * radius + rel2.y * leg2) * (1.0 / dist_sq2);
            } else {
                left_leg = if obstacle1.is_convex {
                    let leg1 = (dist_sq1 - radius_sq).sqrt();
                    Vec2::new(rel1.x * leg1 - rel1.y * radius, rel1.x * radius + rel1.y * leg1) * (1.0 / dist_sq1)
                } else {
                    -obstacle1.unit_dir
                };
                right_leg = if obstacle2.is_convex {
                    let leg2 = (dist_sq2 - radius_sq).sqrt();
                    Vec2::new(rel2.x * leg2 + rel2.y * radius, -rel2.x * radius + rel2.y * leg2) * (1.0 / dist_sq2)
                } else {
                    obstacle1.unit_dir
                };
            }

            // Legs never point into a neighbouring edge at a convex vertex: use that edge's
            // cut-off line instead.
            let left_neighbor = self.edges[obstacle1.prev];
            let mut left_leg = left_leg;
            let mut right_leg = right_leg;
            let mut left_foreign = false;
            let mut right_foreign = false;
            if obstacle1.is_convex && left_leg.det(-left_neighbor.unit_dir) >= 0.0 {
                left_leg = -left_neighbor.unit_dir;
                left_foreign = true;
            }
            if obstacle2.is_convex && right_leg.det(obstacle2.unit_dir) <= 0.0 {
                right_leg = obstacle2.unit_dir;
                right_foreign = true;
            }

            let left_cutoff = (obstacle1.point - pos) * inv_t;
            let right_cutoff = (obstacle2.point - pos) * inv_t;
            let cutoff_vec = right_cutoff - left_cutoff;
            let same = obstacle1 == obstacle2;

            // Project the current velocity onto the velocity obstacle.
            let t = if same { 0.5 } else { (vel - left_cutoff).dot(cutoff_vec) / cutoff_vec.length_sq() };
            let t_left = (vel - left_cutoff).dot(left_leg);
            let t_right = (vel - right_cutoff).dot(right_leg);

            if (t < 0.0 && t_left < 0.0) || (same && t_left < 0.0 && t_right < 0.0) {
                let unit_w = (vel - left_cutoff).normalized();
                lines.push(Line {
                    direction: Vec2::new(unit_w.y, -unit_w.x),
                    point: left_cutoff + unit_w * (radius * inv_t),
                });
                continue;
            } else if t > 1.0 && t_right < 0.0 {
                let unit_w = (vel - right_cutoff).normalized();
                lines.push(Line {
                    direction: Vec2::new(unit_w.y, -unit_w.x),
                    point: right_cutoff + unit_w * (radius * inv_t),
                });
                continue;
            }

            let dist_sq_cutoff = if !(0.0..=1.0).contains(&t) || same {
                f64::INFINITY
            } else {
                (vel - (left_cutoff + cutoff_vec * t)).length_sq()
            };
            let dist_sq_left =
                if t_left < 0.0 { f64::INFINITY } else { (vel - (left_cutoff + left_leg * t_left)).length_sq() };
            let dist_sq_right =
                if t_right < 0.0 { f64::INFINITY } else { (vel - (right_cutoff + right_leg * t_right)).length_sq() };

            if dist_sq_cutoff <= dist_sq_left && dist_sq_cutoff <= dist_sq_right {
                let direction = -obstacle1.unit_dir;
                lines.push(Line { direction, point: left_cutoff + direction.perp() * (radius * inv_t) });
                continue;
            }
            if dist_sq_left <= dist_sq_right {
                if left_foreign {
                    continue;
                }
                lines.push(Line { direction: left_leg, point: left_cutoff + left_leg.perp() * (radius * inv_t) });
                continue;
            }
            if right_foreign {
                continue;
            }
            let direction = -right_leg;
            lines.push(Line { direction, point: right_cutoff + direction.perp() * (radius * inv_t) });
        }
    }
}

/// Agent–agent half-planes (responsibility shared 50/50, hence the `0.5 * u`).
fn agent_lines(agent: &Agent, neighbors: &[&Agent], time_step: f64, lines: &mut Vec<Line>) {
    let inv_t = 1.0 / agent.params.time_horizon;
    for other in neighbors {
        let rel_pos = other.position - agent.position;
        let rel_vel = agent.velocity - other.velocity;
        let dist_sq = rel_pos.length_sq();
        let combined = agent.params.radius + other.params.radius;
        let combined_sq = combined * combined;
        let (direction, u);
        if dist_sq > combined_sq {
            let w = rel_vel - rel_pos * inv_t;
            let w_len_sq = w.length_sq();
            let dot1 = w.dot(rel_pos);
            if dot1 < 0.0 && dot1 * dot1 > combined_sq * w_len_sq {
                // Project on the cut-off circle.
                let w_len = w_len_sq.sqrt();
                let unit_w = w * (1.0 / w_len);
                direction = Vec2::new(unit_w.y, -unit_w.x);
                u = unit_w * (combined * inv_t - w_len);
            } else {
                // Project on a leg.
                let leg = (dist_sq - combined_sq).sqrt();
                direction = if rel_pos.det(w) > 0.0 {
                    Vec2::new(rel_pos.x * leg - rel_pos.y * combined, rel_pos.x * combined + rel_pos.y * leg)
                        * (1.0 / dist_sq)
                } else {
                    -(Vec2::new(rel_pos.x * leg + rel_pos.y * combined, -rel_pos.x * combined + rel_pos.y * leg)
                        * (1.0 / dist_sq))
                };
                u = direction * rel_vel.dot(direction) - rel_vel;
            }
        } else {
            // Already overlapping: separate within one time step.
            let inv_step = 1.0 / time_step;
            let w = rel_vel - rel_pos * inv_step;
            let w_len = w.length();
            let unit_w = if w_len > 0.0 { w * (1.0 / w_len) } else { Vec2::new(1.0, 0.0) };
            direction = Vec2::new(unit_w.y, -unit_w.x);
            u = unit_w * (combined * inv_step - w_len);
        }
        lines.push(Line { point: agent.velocity + u * 0.5, direction });
    }
}

/// Solves a 1D LP on line `line_no` within the speed disc and the earlier lines.
fn linear_program1(
    lines: &[Line],
    line_no: usize,
    radius: f64,
    opt: Vec2,
    direction_opt: bool,
    result: &mut Vec2,
) -> bool {
    let line = lines[line_no];
    let dot = line.point.dot(line.direction);
    let discriminant = dot * dot + radius * radius - line.point.length_sq();
    if discriminant < 0.0 {
        return false; // the speed disc fully invalidates this line
    }
    let sqrt_d = discriminant.sqrt();
    let mut t_left = -dot - sqrt_d;
    let mut t_right = -dot + sqrt_d;
    for other in &lines[..line_no] {
        let denominator = line.direction.det(other.direction);
        let numerator = other.direction.det(line.point - other.point);
        if denominator.abs() <= EPSILON {
            if numerator < 0.0 {
                return false; // parallel and on the wrong side
            }
            continue;
        }
        let t = numerator / denominator;
        if denominator >= 0.0 {
            t_right = t_right.min(t);
        } else {
            t_left = t_left.max(t);
        }
        if t_left > t_right {
            return false;
        }
    }
    *result = if direction_opt {
        if opt.dot(line.direction) > 0.0 {
            line.point + line.direction * t_right
        } else {
            line.point + line.direction * t_left
        }
    } else {
        let t = line.direction.dot(opt - line.point);
        line.point + line.direction * t.clamp(t_left, t_right)
    };
    true
}

/// Solves the 2D LP; returns `lines.len()` on success, else the index of the first line
/// that could not be satisfied (and `result` holds the best point before it).
fn linear_program2(lines: &[Line], radius: f64, opt: Vec2, direction_opt: bool, result: &mut Vec2) -> usize {
    *result = if direction_opt {
        opt * radius
    } else if opt.length_sq() > radius * radius {
        opt.normalized() * radius
    } else {
        opt
    };
    for (i, line) in lines.iter().enumerate() {
        if line.direction.det(line.point - *result) > 0.0 {
            let saved = *result;
            if !linear_program1(lines, i, radius, opt, direction_opt, result) {
                *result = saved;
                return i;
            }
        }
    }
    lines.len()
}

/// Infeasible case: minimise the largest violation of the agent lines while keeping every
/// obstacle line hard.
fn linear_program3(lines: &[Line], num_obstacle_lines: usize, begin: usize, radius: f64, result: &mut Vec2) {
    let mut distance = 0.0;
    for i in begin..lines.len() {
        if lines[i].direction.det(lines[i].point - *result) > distance {
            let mut proj: Vec<Line> = lines[..num_obstacle_lines].to_vec();
            for j in num_obstacle_lines..i {
                let determinant = lines[i].direction.det(lines[j].direction);
                let point = if determinant.abs() <= EPSILON {
                    if lines[i].direction.dot(lines[j].direction) > 0.0 {
                        continue; // same direction: redundant
                    }
                    (lines[i].point + lines[j].point) * 0.5
                } else {
                    lines[i].point
                        + lines[i].direction * (lines[j].direction.det(lines[i].point - lines[j].point) / determinant)
                };
                proj.push(Line { point, direction: (lines[j].direction - lines[i].direction).normalized() });
            }
            let saved = *result;
            if linear_program2(&proj, radius, lines[i].direction.perp(), true, result) < proj.len() {
                *result = saved; // should not happen; keep the previous best
            }
            distance = lines[i].direction.det(lines[i].point - *result);
        }
    }
}

fn dist_sq_point_segment(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    let ab = b - a;
    let len_sq = ab.length_sq();
    if len_sq <= 0.0 {
        return (c - a).length_sq();
    }
    let r = ((c - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    (c - (a + ab * r)).length_sq()
}

/// Preferred velocity toward a goal, easing inside `slow_radius`.
pub fn preferred_velocity_toward(position: Vec2, goal: Vec2, speed: f64, slow_radius: f64) -> Vec2 {
    let to_goal = goal - position;
    let d = to_goal.length();
    if d < 1e-9 {
        return Vec2::default();
    }
    let s = if slow_radius > 0.0 && d < slow_radius { speed * d / slow_radius } else { speed };
    to_goal * (s / d)
}

/// A path to follow: waypoints in order, the last being the destination. It turns a route on
/// the navigation graph into the preferred velocity ORCA needs each step.
#[derive(Clone, Debug, PartialEq)]
pub struct Route {
    waypoints: Vec<Vec2>,
    next: usize,
}

impl Route {
    /// A route through `waypoints`; an empty route is already finished.
    pub fn new(waypoints: Vec<Vec2>) -> Route {
        Route { waypoints, next: 0 }
    }

    /// Index of the waypoint being steered for.
    pub fn next_index(&self) -> usize {
        self.next
    }

    /// Waypoints, destination last.
    pub fn waypoints(&self) -> &[Vec2] {
        &self.waypoints
    }

    /// Whether `position` is within `reach` of the destination.
    pub fn arrived(&self, position: Vec2, reach: f64) -> bool {
        self.waypoints.last().is_none_or(|d| (*d - position).length() <= reach)
    }

    /// Preferred velocity at `position`: toward the next waypoint at `speed`, slowing inside
    /// `reach` of the destination. A waypoint counts as passed when the agent comes within
    /// `reach` of it, or when the agent is already beyond it along the next leg and within
    /// three times `reach` (a crowd can push an agent around a waypoint without touching it).
    pub fn preferred_velocity(&mut self, position: Vec2, speed: f64, reach: f64) -> Vec2 {
        while self.next + 1 < self.waypoints.len() {
            let w = self.waypoints[self.next];
            let leg = self.waypoints[self.next + 1] - w;
            let off = position - w;
            let close = off.length() <= reach;
            let beyond = off.length() <= 3.0 * reach && off.dot(leg) > 0.0;
            if close || beyond {
                self.next += 1;
            } else {
                break;
            }
        }
        match self.waypoints.get(self.next) {
            Some(&goal) if self.next + 1 == self.waypoints.len() => {
                preferred_velocity_toward(position, goal, speed, reach)
            }
            Some(&goal) => preferred_velocity_toward(position, goal, speed, 0.0),
            None => Vec2::default(),
        }
    }
}

/// Uniform grid for neighbour queries; cell size is the largest neighbour distance.
struct Grid {
    cell: f64,
    buckets: std::collections::HashMap<(i64, i64), Vec<usize>>,
}

impl Grid {
    fn build(agents: &[Option<Agent>]) -> Grid {
        let cell = agents.iter().flatten().map(|a| a.params.neighbor_distance).fold(1.0, f64::max);
        let mut buckets: std::collections::HashMap<(i64, i64), Vec<usize>> = std::collections::HashMap::new();
        for (i, a) in agents.iter().enumerate() {
            if let Some(a) = a {
                buckets.entry(Self::key(a.position, cell)).or_default().push(i);
            }
        }
        Grid { cell, buckets }
    }

    fn key(p: Vec2, cell: f64) -> (i64, i64) {
        ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64)
    }

    /// Nearest `max_neighbors` live agents within `neighbor_distance`, nearest first.
    fn neighbors<'a>(&self, agents: &'a [Option<Agent>], id: usize, agent: &Agent) -> Vec<&'a Agent> {
        let range_sq = agent.params.neighbor_distance * agent.params.neighbor_distance;
        let (cx, cy) = Self::key(agent.position, self.cell);
        let mut found: Vec<(f64, usize)> = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(bucket) = self.buckets.get(&(cx + dx, cy + dy)) {
                    for &j in bucket {
                        if j == id {
                            continue;
                        }
                        if let Some(other) = &agents[j] {
                            let d = (other.position - agent.position).length_sq();
                            if d < range_sq {
                                found.push((d, j));
                            }
                        }
                    }
                }
            }
        }
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        found.truncate(agent.params.max_neighbors);
        found.into_iter().filter_map(|(_, j)| agents[j].as_ref()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    /// Smallest pairwise distance minus combined radii over all live agents (negative = overlap).
    fn min_clearance(sim: &Simulator) -> f64 {
        let agents: Vec<&Agent> = (0..sim.slot_count()).filter_map(|i| sim.agent(i)).collect();
        let mut best = f64::INFINITY;
        for i in 0..agents.len() {
            for j in i + 1..agents.len() {
                let d = (agents[i].position - agents[j].position).length()
                    - agents[i].params.radius
                    - agents[j].params.radius;
                best = best.min(d);
            }
        }
        best
    }

    fn all_arrived(sim: &Simulator, goals: &[(AgentId, Vec2)], tol: f64) -> bool {
        goals.iter().all(|(id, g)| (sim.agent(*id).unwrap().position - *g).length() < tol)
    }

    fn drive(
        sim: &mut Simulator,
        goals: &[(AgentId, Vec2)],
        speed: f64,
        max_steps: usize,
        min_clear: &mut f64,
    ) -> usize {
        for step in 0..max_steps {
            for (id, g) in goals {
                sim.set_goal(*id, *g, speed, 1.0).unwrap();
            }
            sim.step();
            *min_clear = min_clear.min(min_clearance(sim));
            if all_arrived(sim, goals, 0.35) {
                return step;
            }
        }
        max_steps
    }

    #[test]
    fn alone_an_agent_takes_its_preferred_velocity_within_max_speed() {
        let mut sim = Simulator::new(0.1);
        let id = sim.add_agent(Vec2::new(0.0, 0.0), AgentParams::default()).unwrap();
        sim.set_preferred_velocity(id, Vec2::new(1.0, 0.5)).unwrap();
        sim.step();
        let a = sim.agent(id).unwrap();
        assert!(near(a.velocity.x, 1.0, 1e-12) && near(a.velocity.y, 0.5, 1e-12));
        assert!(near(a.position.x, 0.1, 1e-12));
        sim.set_preferred_velocity(id, Vec2::new(10.0, 0.0)).unwrap();
        sim.step();
        assert!(near(sim.agent(id).unwrap().velocity.length(), 1.4, 1e-12));
    }

    #[test]
    fn head_on_pair_pass_without_touching() {
        let mut sim = Simulator::new(0.1);
        let a = sim.add_agent(Vec2::new(-8.0, 0.0), AgentParams::default()).unwrap();
        let b = sim.add_agent(Vec2::new(8.0, 0.05), AgentParams::default()).unwrap();
        let goals = [(a, Vec2::new(8.0, 0.0)), (b, Vec2::new(-8.0, 0.05))];
        let mut clear = f64::INFINITY;
        let steps = drive(&mut sim, &goals, 1.4, 400, &mut clear);
        assert!(steps < 400, "did not arrive");
        assert!(clear > -1e-3, "overlap of {clear} m");
        // They did not simply stop: the trip took under twice the free-flight time.
        assert!(steps as f64 * 0.1 < 2.0 * 16.0 / 1.4, "{steps} steps");
    }

    #[test]
    fn circle_crossing_is_collision_free_and_everyone_arrives() {
        let n = 24;
        let mut sim = Simulator::new(0.1);
        let mut goals = Vec::new();
        for i in 0..n {
            // Slight deterministic asymmetry breaks the perfect symmetry that stalls ORCA.
            let angle = std::f64::consts::TAU * i as f64 / n as f64 + 0.01 * ((i * 7) % 5) as f64;
            let p = Vec2::new(12.0 * angle.cos(), 12.0 * angle.sin());
            // With everyone converging on one point, each agent must see all the others;
            // the default cap of 10 nearest would ignore half the crowd.
            let params = AgentParams { max_neighbors: 32, neighbor_distance: 15.0, ..Default::default() };
            let id = sim.add_agent(p, params).unwrap();
            goals.push((id, -p));
        }
        let mut clear = f64::INFINITY;
        let steps = drive(&mut sim, &goals, 1.4, 1500, &mut clear);
        assert!(steps < 1500, "did not all arrive");
        // Everyone meets at the centre at once: the constraint set is infeasible there and
        // ORCA's guarantee lapses, so the fallback only bounds the overlap (a few cm against
        // a 30 cm radius) rather than eliminating it.
        assert!(clear > -0.05, "overlap of {clear} m");
    }

    /// ORCA is local avoidance, not a planner: pointed straight at a block it stops at the
    /// face; given waypoints around it (the spec's navigation graph) it follows them without
    /// ever penetrating the block.
    #[test]
    fn block_is_never_penetrated() {
        let mut sim = Simulator::new(0.1);
        // Counter-clockwise square: x ∈ [2, 4], y ∈ [−1, 1].
        let square = [Vec2::new(2.0, -1.0), Vec2::new(4.0, -1.0), Vec2::new(4.0, 1.0), Vec2::new(2.0, 1.0)];
        sim.add_obstacle(&square).unwrap();
        assert_eq!(sim.obstacle_edge_count(), 4);
        let r = 0.3;
        let clearance = |p: Vec2| {
            let dx = (2.0 - p.x).max(0.0).max(p.x - 4.0);
            let dy = (-1.0 - p.y).max(0.0).max(p.y - 1.0);
            (dx * dx + dy * dy).sqrt()
        };

        // Straight at the face: stops with its radius of clearance, never inside.
        let id = sim.add_agent(Vec2::new(0.0, 0.1), AgentParams::default()).unwrap();
        for _ in 0..200 {
            sim.set_goal(id, Vec2::new(7.0, 0.1), 1.4, 1.0).unwrap();
            sim.step();
            let p = sim.agent(id).unwrap().position;
            assert!(clearance(p) >= r - 0.02, "penetrated: centre {p:?}");
        }
        let p = sim.agent(id).unwrap().position;
        assert!(near(p.x, 2.0 - r, 0.05) && sim.agent(id).unwrap().velocity.length() < 0.05, "{p:?}");

        // Routed around via waypoints: arrives, still never inside.
        sim.remove_agent(id);
        let id = sim.add_agent(Vec2::new(0.0, 0.1), AgentParams::default()).unwrap();
        let route = [Vec2::new(1.0, 2.0), Vec2::new(5.0, 2.0), Vec2::new(7.0, 0.1)];
        let mut leg = 0;
        let mut arrived = false;
        for _ in 0..600 {
            sim.set_goal(id, route[leg], 1.4, 0.5).unwrap();
            sim.step();
            let p = sim.agent(id).unwrap().position;
            assert!(clearance(p) >= r - 0.02, "penetrated: centre {p:?}");
            if (p - route[leg]).length() < 0.4 {
                if leg + 1 == route.len() {
                    arrived = true;
                    break;
                }
                leg += 1;
            }
        }
        assert!(arrived, "agent did not complete the route");

        // A bare wall segment blocks from both sides.
        let mut sim = Simulator::new(0.1);
        sim.add_obstacle(&[Vec2::new(3.0, -5.0), Vec2::new(3.0, 5.0)]).unwrap();
        let left = sim.add_agent(Vec2::new(0.0, 0.0), AgentParams::default()).unwrap();
        let right = sim.add_agent(Vec2::new(6.0, 2.0), AgentParams::default()).unwrap();
        for _ in 0..200 {
            sim.set_preferred_velocity(left, Vec2::new(1.4, 0.0)).unwrap();
            sim.set_preferred_velocity(right, Vec2::new(-1.4, 0.0)).unwrap();
            sim.step();
            assert!(sim.agent(left).unwrap().position.x <= 3.0 - r + 0.02);
            assert!(sim.agent(right).unwrap().position.x >= 3.0 + r - 0.02);
        }
        assert!(near(sim.agent(left).unwrap().position.x, 3.0 - r, 0.05));
    }

    #[test]
    fn corridor_with_opposing_streams() {
        let mut sim = Simulator::new(0.1);
        sim.add_obstacle(&[Vec2::new(-20.0, 2.0), Vec2::new(20.0, 2.0)]).unwrap();
        sim.add_obstacle(&[Vec2::new(-20.0, -2.0), Vec2::new(20.0, -2.0)]).unwrap();
        // Three per direction, 1 m apart across a 4 m corridor: tight but passable (five per
        // direction at 0.6 m would fill the width solid in both directions and jam, as it
        // should).
        let mut goals = Vec::new();
        for i in 0..3 {
            let y = -1.0 + 1.0 * i as f64;
            let a = sim.add_agent(Vec2::new(-10.0 - 0.9 * i as f64, y), AgentParams::default()).unwrap();
            goals.push((a, Vec2::new(10.0, y)));
            let b = sim.add_agent(Vec2::new(10.0 + 0.9 * i as f64, y + 0.15), AgentParams::default()).unwrap();
            goals.push((b, Vec2::new(-10.0, y + 0.15)));
        }
        let mut clear = f64::INFINITY;
        let steps = drive(&mut sim, &goals, 1.3, 1200, &mut clear);
        assert!(steps < 1200, "streams did not resolve");
        assert!(clear > -0.01, "overlap of {clear} m");
        for id in 0..sim.slot_count() {
            let p = sim.agent(id).unwrap().position;
            assert!(p.y.abs() <= 2.0 - 0.3 + 0.02, "{p:?} crossed a wall");
        }
    }

    #[test]
    fn infeasible_crush_falls_back_to_a_bounded_safest_velocity() {
        let mut sim = Simulator::new(0.1);
        let centre = sim.add_agent(Vec2::new(0.0, 0.0), AgentParams::default()).unwrap();
        // Eight agents just outside touching distance, all pressing inward.
        let mut ring = Vec::new();
        for i in 0..8 {
            let a = std::f64::consts::TAU * i as f64 / 8.0;
            let p = Vec2::new(0.65 * a.cos(), 0.65 * a.sin());
            let id = sim.add_agent(p, AgentParams::default()).unwrap();
            sim.agent_mut(id).unwrap().velocity = -p.normalized() * 1.0;
            ring.push(id);
        }
        sim.set_preferred_velocity(centre, Vec2::new(1.4, 0.0)).unwrap();
        let agent = *sim.agent(centre).unwrap();
        let neighbors: Vec<&Agent> = ring.iter().map(|id| sim.agent(*id).unwrap()).collect();
        let (v, feasible) = sim.compute_new_velocity(&agent, &neighbors);
        assert!(!feasible, "expected the ring to be infeasible");
        assert!(v.x.is_finite() && v.y.is_finite());
        assert!(v.length() <= 1.4 + 1e-9);
        // Stepping the crush never produces NaN positions.
        for _ in 0..50 {
            sim.step();
        }
        for id in 0..sim.slot_count() {
            let p = sim.agent(id).unwrap().position;
            assert!(p.x.is_finite() && p.y.is_finite());
        }
    }

    #[test]
    fn deterministic_and_order_independent() {
        let build = |swap: bool| {
            let mut sim = Simulator::new(0.1);
            let pts = [Vec2::new(-5.0, 0.0), Vec2::new(5.0, 0.2), Vec2::new(0.0, 5.0), Vec2::new(0.3, -5.0)];
            let order: Vec<usize> = if swap { vec![3, 2, 1, 0] } else { vec![0, 1, 2, 3] };
            let mut ids = [0; 4];
            for &i in &order {
                ids[i] = sim.add_agent(pts[i], AgentParams::default()).unwrap();
            }
            for _ in 0..150 {
                for (i, id) in ids.iter().enumerate() {
                    sim.set_goal(*id, -pts[i], 1.4, 1.0).unwrap();
                }
                sim.step();
            }
            ids.iter().map(|id| sim.agent(*id).unwrap().position).collect::<Vec<_>>()
        };
        let a = build(false);
        let b = build(false);
        assert_eq!(a, b, "same input must give identical output");
        let c = build(true);
        for (p, q) in a.iter().zip(&c) {
            assert!((*p - *q).length() < 1e-9, "insertion order changed the result: {p:?} vs {q:?}");
        }
    }

    #[test]
    fn goal_helper_and_slot_reuse() {
        let v = preferred_velocity_toward(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), 1.4, 1.0);
        assert!(near(v.x, 1.4, 1e-12) && v.y == 0.0);
        let v = preferred_velocity_toward(Vec2::new(0.0, 0.0), Vec2::new(0.5, 0.0), 1.4, 1.0);
        assert!(near(v.x, 0.7, 1e-12));
        assert_eq!(preferred_velocity_toward(Vec2::new(1.0, 1.0), Vec2::new(1.0, 1.0), 1.4, 1.0), Vec2::default());

        let mut sim = Simulator::new(0.1);
        let a = sim.add_agent(Vec2::default(), AgentParams::default()).unwrap();
        let b = sim.add_agent(Vec2::new(1.0, 0.0), AgentParams::default()).unwrap();
        assert_eq!((a, b), (0, 1));
        assert!(sim.remove_agent(a));
        assert!(!sim.remove_agent(a));
        assert_eq!(sim.agent_count(), 1);
        assert_eq!(sim.set_preferred_velocity(a, Vec2::default()), Err(OrcaError::NoSuchAgent));
        let c = sim.add_agent(Vec2::new(2.0, 0.0), AgentParams::default()).unwrap();
        assert_eq!(c, 0, "freed slot is reused");
        assert_eq!(sim.add_obstacle(&[Vec2::default()]), Err(OrcaError::DegenerateObstacle));
        assert_eq!(
            sim.add_agent(Vec2::default(), AgentParams { radius: 0.0, ..Default::default() }),
            Err(OrcaError::BadParams)
        );
        assert_eq!(Simulator::new(-1.0).time_step(), 0.1);
    }

    #[test]
    fn neighbor_limit_and_distance_are_honoured() {
        let mut sim = Simulator::new(0.1);
        let params = AgentParams { max_neighbors: 2, neighbor_distance: 3.0, ..Default::default() };
        let me = sim.add_agent(Vec2::default(), params).unwrap();
        for i in 1..=5 {
            sim.add_agent(Vec2::new(i as f64, 0.0), params).unwrap();
        }
        let grid = Grid::build(&sim.agents);
        let agent = *sim.agent(me).unwrap();
        let n = grid.neighbors(&sim.agents, me, &agent);
        assert_eq!(n.len(), 2);
        assert!(near(n[0].position.x, 1.0, 1e-12) && near(n[1].position.x, 2.0, 1e-12));
    }

    #[test]
    fn route_following_visits_every_waypoint_in_order() {
        // An L-shaped route; a lone agent follows it to the end and stops there.
        let wp = vec![Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let mut route = Route::new(wp.clone());
        let mut sim = Simulator::new(0.1);
        let id = sim.add_agent(Vec2::new(0.0, 0.0), AgentParams::default()).unwrap();
        let mut visited = Vec::new();
        for _ in 0..600 {
            let pos = sim.agent(id).unwrap().position;
            let v = route.preferred_velocity(pos, 1.4, 0.5);
            if visited.last() != Some(&route.next_index()) {
                visited.push(route.next_index());
            }
            sim.set_preferred_velocity(id, v).unwrap();
            sim.step();
        }
        let end = sim.agent(id).unwrap().position;
        assert_eq!(visited, vec![0, 1, 2]);
        assert!(route.arrived(end, 0.5), "ended at {end:?}");
        // Pushed past a waypoint without touching it, the agent still moves on to the next leg.
        let mut r = Route::new(wp);
        r.preferred_velocity(Vec2::new(10.6, 0.8), 1.4, 0.5);
        assert_eq!(r.next_index(), 1);
        assert!(Route::new(Vec::new()).arrived(Vec2::new(5.0, 5.0), 0.5));
    }
}
