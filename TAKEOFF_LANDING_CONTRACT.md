# Minimum global-planner takeoff and landing contract

Status: geometric planning only. This contract defines information and validation needed
before adding physical operation edges; it does not supply missing robot limits or certify
execution. No takeoff or landing edges are implemented.

## Evidence and current supported behavior

Sources are `test_data/scenarios.yaml` (`meta.rules`, `test_parameters`, T01–T06 and
`pending_changes`), `representation/fly_validator.py`, `walk_validator.py`, `anchors.py`
and the stage notes in `representation_comparison.md`. No separate calibrated robot or
controller requirements were found in this repository. YAML explicitly calls the body
sizes and penalties test values and lists takeoff/landing constraints as pending.

| Topic | Documented geometric test behavior | Information still needed from robot/controller owners |
|---|---|---|
| Pose | WALK x/y is body center, z is ground contact, heading in radians; FLY x/y/z is box bottom center, yaw absent. Map coordinates and dimensions use meters. | Frame IDs, transform ownership, real body/reference-point offsets, attitude/yaw representation, localization tolerances and landing heading rules. |
| Clearance | Shared `BoxBody` checks whole straight sweeps against complete obstacle boxes and world bounds, with configured margin/contact tolerance. Ground is flat and known; unknown/non-flat maps are rejected. | Calibrated body/rotor/support envelopes through liftoff/touchdown, landing patch support/size/slope, approach clearance, uncertainty margins and permitted floor/contact behavior. |
| Motion | FLY permits straight translation with test altitude and per-edge distance limits. Ground contact is geometrically allowed. Stationary morphing has zero displacement and a separate enclosing swept box. | Permitted takeoff/landing trajectories and attitude evolution; relative/absolute height limits, lateral displacement, approach/descent/climb limits, velocities, acceleration, duration, terminal tolerances and controller tracking guarantees. |
| Direction | Morphs separately support configured WALK→FLY and FLY→WALK at exact shared positions. Geometric FLY translation is not a physical operation label. | Which physical directions and sites are supported, phase/readiness conditions, controller availability, abort/recovery and terminal-state guarantees. |
| Cost | WALK distance/terrain/reverse/pivot costs, FLY distance rate and per-morph `transition_extra_cost` are nonnegative synthetic costs. | Objective units/normalization and separate takeoff/landing time, energy or operation penalties, plus whether expected failure/recovery costs belong in the objective. |

## Minimum edge contract before implementation

1. **Explicit operation and endpoint phases.** Takeoff connects a valid, supported,
   flight-configured ground pose to an airborne FLY pose; landing connects airborne FLY
   to a flight-configured ground pose. Neither directly changes WALK mode. Keep the sequence
   WALK→stationary morph→ground FLY→takeoff→airborne FLY, and the reverse sequence
   airborne FLY→landing→ground FLY→stationary morph→WALK. Existing `MotionMode.FLY` does
   not encode grounded/airborne phase, readiness or controller state. Future phase/operation
   representation must preserve these distinctions and prevent bypass through ordinary
   translations. This sequencing is the proposed minimum interface, not measured capability.
2. **Pose and clearance agreement.** Specify the common planning frame, reference point,
   heading/attitude, units and endpoint/contact tolerances. Validate both endpoint bodies,
   the entire operation envelope and required ground support/clearance. Reuse shared box
   geometry where its conservative envelope applies; attitude-dependent or curved operations
   require a supplied appropriate swept envelope/validator. A clear chord alone is insufficient.
3. **Controller-supplied motion limits.** Require explicit supported trajectories, preconditions,
   bounds and terminal guarantees for each direction before accepting an edge. Do not interpret
   generic FLY test altitude/distance settings as takeoff/landing constraints, assume a vertical
   trajectory, or invent rates, heights or timing. Missing data leaves the operation unsupported.
4. **Directed validation.** Validate takeoff and landing independently against their own
   preconditions, envelopes and controller guarantees; never automatically insert a reciprocal
   physical edge. Surface eligibility may differ from WALK feasibility. A supported site must
   meet landing support requirements as well as flight and morphing geometry where applicable.
5. **Separate configured costs.** Price each operation independently and nonnegatively in the
   same objective as other edges. Keep stationary morph penalties separate and define whether
   operation cost includes displacement to avoid double counting. Recompute route totals from
   all movement, morph and operation edges. Recheck the A* heuristic against added costs;
   an operation whose cost does not cover the distance-rate lower bound needs a weaker heuristic
   (possibly zero). Do not reuse `transition_extra_cost` as a physical operation cost.
6. **Execution boundary.** Planner geometric feasibility is insufficient for controller handoff.
   Physical support requires validated operation types, pose agreement, calibrated limits and
   explicit controller acceptance/preconditions, with stale-map and failure behavior defined.
   Even an all-WALK test route is not certified executable by this package.

## Enforced public behavior now

`joint_planner.execution_policy` supports only `geometric_only` (also the fallback for
existing callers). `JointPlanner.plan()` returns `route_kind='geometric'` and
`executable=False` on success and no-path results. `plan(require_execution=True)` raises
`NotImplementedError` before searching, including WALK-only requests. Geometry-only
success, optimality and cost validity do not establish physical execution.

`FlyStateValidator.is_edge_valid(..., motion='takeoff'/'landing')` rejects the operation,
and `TransitionValidator` rejects either name as a morph direction. Both validators reject
configuration that tries to enable an unimplemented takeoff/landing policy. A floor-to-air
straight translation can pass geometric sweep validation while the explicitly named physical
operation remains unsupported; this distinction is tested. Graph edges currently carry no
physical controller command. Downstream consumers must preserve the geometric result labels
and must not treat untyped FLY edges as takeoff/landing approval.

Pending robot/controller answers are the third column above. Until they are supplied and
validated, no physical operation limits, connections or costs will be added. The existing
finite geometric graph and cost audits remain a reference for later implementation.
