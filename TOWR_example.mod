# AMPL rewrite of towr/test/hopper_example.cc.
#
# The model keeps the same structure as the TOWR example:
# - cubic Hermite splines for base position, base Euler angles, foot position,
#   and contact force;
# - fixed monoped contact schedule;
# - flat ground;
# - single rigid body dynamics;
# - terrain, force cone, range-of-motion, swing, acceleration-continuity, and
#   boundary constraints.
#
# TOWR's example has no cost terms by default.  AMPL solvers are happier with an
# objective, so reg_weight can be set to a tiny value, or to 0 for a pure
# feasibility problem.

set DIM := 1..3;       # 1=x/roll, 2=y/pitch, 3=z/yaw
set XY := 1..2;
set BASE_POLYS ordered;
set BASE_NODES ordered;
set EE_POLYS ordered;
set EE_NODES ordered;
set FORCE_POLYS ordered;
set FORCE_NODES ordered;
set DYN_K ordered;
set ROM_K ordered;

param X := 1;
param Y := 2;
param Z := 3;

param eps default 1e-9;
param reg_weight default 1e-8;

param mass default 20.0;
param grav default 9.80665;
param mu default 0.5;
param fn_max default 1000.0;

param nominal{DIM};
param max_dev{DIM};
param inertia{DIM,DIM};

param b_start{BASE_POLYS};
param b_dur{BASE_POLYS};
param e_start{EE_POLYS};
param e_dur{EE_POLYS};
param f_start{FORCE_POLYS};
param f_dur{FORCE_POLYS};

param tdyn{DYN_K};
param trom{ROM_K};

set EE_STANCE_POLYS within EE_POLYS;
set EE_STANCE_NODES within EE_NODES;
set EE_SWING_MID_NODES within EE_NODES;
set FORCE_SWING_NODES within FORCE_NODES;
set FORCE_CONE_NODES within FORCE_NODES;

# For stance phases, TOWR optimizes one foot position and uses it for both
# boundary nodes of the constant stance polynomial.
param ee_stance_owner{EE_STANCE_NODES} in EE_STANCE_POLYS;

var bp{BASE_NODES,DIM};  # base linear node position
var bv{BASE_NODES,DIM};  # base linear node velocity
var ap{BASE_NODES,DIM};  # base Euler node position: roll,pitch,yaw
var av{BASE_NODES,DIM};  # base Euler rates

var ep_stance{EE_STANCE_POLYS,DIM};      # optimized stance footholds
var ep_swing{EE_SWING_MID_NODES,DIM};    # optimized swing midpoint positions
var ev_swing_xy{EE_SWING_MID_NODES,XY};  # optimized swing midpoint xy velocities

var fp_stance{FORCE_CONE_NODES,DIM};     # optimized stance force node values
var fv_stance{FORCE_CONE_NODES,DIM};     # optimized stance force node derivatives

# Full node arrays as defined variables.  These reproduce the phase-based
# NodesVariablesEEMotion / NodesVariablesEEForce parameterization in C++:
# - stance foot positions are shared by adjacent stance nodes;
# - stance foot velocities are zero;
# - swing vertical midpoint velocity is fixed to zero;
# - all swing contact forces are identically zero.
var ep{n in EE_NODES, d in DIM} =
  if n in EE_STANCE_NODES then ep_stance[ee_stance_owner[n],d]
  else ep_swing[n,d];

var ev{n in EE_NODES, d in DIM} =
  if n in EE_SWING_MID_NODES and d in XY then ev_swing_xy[n,d]
  else 0.0;

var fp{n in FORCE_NODES, d in DIM} =
  if n in FORCE_CONE_NODES then fp_stance[n,d]
  else 0.0;

var fv{n in FORCE_NODES, d in DIM} =
  if n in FORCE_CONE_NODES then fv_stance[n,d]
  else 0.0;

# Defined quantities below are not intended as independent decision variables.
# They mirror C++ calls such as spline->GetPoint(t), EulerConverter::GetM(),
# and SingleRigidBodyDynamics' temporary computations.  AMPL substitutes
# defined variables into the constraints, keeping the true free-variable set
# close to TOWR's node variables.

# Cubic Hermite helper expressions expanded inline:
# C = -(3*(p0-p1)+T*(2*v0+v1))/T^2
# D =  (2*(p0-p1)+T*(v0+v1))/T^3

var c{k in DYN_K, d in DIM} =      # sampled COM position
  sum {p in BASE_POLYS:
        ((b_start[p] < tdyn[k] or (p = first(BASE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= b_start[p] + b_dur[p] + eps)}
    (bp[p,d] + bv[p,d]*(tdyn[k]-b_start[p])
     - (3*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(2*bv[p,d]+bv[p+1,d]))
       / b_dur[p]^2 * (tdyn[k]-b_start[p])^2
     + (2*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(bv[p,d]+bv[p+1,d]))
       / b_dur[p]^3 * (tdyn[k]-b_start[p])^3);

var cdd{k in DYN_K, d in DIM} =    # sampled COM acceleration
  sum {p in BASE_POLYS:
        ((b_start[p] < tdyn[k] or (p = first(BASE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= b_start[p] + b_dur[p] + eps)}
    (2 * (-(3*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(2*bv[p,d]+bv[p+1,d]))
           / b_dur[p]^2)
     + 6 * ((2*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(bv[p,d]+bv[p+1,d]))
           / b_dur[p]^3) * (tdyn[k]-b_start[p]));

var ang{k in DYN_K, d in DIM} =    # sampled Euler angles
  sum {p in BASE_POLYS:
        ((b_start[p] < tdyn[k] or (p = first(BASE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= b_start[p] + b_dur[p] + eps)}
    (ap[p,d] + av[p,d]*(tdyn[k]-b_start[p])
     - (3*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(2*av[p,d]+av[p+1,d]))
       / b_dur[p]^2 * (tdyn[k]-b_start[p])^2
     + (2*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(av[p,d]+av[p+1,d]))
       / b_dur[p]^3 * (tdyn[k]-b_start[p])^3);

var erate{k in DYN_K, d in DIM} =  # sampled Euler rates
  sum {p in BASE_POLYS:
        ((b_start[p] < tdyn[k] or (p = first(BASE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= b_start[p] + b_dur[p] + eps)}
    (av[p,d]
     + 2 * (-(3*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(2*av[p,d]+av[p+1,d]))
            / b_dur[p]^2) * (tdyn[k]-b_start[p])
     + 3 * ((2*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(av[p,d]+av[p+1,d]))
            / b_dur[p]^3) * (tdyn[k]-b_start[p])^2);

var eacc{k in DYN_K, d in DIM} =   # sampled Euler accelerations
  sum {p in BASE_POLYS:
        ((b_start[p] < tdyn[k] or (p = first(BASE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= b_start[p] + b_dur[p] + eps)}
    (2 * (-(3*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(2*av[p,d]+av[p+1,d]))
           / b_dur[p]^2)
     + 6 * ((2*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(av[p,d]+av[p+1,d]))
           / b_dur[p]^3) * (tdyn[k]-b_start[p]));

var foot{k in DYN_K, d in DIM} =   # sampled foot position for dynamics
  sum {p in EE_POLYS:
        ((e_start[p] < tdyn[k] or (p = first(EE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= e_start[p] + e_dur[p] + eps)}
    (ep[p,d] + ev[p,d]*(tdyn[k]-e_start[p])
     - (3*(ep[p,d]-ep[p+1,d]) + e_dur[p]*(2*ev[p,d]+ev[p+1,d]))
       / e_dur[p]^2 * (tdyn[k]-e_start[p])^2
     + (2*(ep[p,d]-ep[p+1,d]) + e_dur[p]*(ev[p,d]+ev[p+1,d]))
       / e_dur[p]^3 * (tdyn[k]-e_start[p])^3);

var force{k in DYN_K, d in DIM} =  # sampled contact force for dynamics
  sum {p in FORCE_POLYS:
        ((f_start[p] < tdyn[k] or (p = first(FORCE_POLYS) and tdyn[k] = 0))
         and tdyn[k] <= f_start[p] + f_dur[p] + eps)}
    (fp[p,d] + fv[p,d]*(tdyn[k]-f_start[p])
     - (3*(fp[p,d]-fp[p+1,d]) + f_dur[p]*(2*fv[p,d]+fv[p+1,d]))
       / f_dur[p]^2 * (tdyn[k]-f_start[p])^2
     + (2*(fp[p,d]-fp[p+1,d]) + f_dur[p]*(fv[p,d]+fv[p+1,d]))
       / f_dur[p]^3 * (tdyn[k]-f_start[p])^3);

# Euler ZYX conversion copied from TOWR's EulerConverter.
var omega{k in DYN_K, d in DIM} =
  if d = X then
    -sin(ang[k,Z])*erate[k,Y] + cos(ang[k,Y])*cos(ang[k,Z])*erate[k,X]
  else if d = Y then
     cos(ang[k,Z])*erate[k,Y] + cos(ang[k,Y])*sin(ang[k,Z])*erate[k,X]
  else
    erate[k,Z] - sin(ang[k,Y])*erate[k,X];

var alpha{k in DYN_K, d in DIM} =
  if d = X then
    (-cos(ang[k,Z])*erate[k,Z])*erate[k,Y]
    + (-cos(ang[k,Z])*sin(ang[k,Y])*erate[k,Y]
       - cos(ang[k,Y])*sin(ang[k,Z])*erate[k,Z])*erate[k,X]
    - sin(ang[k,Z])*eacc[k,Y]
    + cos(ang[k,Y])*cos(ang[k,Z])*eacc[k,X]
  else if d = Y then
    (-sin(ang[k,Z])*erate[k,Z])*erate[k,Y]
    + (cos(ang[k,Y])*cos(ang[k,Z])*erate[k,Z]
       - sin(ang[k,Y])*sin(ang[k,Z])*erate[k,Y])*erate[k,X]
    + cos(ang[k,Z])*eacc[k,Y]
    + cos(ang[k,Y])*sin(ang[k,Z])*eacc[k,X]
  else
    -cos(ang[k,Y])*erate[k,Y]*erate[k,X]
    + eacc[k,Z] - sin(ang[k,Y])*eacc[k,X];

var R{k in DYN_K, i in DIM, j in DIM} =
  if i = 1 and j = 1 then cos(ang[k,Y])*cos(ang[k,Z])
  else if i = 1 and j = 2 then cos(ang[k,Z])*sin(ang[k,X])*sin(ang[k,Y]) - cos(ang[k,X])*sin(ang[k,Z])
  else if i = 1 and j = 3 then sin(ang[k,X])*sin(ang[k,Z]) + cos(ang[k,X])*cos(ang[k,Z])*sin(ang[k,Y])
  else if i = 2 and j = 1 then cos(ang[k,Y])*sin(ang[k,Z])
  else if i = 2 and j = 2 then cos(ang[k,X])*cos(ang[k,Z]) + sin(ang[k,X])*sin(ang[k,Y])*sin(ang[k,Z])
  else if i = 2 and j = 3 then cos(ang[k,X])*sin(ang[k,Y])*sin(ang[k,Z]) - cos(ang[k,Z])*sin(ang[k,X])
  else if i = 3 and j = 1 then -sin(ang[k,Y])
  else if i = 3 and j = 2 then cos(ang[k,Y])*sin(ang[k,X])
  else cos(ang[k,X])*cos(ang[k,Y]);

var Iw{k in DYN_K, i in DIM, j in DIM} =
  sum {a in DIM, b in DIM} R[k,i,a] * inertia[a,b] * R[k,j,b];

var h{k in DYN_K, i in DIM} =
  sum {j in DIM} Iw[k,i,j] * omega[k,j];

var Ialpha{k in DYN_K, i in DIM} =
  sum {j in DIM} Iw[k,i,j] * alpha[k,j];

# Single rigid body dynamics: Iw*alpha + omega x (Iw*omega) - f x (com-foot) = 0.
subject to dyn_ang_x{k in DYN_K}:
  Ialpha[k,X] + omega[k,Y]*h[k,Z] - omega[k,Z]*h[k,Y]
  - (force[k,Y]*(c[k,Z]-foot[k,Z]) - force[k,Z]*(c[k,Y]-foot[k,Y])) = 0;
subject to dyn_ang_y{k in DYN_K}:
  Ialpha[k,Y] + omega[k,Z]*h[k,X] - omega[k,X]*h[k,Z]
  - (force[k,Z]*(c[k,X]-foot[k,X]) - force[k,X]*(c[k,Z]-foot[k,Z])) = 0;
subject to dyn_ang_z{k in DYN_K}:
  Ialpha[k,Z] + omega[k,X]*h[k,Y] - omega[k,Y]*h[k,X]
  - (force[k,X]*(c[k,Y]-foot[k,Y]) - force[k,Y]*(c[k,X]-foot[k,X])) = 0;
subject to dyn_lin_x{k in DYN_K}:
  mass*cdd[k,X] - force[k,X] = 0;
subject to dyn_lin_y{k in DYN_K}:
  mass*cdd[k,Y] - force[k,Y] = 0;
subject to dyn_lin_z{k in DYN_K}:
  mass*cdd[k,Z] - force[k,Z] + mass*grav = 0;

# Boundary constraints from hopper_example.cc and NlpFormulation defaults.
subject to base_initial_pos{d in DIM}: bp[first(BASE_NODES),d] = if d = Z then 0.5 else 0.0;
subject to base_initial_vel{d in DIM}: bv[first(BASE_NODES),d] = 0.0;
subject to ang_initial_pos{d in DIM}: ap[first(BASE_NODES),d] = 0.0;
subject to ang_initial_vel{d in DIM}: av[first(BASE_NODES),d] = 0.0;

subject to base_final_x: bp[last(BASE_NODES),X] = 1.0;
subject to base_final_y: bp[last(BASE_NODES),Y] = 0.0;
subject to base_final_vel{d in DIM}: bv[last(BASE_NODES),d] = 0.0;
subject to ang_final_pos{d in DIM}: ap[last(BASE_NODES),d] = 0.0;
subject to ang_final_vel{d in DIM}: av[last(BASE_NODES),d] = 0.0;

subject to foot_initial{d in DIM}: ep[first(EE_NODES),d] = 0.0;

# TerrainConstraint in C++ skips the first node because the initial foot
# position is already fixed.  Constant/contact nodes must be exactly on the
# terrain; swing midpoint nodes may be above it.
subject to terrain_stance_nodes{n in EE_STANCE_NODES: n != first(EE_NODES)}:
  ep[n,Z] = 0.0;
subject to terrain_swing_nodes{n in EE_SWING_MID_NODES}:
  ep[n,Z] >= 0.0;

# Force constraint for flat ground: n=(0,0,1), t1=(1,0,0), t2=(0,1,0).
subject to normal_force_lb{n in FORCE_CONE_NODES}: fp[n,Z] >= 0.0;
subject to normal_force_ub{n in FORCE_CONE_NODES}: fp[n,Z] <= fn_max;
subject to friction_px{n in FORCE_CONE_NODES}: fp[n,X] - mu*fp[n,Z] <= 0.0;
subject to friction_nx{n in FORCE_CONE_NODES}: fp[n,X] + mu*fp[n,Z] >= 0.0;
subject to friction_py{n in FORCE_CONE_NODES}: fp[n,Y] - mu*fp[n,Z] <= 0.0;
subject to friction_ny{n in FORCE_CONE_NODES}: fp[n,Y] + mu*fp[n,Z] >= 0.0;

# Base acceleration continuity, matching SplineAccConstraint.
subject to base_acc_continuity{p in BASE_POLYS, d in DIM: p < last(BASE_POLYS)}:
  (2 * (-(3*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(2*bv[p,d]+bv[p+1,d])) / b_dur[p]^2)
   + 6 * ((2*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(bv[p,d]+bv[p+1,d])) / b_dur[p]^3) * b_dur[p])
  =
  (2 * (-(3*(bp[p+1,d]-bp[p+2,d]) + b_dur[p+1]*(2*bv[p+1,d]+bv[p+2,d])) / b_dur[p+1]^2));

subject to ang_acc_continuity{p in BASE_POLYS, d in DIM: p < last(BASE_POLYS)}:
  (2 * (-(3*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(2*av[p,d]+av[p+1,d])) / b_dur[p]^2)
   + 6 * ((2*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(av[p,d]+av[p+1,d])) / b_dur[p]^3) * b_dur[p])
  =
  (2 * (-(3*(ap[p+1,d]-ap[p+2,d]) + b_dur[p+1]*(2*av[p+1,d]+av[p+2,d])) / b_dur[p+1]^2));

# ROM samples are also defined quantities, just like the C++ constraint code
# querying splines at dt_constraint_range_of_motion_.
var c_rom{k in ROM_K, d in DIM} =
  sum {p in BASE_POLYS:
        ((b_start[p] < trom[k] or (p = first(BASE_POLYS) and trom[k] = 0))
         and trom[k] <= b_start[p] + b_dur[p] + eps)}
    (bp[p,d] + bv[p,d]*(trom[k]-b_start[p])
     - (3*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(2*bv[p,d]+bv[p+1,d]))
       / b_dur[p]^2 * (trom[k]-b_start[p])^2
     + (2*(bp[p,d]-bp[p+1,d]) + b_dur[p]*(bv[p,d]+bv[p+1,d]))
       / b_dur[p]^3 * (trom[k]-b_start[p])^3);

var ang_rom{k in ROM_K, d in DIM} =
  sum {p in BASE_POLYS:
        ((b_start[p] < trom[k] or (p = first(BASE_POLYS) and trom[k] = 0))
         and trom[k] <= b_start[p] + b_dur[p] + eps)}
    (ap[p,d] + av[p,d]*(trom[k]-b_start[p])
     - (3*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(2*av[p,d]+av[p+1,d]))
       / b_dur[p]^2 * (trom[k]-b_start[p])^2
     + (2*(ap[p,d]-ap[p+1,d]) + b_dur[p]*(av[p,d]+av[p+1,d]))
       / b_dur[p]^3 * (trom[k]-b_start[p])^3);

var foot_rom{k in ROM_K, d in DIM} =
  sum {p in EE_POLYS:
        ((e_start[p] < trom[k] or (p = first(EE_POLYS) and trom[k] = 0))
         and trom[k] <= e_start[p] + e_dur[p] + eps)}
    (ep[p,d] + ev[p,d]*(trom[k]-e_start[p])
     - (3*(ep[p,d]-ep[p+1,d]) + e_dur[p]*(2*ev[p,d]+ev[p+1,d]))
       / e_dur[p]^2 * (trom[k]-e_start[p])^2
     + (2*(ep[p,d]-ep[p+1,d]) + e_dur[p]*(ev[p,d]+ev[p+1,d]))
       / e_dur[p]^3 * (trom[k]-e_start[p])^3);

var Rrom{k in ROM_K, i in DIM, j in DIM} =
  if i = 1 and j = 1 then cos(ang_rom[k,Y])*cos(ang_rom[k,Z])
  else if i = 1 and j = 2 then cos(ang_rom[k,Z])*sin(ang_rom[k,X])*sin(ang_rom[k,Y]) - cos(ang_rom[k,X])*sin(ang_rom[k,Z])
  else if i = 1 and j = 3 then sin(ang_rom[k,X])*sin(ang_rom[k,Z]) + cos(ang_rom[k,X])*cos(ang_rom[k,Z])*sin(ang_rom[k,Y])
  else if i = 2 and j = 1 then cos(ang_rom[k,Y])*sin(ang_rom[k,Z])
  else if i = 2 and j = 2 then cos(ang_rom[k,X])*cos(ang_rom[k,Z]) + sin(ang_rom[k,X])*sin(ang_rom[k,Y])*sin(ang_rom[k,Z])
  else if i = 2 and j = 3 then cos(ang_rom[k,X])*sin(ang_rom[k,Y])*sin(ang_rom[k,Z]) - cos(ang_rom[k,Z])*sin(ang_rom[k,X])
  else if i = 3 and j = 1 then -sin(ang_rom[k,Y])
  else if i = 3 and j = 2 then cos(ang_rom[k,Y])*sin(ang_rom[k,X])
  else cos(ang_rom[k,X])*cos(ang_rom[k,Y]);

var relB{k in ROM_K, d in DIM} =
  sum {j in DIM} Rrom[k,j,d] * (foot_rom[k,j] - c_rom[k,j]);

subject to rom_bounds{k in ROM_K, d in DIM}:
  nominal[d] - max_dev[d] <= relB[k,d] <= nominal[d] + max_dev[d];

# SwingConstraint: internal swing node xy is midpoint, xy velocity is
# endpoint-to-endpoint displacement divided by the swing duration.
subject to swing_midpoint_x:
  ep[2,X] = 0.5*(ep[1,X] + ep[3,X]);
subject to swing_midpoint_y:
  ep[2,Y] = 0.5*(ep[1,Y] + ep[3,Y]);
subject to swing_velocity_x:
  ev[2,X] = (ep[3,X] - ep[1,X]) / 0.2;
subject to swing_velocity_y:
  ev[2,Y] = (ep[3,Y] - ep[1,Y]) / 0.2;
subject to swing2_midpoint_x:
  ep[5,X] = 0.5*(ep[4,X] + ep[6,X]);
subject to swing2_midpoint_y:
  ep[5,Y] = 0.5*(ep[4,Y] + ep[6,Y]);
subject to swing2_velocity_x:
  ev[5,X] = (ep[6,X] - ep[4,X]) / 0.2;
subject to swing2_velocity_y:
  ev[5,Y] = (ep[6,Y] - ep[4,Y]) / 0.2;
subject to swing3_midpoint_x:
  ep[8,X] = 0.5*(ep[7,X] + ep[9,X]);
subject to swing3_midpoint_y:
  ep[8,Y] = 0.5*(ep[7,Y] + ep[9,Y]);
subject to swing3_velocity_x:
  ev[8,X] = (ep[9,X] - ep[7,X]) / 0.2;
subject to swing3_velocity_y:
  ev[8,Y] = (ep[9,Y] - ep[7,Y]) / 0.2;

minimize towrlike_objective:
  reg_weight * (
    sum {n in BASE_NODES, d in DIM} (bp[n,d]^2 + bv[n,d]^2 + ap[n,d]^2 + av[n,d]^2)
  + sum {n in EE_NODES, d in DIM} (ep[n,d]^2 + ev[n,d]^2)
  + sum {n in FORCE_NODES, d in DIM} (fp[n,d]^2 + fv[n,d]^2)
  );
