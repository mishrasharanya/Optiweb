#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* grid
#----------- modules
# model-defined modules;
set MOD_MODULES;
set MOD_GRID {MOD_MODULES}; # this set will be redeclared later

# data-defined modules
set DAT_MODULES default {};
set DAT_GRID {DAT_MODULES};

# all modules
set MODULES = MOD_MODULES union DAT_MODULES;
set GRID_MODULES {i1 in MODULES} := 
  if i1 in MOD_MODULES then MOD_GRID[i1]  
  else DAT_GRID[i1];

#----------- phases
# dynamics are constant within a phase
set SUBPHASES within Reals;
set SUBPHASES_MBRS = {'PHASE', 'BEG', 'END'};
param subphases {SUBPHASES, SUBPHASES_MBRS};

# subphases must map to phase through floor function
check {i1 in SUBPHASES} : subphases[i1, 'PHASE'] = floor(i1);
# beginning must be less than end
check {i1 in SUBPHASES} : subphases[i1, 'BEG'] <= subphases[i1, 'END'];
# make sure sets don't overlap
check {i1 in SUBPHASES, i2 in SUBPHASES : i1 != i2} : inter {i3 in {i1, i2}} union {i4 in {'BEG', 'END'}} {subphases[i3, i4]} within {};

set PHASES = setof {i1 in SUBPHASES} subphases[i1, 'PHASE'];

#----------- grid points
param GRID_MAX >= 0;

set GRID_MBRS = {0} union SUBPHASES union PHASES;

set GRID {i1 in GRID_MBRS} ordered by [0, GRID_MAX] :=
  if i1 = 0 then # add grid points from registered modules
    union {i2 in MODULES} GRID_MODULES[i2]
  else if i1 in Integers then # collect grid points for each phase
    union {i2 in SUBPHASES : floor(i2) = i1} GRID[i2]
  else {i2 in GRID[0] : subphases[i1, 'BEG'] <= i2 <= subphases[i1, 'END']};  # collect grid points for each subphase

#----------- decision variable: time
var t {GRID[0]}; # time
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* grid!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* bezier
# Resources:
# https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html

set BEZ_LBLS;
set BEZ_SUBGRID {BEZ_LBLS};
set BEZ_MBRS := {'DER', 'CP_ROW', 'CP_COL'};
set BEZ_SUBGRID_MBRS := {'BEG', 'END'};

param bez {BEZ_LBLS, BEZ_MBRS};
param bez_subgrid {i1 in BEZ_LBLS, BEZ_SUBGRID[i1], BEZ_SUBGRID_MBRS};

set BEZ {i1 in BEZ_LBLS, i2 in BEZ_MBRS} :=
  if i2 in {'DER', 'DEG'} then 0..bez[i1, i2]
  else if i2 in {'CP_ROW', 'CP_COL'} then 0..bez[i1, i2]-1
  else {};

set BEZ_GRID {i1 in BEZ_LBLS, i2 in BEZ_SUBGRID[i1]} = GRID[0] inter interval [bez_subgrid[i1, i2, 'BEG'], bez_subgrid[i1, i2, 'END']];

# decision variables: bezier knots and control points
var bez_tau {i1 in BEZ_LBLS, i2 in BEZ_SUBGRID[i1], BEZ_GRID[i1, i2]} >= 0, <= 1;
var bez_ctls {i1 in BEZ_LBLS, BEZ_SUBGRID[i1], BEZ[i1, 'CP_ROW'], BEZ[i1, 'CP_COL']};

set BEZ_N := setof {i1 in BEZ_LBLS} bez[i1, 'CP_COL'] - 1;
set BEZ_COL {i1 in BEZ_N} := 0..i1;

param bez_B {i1 in BEZ_N, i2 in BEZ_COL[i1], i3 in BEZ_COL[i1]: i3 <= i2} = (-1)^(i2 - i3) * prod {i4 in BEZ_COL[i1]: i4 <= i2 - 1} (i1 - i4) / ((i3 - min(i4, i3 - 1)) * (i2 - i3 - min(i4, i2 - i3 - 1)));

var bez_curve {i1 in BEZ_LBLS, i2 in BEZ_SUBGRID[i1], i3 in BEZ[i1, 'DER'], i4 in BEZ[i1, 'CP_ROW'], i5 in BEZ_GRID[i1, i2]} = sum {i6 in BEZ_COL[bez[i1, 'CP_COL'] - 1], i7 in BEZ_COL[bez[i1, 'CP_COL'] - 1]: i3 <= i6 and i7 <= i6} ((prod {i8 in BEZ[i1, 'DER']: i8 < i3} (i6 - i8)) * bez_tau[i1, i2, i5]^(i6 - i3) * bez_B[bez[i1, 'CP_COL'] - 1, i6, i7] * bez_ctls[i1, i2, i4, i7]);
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* bez!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* state
param nq >= 0 integer;
set Q = 1..nq; # config space
set X = 1..2*nq; # state space

#----------- decision variables: generalized coordinates and forces
# state variables
var q {Q, GRID[0]};
var v {Q, GRID[0]}; # v = dq/dt
var a {Q, GRID[0]}; # a = d^2q/dt^2

# control input
var u {Q, GRID[0]};

# defined state variables
var x {i1 in X, i2 in GRID[0]} =
  if i1 <= nq then q[i1, i2]
  else v[i1-nq, i2];
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* state!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* robot
set RBT_IDX  ordered = {'PARENT', 'S_RX', 'S_RY', 'S_RZ', 'S_PX', 'S_PY', 'S_PZ', 'Q_LB', 'Q_UB', 'F_RX', 'F_RY', 'F_RZ', 'F_PX', 'F_PY', 'F_PZ', 'F_TH', 'MASS', 'M_RX', 'M_RY', 'M_RZ', 'M_PX', 'M_PY', 'M_PZ', 'M_TH', 'IXX', 'IYY', 'IZZ', 'IXY', 'IXZ', 'IYZ'};
param robot {Q, RBT_IDX};

#----------- kinematic tree
# -1 = leaves to base, 0 = base to leaves, 1 = 1 to leaves
set SPAT_LINK_MBRS := {-1, 0, 1};

set SPAT_L {i1 in SPAT_LINK_MBRS} ordered :=
  if i1 = -1 then nq..0 by -1
  else i1..nq;

# P = parent, C = child, S = subtree, K = path to base
set SPAT_TREE_P {i1 in SPAT_L[0]} ordered = if i1 in SPAT_L[1] then {robot[i1, 'PARENT']} else {};
set SPAT_TREE_C {i1 in SPAT_L[0]} ordered = setof {i2 in SPAT_L[1], i3 in SPAT_TREE_P[i2]: i1 = i3} i2;
set SPAT_TREE_S {i1 in SPAT_L[-1]} ordered = {i1} union (union {i2 in SPAT_TREE_C[i1]} SPAT_TREE_S[i2]);
set SPAT_TREE_K {i1 in SPAT_L[0]} ordered = {i1} union (union {i2 in SPAT_TREE_P[i1]: i1 in SPAT_L[1]} SPAT_TREE_K[i2]);
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* robot!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* constraints
param nr >= 0 integer; # total number of constraints across all phases
set R = 1..nr; # constraints

set CON_IDX  ordered = {'PHASE', 'TYPE', 'ROW', 'BODY', 'R_RX', 'R_RY', 'R_RZ', 'R_PX', 'R_PY', 'R_PZ', 'F_RX', 'F_RY', 'F_RZ', 'F_PX', 'F_PY', 'F_PZ', 'F_TH', 'F_O'};
param constraint {R, CON_IDX} symbolic;

#----------- indices for physical and virtual constraints
# P = physical constraint or force, V = virtual constraint, U = virtual force
# M = constrained motions, F = constraint forces
set CON_P {i1 in PHASES} = setof {i2 in R: constraint[i2, 'PHASE'] = i1 and constraint[i2, 'TYPE'] = 'p'} constraint[i2, 'ROW'];
set CON_V {i1 in PHASES} = setof {i2 in R: constraint[i2, 'PHASE'] = i1 and constraint[i2, 'TYPE'] = 'v'} constraint[i2, 'ROW'];
set CON_U {i1 in PHASES} = setof {i2 in R: constraint[i2, 'PHASE'] = i1 and constraint[i2, 'TYPE'] = 'u'} constraint[i2, 'ROW'];

#----------- indices for physical and virtual constraints
# M = constrained motions, F = constraint forces
set CON_M {i1 in PHASES} = CON_P[i1] union CON_V[i1];
set CON_F {i1 in PHASES} = CON_P[i1] union CON_U[i1];
set CON_J {i1 in PHASES} = CON_M[i1] union CON_F[i1];

# mapping from i to k in J[p, i, j] = sum J[p, k[i], j] in phase p
set CON_R {i1 in PHASES, i2 in CON_J[i1]} := setof {i3 in R: constraint[i3, 'PHASE'] = i1 and constraint[i3, 'ROW'] = i2} i3;
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* constraints!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* spatial transforms
param nm = 6;
set SPAT_M = 1..nm;

#----------- all transforms in robot and constraints
set SPAT_EXPC := 1..nm+1;

# constant transforms
set SPAT_XP := 1..(2 * nq + nr);

param x_jp := 1; # RBT_F
param x_im := nq + 1; # RBT_M
param x_cb := 2*nq + 1; # CON_F

param spat_expcp {i1 in SPAT_XP, i2 in SPAT_EXPC} :=
  if x_jp <= i1 < x_im then robot[i1 - x_jp + 1, next('F_RX', RBT_IDX, i2 - 1)]
  else if x_im <= i1 < x_cb then robot[i1 - x_im + 1, next('M_RX', RBT_IDX, i2 - 1)]
  else constraint[i1 - x_cb + 1, next('F_RX', CON_IDX, i2 - 1)];

param Ep {i1 in SPAT_XP, i2 in SPAT_M, i3 in SPAT_M: i2 <= 3 and i3 <= 3} =
  if i2 = 1 and i3 = 1 then 1 + (1 - cos(spat_expcp[i1, 7]))*(-spat_expcp[i1, 2]^2 - spat_expcp[i1, 3]^2)
  else if i2 = 1 and i3 = 2 then (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 1]*spat_expcp[i1, 2] + sin(spat_expcp[i1, 7])*spat_expcp[i1, 3]
  else if i2 = 1 and i3 = 3 then -(sin(spat_expcp[i1, 7])*spat_expcp[i1, 2]) + (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 1]*spat_expcp[i1, 3]
  else if i2 = 2 and i3 = 1 then (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 1]*spat_expcp[i1, 2] - sin(spat_expcp[i1, 7])*spat_expcp[i1, 3]
  else if i2 = 2 and i3 = 2 then 1 + (1 - cos(spat_expcp[i1, 7]))*(-spat_expcp[i1, 1]^2 - spat_expcp[i1, 3]^2)
  else if i2 = 2 and i3 = 3 then sin(spat_expcp[i1, 7])*spat_expcp[i1, 1] + (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 2]*spat_expcp[i1, 3]
  else if i2 = 3 and i3 = 1 then sin(spat_expcp[i1, 7])*spat_expcp[i1, 2] + (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 1]*spat_expcp[i1, 3]
  else if i2 = 3 and i3 = 2 then -(sin(spat_expcp[i1, 7])*spat_expcp[i1, 1]) + (1 - cos(spat_expcp[i1, 7]))*spat_expcp[i1, 2]*spat_expcp[i1, 3]
  else if i2 = 3 and i3 = 3 then 1 + (1 - cos(spat_expcp[i1, 7]))*(-spat_expcp[i1, 1]^2 - spat_expcp[i1, 2]^2);

# px = (p) x = skew-symmetric matrix of p = (-E r) x
param ppx {i1 in SPAT_XP, i2 in SPAT_M, i3 in SPAT_M: i2 <= 3 and i3 <= 3} =
  if i2 = 1 and i3 = 2 then -((1 - cos(spat_expcp[i1, 7]))*(-(spat_expcp[i1, 2]*spat_expcp[i1, 4]) + spat_expcp[i1, 1]*spat_expcp[i1, 5])) + spat_expcp[i1, 6]*spat_expcp[i1, 7] - (-(spat_expcp[i1, 1]*spat_expcp[i1, 3]*spat_expcp[i1, 4]) - spat_expcp[i1, 2]*spat_expcp[i1, 3]*spat_expcp[i1, 5] - (-spat_expcp[i1, 1]^2 - spat_expcp[i1, 2]^2)*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7])
  else if i2 = 1 and i3 = 3 then (1 - cos(spat_expcp[i1, 7]))*(spat_expcp[i1, 3]*spat_expcp[i1, 4] - spat_expcp[i1, 1]*spat_expcp[i1, 6]) - spat_expcp[i1, 5]*spat_expcp[i1, 7] + (-(spat_expcp[i1, 1]*spat_expcp[i1, 2]*spat_expcp[i1, 4]) - (-spat_expcp[i1, 1]^2 - spat_expcp[i1, 3]^2)*spat_expcp[i1, 5] - spat_expcp[i1, 2]*spat_expcp[i1, 3]*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7])
  else if i2 = 2 and i3 = 1 then (1 - cos(spat_expcp[i1, 7]))*(-(spat_expcp[i1, 2]*spat_expcp[i1, 4]) + spat_expcp[i1, 1]*spat_expcp[i1, 5]) - spat_expcp[i1, 6]*spat_expcp[i1, 7] + (-(spat_expcp[i1, 1]*spat_expcp[i1, 3]*spat_expcp[i1, 4]) - spat_expcp[i1, 2]*spat_expcp[i1, 3]*spat_expcp[i1, 5] - (-spat_expcp[i1, 1]^2 - spat_expcp[i1, 2]^2)*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7])
  else if i2 = 2 and i3 = 3 then -((1 - cos(spat_expcp[i1, 7]))*(-(spat_expcp[i1, 3]*spat_expcp[i1, 5]) + spat_expcp[i1, 2]*spat_expcp[i1, 6])) + spat_expcp[i1, 4]*spat_expcp[i1, 7] - (-((-spat_expcp[i1, 2]^2 - spat_expcp[i1, 3]^2)*spat_expcp[i1, 4]) - spat_expcp[i1, 1]*spat_expcp[i1, 2]*spat_expcp[i1, 5] - spat_expcp[i1, 1]*spat_expcp[i1, 3]*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7])
  else if i2 = 3 and i3 = 1 then -((1 - cos(spat_expcp[i1, 7]))*(spat_expcp[i1, 3]*spat_expcp[i1, 4] - spat_expcp[i1, 1]*spat_expcp[i1, 6])) + spat_expcp[i1, 5]*spat_expcp[i1, 7] - (-(spat_expcp[i1, 1]*spat_expcp[i1, 2]*spat_expcp[i1, 4]) - (-spat_expcp[i1, 1]^2 - spat_expcp[i1, 3]^2)*spat_expcp[i1, 5] - spat_expcp[i1, 2]*spat_expcp[i1, 3]*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7])
  else if i2 = 3 and i3 = 2 then (1 - cos(spat_expcp[i1, 7]))*(-(spat_expcp[i1, 3]*spat_expcp[i1, 5]) + spat_expcp[i1, 2]*spat_expcp[i1, 6]) - spat_expcp[i1, 4]*spat_expcp[i1, 7] + (-((-spat_expcp[i1, 2]^2 - spat_expcp[i1, 3]^2)*spat_expcp[i1, 4]) - spat_expcp[i1, 1]*spat_expcp[i1, 2]*spat_expcp[i1, 5] - spat_expcp[i1, 1]*spat_expcp[i1, 3]*spat_expcp[i1, 6])*(-sin(spat_expcp[i1, 7]) + spat_expcp[i1, 7]);

# p x E = skew[-E.r].E = -E.skew[r].E^T.E = -E.skew[r]
param spat_Xp {i1 in SPAT_XP, i2 in SPAT_M, i3 in SPAT_M} =
  if i2 <= 3 and i3 <= 3 then Ep[i1, i2, i3]
  else if i2 > 3 and i3 > 3 then Ep[i1, i2 - 3, i3 - 3]
  else if 3 < i2 <= 6 and i3 <= 3 then sum {i4 in SPAT_M: i4 <= 3} ppx[i1, i2 - 3, i4] * Ep[i1, i4, i3];

#----------- variable transforms
set SPAT_XV := 1..nq;

param x_ij := 1; # RBT_S

param spat_expcv {i1 in SPAT_XV, i2 in SPAT_M} :=
  robot[i1, next('S_RX', RBT_IDX, i2 - 1)];

var Ev {i1 in SPAT_XV, i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0]: i2 <= 3 and i3 <= 3} =
  if i2 = 1 and i3 = 1 then 1 + (1 - cos(q[i1, i4]))*(-spat_expcv[i1, 2]^2 - spat_expcv[i1, 3]^2)
  else if i2 = 1 and i3 = 2 then (1 - cos(q[i1, i4]))*spat_expcv[i1, 1]*spat_expcv[i1, 2] + sin(q[i1, i4])*spat_expcv[i1, 3]
  else if i2 = 1 and i3 = 3 then -(sin(q[i1, i4])*spat_expcv[i1, 2]) + (1 - cos(q[i1, i4]))*spat_expcv[i1, 1]*spat_expcv[i1, 3]
  else if i2 = 2 and i3 = 1 then (1 - cos(q[i1, i4]))*spat_expcv[i1, 1]*spat_expcv[i1, 2] - sin(q[i1, i4])*spat_expcv[i1, 3]
  else if i2 = 2 and i3 = 2 then 1 + (1 - cos(q[i1, i4]))*(-spat_expcv[i1, 1]^2 - spat_expcv[i1, 3]^2)
  else if i2 = 2 and i3 = 3 then sin(q[i1, i4])*spat_expcv[i1, 1] + (1 - cos(q[i1, i4]))*spat_expcv[i1, 2]*spat_expcv[i1, 3]
  else if i2 = 3 and i3 = 1 then sin(q[i1, i4])*spat_expcv[i1, 2] + (1 - cos(q[i1, i4]))*spat_expcv[i1, 1]*spat_expcv[i1, 3]
  else if i2 = 3 and i3 = 2 then -(sin(q[i1, i4])*spat_expcv[i1, 1]) + (1 - cos(q[i1, i4]))*spat_expcv[i1, 2]*spat_expcv[i1, 3]
  else if i2 = 3 and i3 = 3 then 1 + (1 - cos(q[i1, i4]))*(-spat_expcv[i1, 1]^2 - spat_expcv[i1, 2]^2);

# px = (p) x = skew-symmetric matrix of p = (-E r) x
var pvx {i1 in SPAT_XV, i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0]: i2 <= 3 and i3 <= 3} =
  if i2 = 1 and i3 = 2 then -((1 - cos(q[i1, i4]))*(-(spat_expcv[i1, 2]*spat_expcv[i1, 4]) + spat_expcv[i1, 1]*spat_expcv[i1, 5])) + spat_expcv[i1, 6]*q[i1, i4] - (-(spat_expcv[i1, 1]*spat_expcv[i1, 3]*spat_expcv[i1, 4]) - spat_expcv[i1, 2]*spat_expcv[i1, 3]*spat_expcv[i1, 5] - (-spat_expcv[i1, 1]^2 - spat_expcv[i1, 2]^2)*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4])
  else if i2 = 1 and i3 = 3 then (1 - cos(q[i1, i4]))*(spat_expcv[i1, 3]*spat_expcv[i1, 4] - spat_expcv[i1, 1]*spat_expcv[i1, 6]) - spat_expcv[i1, 5]*q[i1, i4] + (-(spat_expcv[i1, 1]*spat_expcv[i1, 2]*spat_expcv[i1, 4]) - (-spat_expcv[i1, 1]^2 - spat_expcv[i1, 3]^2)*spat_expcv[i1, 5] - spat_expcv[i1, 2]*spat_expcv[i1, 3]*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4])
  else if i2 = 2 and i3 = 1 then (1 - cos(q[i1, i4]))*(-(spat_expcv[i1, 2]*spat_expcv[i1, 4]) + spat_expcv[i1, 1]*spat_expcv[i1, 5]) - spat_expcv[i1, 6]*q[i1, i4] + (-(spat_expcv[i1, 1]*spat_expcv[i1, 3]*spat_expcv[i1, 4]) - spat_expcv[i1, 2]*spat_expcv[i1, 3]*spat_expcv[i1, 5] - (-spat_expcv[i1, 1]^2 - spat_expcv[i1, 2]^2)*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4])
  else if i2 = 2 and i3 = 3 then -((1 - cos(q[i1, i4]))*(-(spat_expcv[i1, 3]*spat_expcv[i1, 5]) + spat_expcv[i1, 2]*spat_expcv[i1, 6])) + spat_expcv[i1, 4]*q[i1, i4] - (-((-spat_expcv[i1, 2]^2 - spat_expcv[i1, 3]^2)*spat_expcv[i1, 4]) - spat_expcv[i1, 1]*spat_expcv[i1, 2]*spat_expcv[i1, 5] - spat_expcv[i1, 1]*spat_expcv[i1, 3]*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4])
  else if i2 = 3 and i3 = 1 then -((1 - cos(q[i1, i4]))*(spat_expcv[i1, 3]*spat_expcv[i1, 4] - spat_expcv[i1, 1]*spat_expcv[i1, 6])) + spat_expcv[i1, 5]*q[i1, i4] - (-(spat_expcv[i1, 1]*spat_expcv[i1, 2]*spat_expcv[i1, 4]) - (-spat_expcv[i1, 1]^2 - spat_expcv[i1, 3]^2)*spat_expcv[i1, 5] - spat_expcv[i1, 2]*spat_expcv[i1, 3]*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4])
  else if i2 = 3 and i3 = 2 then (1 - cos(q[i1, i4]))*(-(spat_expcv[i1, 3]*spat_expcv[i1, 5]) + spat_expcv[i1, 2]*spat_expcv[i1, 6]) - spat_expcv[i1, 4]*q[i1, i4] + (-((-spat_expcv[i1, 2]^2 - spat_expcv[i1, 3]^2)*spat_expcv[i1, 4]) - spat_expcv[i1, 1]*spat_expcv[i1, 2]*spat_expcv[i1, 5] - spat_expcv[i1, 1]*spat_expcv[i1, 3]*spat_expcv[i1, 6])*(-sin(q[i1, i4]) + q[i1, i4]);

# p x E = skew[-E.r].E = -E.skew[r].E^T.E = -E.skew[r]
var spat_Xv {i1 in SPAT_XV, i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0]} =
  if i2 <= 3 and i3 <= 3 then Ev[i1, i2, i3, i4]
  else if i2 > 3 and i3 > 3 then Ev[i1, i2 - 3, i3 - 3, i4]
  else if 3 < i2 <= 6 and i3 <= 3 then sum {i5 in SPAT_M: i5 <= 3} pvx[i1, i2 - 3, i5, i4] * Ev[i1, i5, i3, i4];

#----------- track the robot's position
var spat_X_ip {i1 in SPAT_L[1], i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0]} =
  sum{i5 in SPAT_M} spat_Xv[x_ij + i1 - 1, i2, i5, i4] * spat_Xp[x_jp + i1 - 1, i5, i3];

#----------- mass matrix M and internal forces b
param spat_ag {SPAT_M};

param spat_s_ii {i1 in SPAT_L[1], i2 in SPAT_M} =
  robot[i1, next('S_RX', RBT_IDX, i2 - 1)];

param spat_I_mm {i1 in SPAT_L[1], i2 in SPAT_M, i3 in SPAT_M} =
  if i2 = i3 and i3 > 3 then robot[i1, 'MASS']
  else if i2 = i3 and i3 <= 3 then robot[i1, next('IXX', RBT_IDX, i2 - 1)]
  else if (i2 = 1 and i3 = 2) or (i2 = 2 and i3 = 1) then robot[i1, 'IXY']
  else if (i2 = 1 and i3 = 3) or (i2 = 3 and i3 = 1) then robot[i1, 'IXZ']
  else if (i2 = 2 and i3 = 3) or (i2 = 3 and i3 = 2) then robot[i1, 'IYZ'];

param spat_I_im {i1 in SPAT_L[1], i2 in SPAT_M, i3 in SPAT_M} = sum {i4 in SPAT_M, i5 in SPAT_M} spat_Xp[x_im + i1 - 1, i4, i2] * spat_I_mm[i1, i4, i5] * spat_Xp[x_im + i1 - 1, i5, i3];

#----------- composite rigid body algorithm
var CRB_IC {i1 in SPAT_L[-1], i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0] : i1 > 0} = spat_I_im[i1, i2, i3] + sum {i5 in SPAT_TREE_C[i1], i6 in SPAT_M, i7 in SPAT_M} spat_X_ip[i5, i6, i2, i4] * CRB_IC[i5, i6, i7, i4] * spat_X_ip[i5, i7, i3, i4];

var CRB_f {i1 in SPAT_L[-1], i2 in SPAT_TREE_K[i1], i3 in SPAT_M, i4 in GRID[0] : i1 > 0} = if i1 = i2 then sum {i5 in SPAT_M} CRB_IC[i1, i3, i5, i4] * spat_s_ii[i1, i5] else sum {i5 in SPAT_M} spat_X_ip[prev(i2), i5, i3, i4] * CRB_f[i1, prev(i2), i5, i4];

var M {i1 in SPAT_L[-1], i2 in SPAT_L[-1], i3 in GRID[0] : i1 > 0 && i2 > 0} = if i1 in SPAT_TREE_S[i2] then sum {i5 in SPAT_M} CRB_f[i1, i2, i5, i3] * spat_s_ii[i2, i5] else if i2 in SPAT_TREE_S[i1] then M[i2, i1, i3] else 0;

#----------- recursive Netwon-Euler algorithm
var RNEA_v {i1 in SPAT_L[0], i2 in SPAT_M, i3 in GRID[0]} = if i1 = 0 then 0 else spat_s_ii[i1, i2] * v[i1, i3] + sum {i4 in SPAT_TREE_P[i1], i5 in SPAT_M} spat_X_ip[i1, i2, i5, i3] * RNEA_v[i4, i5, i3];

var RNEA_vx {i1 in SPAT_L[1], i2 in SPAT_M, i3 in SPAT_M, i4 in GRID[0]} =
  if i2 = 1 and i3 = 2 then -RNEA_v[i1, 3, i4]
  else if i2 = 1 and i3 = 3 then RNEA_v[i1, 2, i4]
  else if i2 = 2 and i3 = 1 then RNEA_v[i1, 3, i4]
  else if i2 = 2 and i3 = 3 then -RNEA_v[i1, 1, i4]
  else if i2 = 3 and i3 = 1 then -RNEA_v[i1, 2, i4]
  else if i2 = 3 and i3 = 2 then RNEA_v[i1, 1, i4]
  else if i2 = 4 and i3 = 2 then -RNEA_v[i1, 6, i4]
  else if i2 = 4 and i3 = 3 then RNEA_v[i1, 5, i4]
  else if i2 = 4 and i3 = 5 then -RNEA_v[i1, 3, i4]
  else if i2 = 4 and i3 = 6 then RNEA_v[i1, 2, i4]
  else if i2 = 5 and i3 = 1 then RNEA_v[i1, 6, i4]
  else if i2 = 5 and i3 = 3 then -RNEA_v[i1, 4, i4]
  else if i2 = 5 and i3 = 4 then RNEA_v[i1, 3, i4]
  else if i2 = 5 and i3 = 6 then -RNEA_v[i1, 1, i4]
  else if i2 = 6 and i3 = 1 then -RNEA_v[i1, 5, i4]
  else if i2 = 6 and i3 = 2 then RNEA_v[i1, 4, i4]
  else if i2 = 6 and i3 = 4 then -RNEA_v[i1, 2, i4]
  else if i2 = 6 and i3 = 5 then RNEA_v[i1, 1, i4];

var RNEA_a {i1 in SPAT_L[0], i2 in SPAT_M, i3 in GRID[0]} = if i1 = 0 then spat_ag[i2] else sum {i4 in SPAT_TREE_P[i1], i5 in SPAT_M} spat_X_ip[i1, i2, i5, i3] * RNEA_a[i4, i5, i3] + (sum {i5 in SPAT_M} RNEA_vx[i1, i2, i5, i3] * spat_s_ii[i1, i5]) * v[i1, i3];

var RNEA_fb {i1 in SPAT_L[1], i2 in SPAT_M, i3 in GRID[0]} = sum {i5 in SPAT_M} spat_I_im[i1, i2, i5] * RNEA_a[i1, i5, i3] + sum {i5 in SPAT_M, i6 in SPAT_M} -RNEA_vx[i1, i5, i2, i3] * spat_I_im[i1, i5, i6] * RNEA_v[i1, i6, i3];

var RNEA_f {i1 in SPAT_L[-1], i2 in SPAT_M, i3 in GRID[0] : i1 > 0} = RNEA_fb[i1, i2, i3] + sum {i5 in SPAT_TREE_C[i1], i6 in SPAT_M} spat_X_ip[i5, i6, i2, i3] * RNEA_f[i5, i6, i3];

var b {i1 in SPAT_L[1], i3 in GRID[0]} = sum {i2 in SPAT_M} spat_s_ii[i1, i2] * RNEA_f[i1, i2, i3];

#----------- operational space constraints
param OSIM_r_oo {i1 in R, i2 in SPAT_M} = constraint[i1, next('R_RX', CON_IDX, i2 - 1)];

# rigid bodies used in computation of constraints
set OSIM_K {i1 in PHASES} ordered by [0, nq] := union {i2 in CON_M[i1], i3 in CON_R[i1, i2]} SPAT_TREE_K[constraint[i3, 'BODY']];

set OSIM_B {i1 in PHASES} := setof {i2 in CON_M[i1], i3 in CON_R[i1, i2]} constraint[i3, 'BODY'];

var OSIM_R_0i {i1 in PHASES, i2 in OSIM_K[i1], i3 in SPAT_M, i4 in SPAT_M, i5 in GRID[i1]} = 
  if i2 = 0 and i3 = i4 then 1
  else sum {i6 in SPAT_TREE_P[i2], i7 in SPAT_M: (i3 <= 3 and i4 <= 3) or (i3 > 3 and i4 > 3)} OSIM_R_0i[i1, i6, i3, i7, i5] * spat_X_ip[i2, i4, i7, i5];

var OSIM_Rdot_0i {i1 in PHASES, i2 in OSIM_K[i1], i3 in SPAT_M, i4 in SPAT_M, i5 in GRID[i1]} = 
  if i2 > 0 then sum {i6 in SPAT_M: (i3 <= 3 and i4 <= 3) or (i3 > 3 and i4 > 3)} OSIM_R_0i[i1, i2, i3, i6, i5] * RNEA_vx[i2, i6, i4, i5];

var OSIM_X_bi {i1 in PHASES, i2 in OSIM_B[i1], i3 in SPAT_TREE_K[i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} = 
  if i2 = i3 and i4 = i5 then 1 
  else if i2 > i3 then sum {i7 in SPAT_M} OSIM_X_bi[i1, i2, prev(i3), i4, i7, i6] * spat_X_ip[prev(i3), i7, i5, i6];

var OSIM_v_rel {i1 in PHASES, i2 in OSIM_B[i1], i3 in SPAT_TREE_K[i2], i4 in SPAT_M, i5 in GRID[i1]} = if i3 > 0 then (-RNEA_v[i2, i4, i5] + sum {i6 in SPAT_M} OSIM_X_bi[i1, i2, i3, i4, i6, i5] * RNEA_v[i3, i6, i5]);

var OSIM_v_relx {i1 in PHASES, i2 in OSIM_B[i1], i3 in SPAT_TREE_K[i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} =
  if i4 = 1 and i5 = 2 then -OSIM_v_rel[i1, i2, i3, 3, i6]
  else if i4 = 1 and i5 = 3 then OSIM_v_rel[i1, i2, i3, 2, i6]
  else if i4 = 2 and i5 = 1 then OSIM_v_rel[i1, i2, i3, 3, i6]
  else if i4 = 2 and i5 = 3 then -OSIM_v_rel[i1, i2, i3, 1, i6]
  else if i4 = 3 and i5 = 1 then -OSIM_v_rel[i1, i2, i3, 2, i6]
  else if i4 = 3 and i5 = 2 then OSIM_v_rel[i1, i2, i3, 1, i6]
  else if i4 = 4 and i5 = 2 then -OSIM_v_rel[i1, i2, i3, 6, i6]
  else if i4 = 4 and i5 = 3 then OSIM_v_rel[i1, i2, i3, 5, i6]
  else if i4 = 4 and i5 = 5 then -OSIM_v_rel[i1, i2, i3, 3, i6]
  else if i4 = 4 and i5 = 6 then OSIM_v_rel[i1, i2, i3, 2, i6]
  else if i4 = 5 and i5 = 1 then OSIM_v_rel[i1, i2, i3, 6, i6]
  else if i4 = 5 and i5 = 3 then -OSIM_v_rel[i1, i2, i3, 4, i6]
  else if i4 = 5 and i5 = 4 then OSIM_v_rel[i1, i2, i3, 3, i6]
  else if i4 = 5 and i5 = 6 then -OSIM_v_rel[i1, i2, i3, 1, i6]
  else if i4 = 6 and i5 = 1 then -OSIM_v_rel[i1, i2, i3, 5, i6]
  else if i4 = 6 and i5 = 2 then OSIM_v_rel[i1, i2, i3, 4, i6]
  else if i4 = 6 and i5 = 4 then -OSIM_v_rel[i1, i2, i3, 2, i6]
  else if i4 = 6 and i5 = 5 then OSIM_v_rel[i1, i2, i3, 1, i6];

var OSIM_Xdot_bi {i1 in PHASES, i2 in OSIM_B[i1], i3 in SPAT_TREE_K[i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} = 
  if i3 > 0 then sum {i7 in SPAT_M} OSIM_v_relx[i1, i2, i3, i4, i7, i6] * OSIM_X_bi[i1, i2, i3, i7, i5, i6];

# X_ob = T_cb = frame with axes aligned relative to the body
#       -or-
# X_ob = R_0b * R_bc * T_cb = frame with axes aligned 
# with {0} located at {c} relative to b; 0cb => (0c)b
# R_0b * R_bc * R_cb = R_0b, so apply simplification in computation of SO(3)
# R_0b * R_bc * (p x R_cb) = -R_0b * R_bc * R_cb * rx = -R_0b rx, need to compute
var OSIM_X_ob {i1 in PHASES, i2 in CON_J[i1], i3 in CON_R[i1, i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} =
  if constraint[i3, 'F_O'] = constraint[i3, 'BODY'] then spat_Xp[x_cb + i3 - 1, i4, i5]
  else if i4 <= 3 and i5 <= 3 then OSIM_R_0i[i1, constraint[i3, 'BODY'], i4, i5, i6]
  else if i4 > 3 and i5 > 3 then OSIM_R_0i[i1, constraint[i3, 'BODY'], i4 - 3, i5 - 3, i6]
  else if i4 > 3 and i5 <= 3 then sum {i7 in SPAT_M, i8 in SPAT_M: i7 <= 3 and i8 <= 3} OSIM_R_0i[i1, constraint[i3, 'BODY'], i4 - 3, i7, i6] * Ep[x_cb + i3 - 1, i8, i7] * spat_Xp[x_cb + i3 - 1, i8 + 3, i5];

var OSIM_Xdot_ob {i1 in PHASES, i2 in CON_J[i1], i3 in CON_R[i1, i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} =
  if constraint[i3, 'F_O'] = constraint[i3, 'BODY'] then 0
  else if i4 <= 3 and i5 <= 3 then OSIM_Rdot_0i[i1, constraint[i3, 'BODY'], i4, i5, i6]
  else if i4 > 3 and i5 > 3 then OSIM_Rdot_0i[i1, constraint[i3, 'BODY'], i4 - 3, i5 - 3, i6]
  else if i4 > 3 and i5 <= 3 then sum {i7 in SPAT_M, i8 in SPAT_M: i7 <= 3 and i8 <= 3} OSIM_Rdot_0i[i1, constraint[i3, 'BODY'], i4 - 3, i7, i6] * Ep[x_cb + i3 - 1, i8, i7] * spat_Xp[x_cb + i3 - 1, i8 + 3, i5];

# variables for computing positions and angles
# X_c0 = X_0c_0 <= this transform will give position of {c} in {0} coordinates
var OSIM_X_c0 {i1 in PHASES, i2 in CON_M[i1], i3 in CON_R[i1, i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]} = sum {i7 in SPAT_M} spat_Xp[x_cb + i3 - 1, i4, i7] * OSIM_X_bi[i1, constraint[i3, 'BODY'], 0, i7, i5, i6];

# R_0c = R_0b * R_bc <= this transform will give angles of {c} in {0} coordinates
#var OSIM_R_0c {i1 in PHASES, i2 in CON_J[i1], i3 in CON_R[i1, i2], i4 in SPAT_M, i5 in SPAT_M, i6 in GRID[i1]: i4 <= 3 and i5 <= 3} = sum {i7 in SPAT_M: i7 <= 3} OSIM_R_0i[i1, constraint[i3, 'BODY'], i4, i7, i6] * Ep[x_cb + i3 - 1, i5, i7];

var J {i1 in PHASES, i2 in CON_J[i1], i3 in SPAT_L[1], i4 in GRID[i1]} = sum {i5 in CON_R[i1, i2], i6 in SPAT_M, i7 in SPAT_M, i8 in SPAT_M : i3 in SPAT_TREE_K[constraint[i5, 'BODY']]} OSIM_r_oo[i5, i6] * OSIM_X_ob[i1, i2, i5, i6, i7, i4] * OSIM_X_bi[i1, constraint[i5, 'BODY'], i3, i7, i8, i4] * spat_s_ii[i3, i8];

var phi {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} = sum {i4 in CON_R[i1, i2], i5 in SPAT_TREE_K[constraint[i4, 'BODY']], i6 in SPAT_M, i7 in SPAT_M, i8 in SPAT_M : i5 > 0} (OSIM_r_oo[i4, i6] * (OSIM_Xdot_ob[i1, i2, i4, i6, i7, i3] * OSIM_X_bi[i1, constraint[i4, 'BODY'], i5, i7, i8, i3] + OSIM_X_ob[i1, i2, i4, i6, i7, i3] * OSIM_Xdot_bi[i1, constraint[i4, 'BODY'], i5, i7, i8, i3]) * spat_s_ii[i5, i8]) * v[i5, i3];

var OSIM_position {i1 in PHASES, i2 in CON_M[i1], i3 in CON_R[i1, i2], i4 in SPAT_M, i5 in GRID[i1]} =
  if i4 = 1 then atan2(-OSIM_X_c0[i1, i2, i3, 3, 2, i5],OSIM_X_c0[i1, i2, i3, 3, 3, i5])
  else if i4 = 2 then asin(OSIM_X_c0[i1, i2, i3, 3, 1, i5])
  else if i4 = 3 then atan2(-OSIM_X_c0[i1, i2, i3, 2, 1, i5],OSIM_X_c0[i1, i2, i3, 1, 1, i5])
  else if i4 = 4 then (-(OSIM_X_c0[i1, i2, i3, 1, 3, i5]*OSIM_X_c0[i1, i2, i3, 4, 2, i5]) + OSIM_X_c0[i1, i2, i3, 1, 2, i5]*OSIM_X_c0[i1, i2, i3, 4, 3, i5] - OSIM_X_c0[i1, i2, i3, 2, 3, i5]*OSIM_X_c0[i1, i2, i3, 5, 2, i5] + OSIM_X_c0[i1, i2, i3, 2, 2, i5]*OSIM_X_c0[i1, i2, i3, 5, 3, i5] - OSIM_X_c0[i1, i2, i3, 3, 3, i5]*OSIM_X_c0[i1, i2, i3, 6, 2, i5] + OSIM_X_c0[i1, i2, i3, 3, 2, i5]*OSIM_X_c0[i1, i2, i3, 6, 3, i5])/2
  else if i4 = 5 then (OSIM_X_c0[i1, i2, i3, 1, 3, i5]*OSIM_X_c0[i1, i2, i3, 4, 1, i5] - OSIM_X_c0[i1, i2, i3, 1, 1, i5]*OSIM_X_c0[i1, i2, i3, 4, 3, i5] + OSIM_X_c0[i1, i2, i3, 2, 3, i5]*OSIM_X_c0[i1, i2, i3, 5, 1, i5] - OSIM_X_c0[i1, i2, i3, 2, 1, i5]*OSIM_X_c0[i1, i2, i3, 5, 3, i5] + OSIM_X_c0[i1, i2, i3, 3, 3, i5]*OSIM_X_c0[i1, i2, i3, 6, 1, i5] - OSIM_X_c0[i1, i2, i3, 3, 1, i5]*OSIM_X_c0[i1, i2, i3, 6, 3, i5])/2
  else if i4 = 6 then (-(OSIM_X_c0[i1, i2, i3, 1, 2, i5]*OSIM_X_c0[i1, i2, i3, 4, 1, i5]) + OSIM_X_c0[i1, i2, i3, 1, 1, i5]*OSIM_X_c0[i1, i2, i3, 4, 2, i5] - OSIM_X_c0[i1, i2, i3, 2, 2, i5]*OSIM_X_c0[i1, i2, i3, 5, 1, i5] + OSIM_X_c0[i1, i2, i3, 2, 1, i5]*OSIM_X_c0[i1, i2, i3, 5, 2, i5] - OSIM_X_c0[i1, i2, i3, 3, 2, i5]*OSIM_X_c0[i1, i2, i3, 6, 1, i5] + OSIM_X_c0[i1, i2, i3, 3, 1, i5]*OSIM_X_c0[i1, i2, i3, 6, 2, i5])/2;

var g {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} = sum {i4 in CON_R[i1, i2], i5 in SPAT_M} OSIM_r_oo[i4, i5] * OSIM_position[i1, i2, i4, i5, i3];

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* eom
#----------- continuous dynamics
# decision variable: constraint forces and motion constraints
var f {i1 in PHASES, CON_F[i1], GRID[i1]};

var g_des {i1 in PHASES, CON_M[i1], GRID[i1]} default 0;
var gdot_des {i1 in PHASES, CON_M[i1], GRID[i1]} default 0;
var gddot_des {i1 in PHASES, CON_M[i1], GRID[i1]} default 0;

# constraint variables
var Jv {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} = sum {i4 in Q} J[i1,i2,i4,i3]*v[i4,i3];
var Ja {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} = sum {i4 in Q} J[i1,i2,i4,i3]*a[i4,i3];
var Jtf {i1 in PHASES, i2 in Q, i3 in GRID[i1]} = sum {i4 in CON_F[i1]} J[i1,i4,i2,i3]*f[i1,i4,i3];

# constraints
subject to CONQ {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} : g[i1,i2,i3] - g_des[i1,i2,i3] = 0;
subject to CONV {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} : Jv[i1,i2,i3] - gdot_des[i1,i2,i3] = 0;
subject to CONA {i1 in PHASES, i2 in CON_M[i1], i3 in GRID[i1]} : Ja[i1,i2,i3] + phi[i1,i2,i3] - gddot_des[i1,i2,i3] = 0;

# EL variables
var EL {i1 in Q, i2 in GRID[0]} = sum {i3 in Q} M[i1,i3,i2]*a[i3,i2] + b[i1,i2] - u[i1,i2];

# constrained equations of motion
subject to EOM {i1 in PHASES, i2 in Q, i3 in GRID[i1]} : EL[i2,i3] = Jtf[i1,i2,i3];

#----------- instantaneous impacts
set PIM dimen 2; # PIM = switching/post-impact times as pair (t-, t+)
set PIM_MBRS := {'-', '+'};
set PIM_GRID {i1 in PIM_MBRS} := setof {(i2, i3) in PIM} if i1 = '-' then i2 else i3;

# coefficient of restitution
param epsilon {i1 in PHASES, CON_M[i1], PIM_GRID['+'] inter GRID[i1]} default 0;

# decision variable: constraint impulses
var I {i1 in PHASES, CON_F[i1], PIM_GRID['+'] inter GRID[i1]};

# impact variables
var Mv {i1 in Q, (i3, i4) in PIM} = sum {i2 in Q} M[i1,i2,i3]*(v[i2,i4]-v[i2,i3]); # M(v+ - v-)

var JtI {i1 in PHASES, i2 in Q, i3 in PIM_GRID['+'] inter GRID[i1]} = sum {i4 in CON_F[i1]} J[i1,i4,i2,i3] * I[i1,i4,i3];

var eJv {i1 in PHASES, i2 in CON_M[i1], (i3, i4) in PIM: i4 in GRID[i1]} = epsilon[i1,i2,i4] * sum {i5 in Q} J[i1,i2,i5,i4]*v[i5,i3];

# pre-impact equals post-impact switching time
subject to TPI {(i1, i2) in PIM} : t[i2] = t[i1]; # t+ = t-

# plastic impact equations
subject to IME {i1 in PHASES, i2 in Q, (i3, i4) in PIM: i4 in GRID[i1]} : Mv[i2,i3,i4] - JtI[i1,i2,i4] = 0;
subject to POSTQ {i1 in PHASES, i2 in Q, (i3, i4) in PIM: i4 in GRID[i1]} : q[i2,i4] - q[i2,i3] = 0;
subject to POSTV {i1 in PHASES, i2 in CON_M[i1], (i3, i4) in PIM: i4 in GRID[i1]} : Jv[i1,i2,i4] + eJv[i1,i2,i3,i4] = 0;

# (to, from) in PIM must be unique to avoid competing constraints
check {i1 in setof {(i3,i4) in PIM} i4} : sum {i2 in setof {(i3,i4) in PIM} i4 : i2 = i1} 1 = 1;
check {i1 in setof {(i3,i4) in PIM} i3} : sum {i2 in setof {(i3,i4) in PIM} i3 : i2 = i1} 1 = 1;
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* eom!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* Runge-Kutta
set ODE_SOLVERS;
set ODE_SUBGRID {ODE_SOLVERS} dimen 2;
param ODE_K {ODE_SOLVERS};
set ODEMBRS := {0, 'BEG', 'END', 'K'};
set ODE {i1 in ODE_SOLVERS, i2 in ODEMBRS} :=
  if i2 = 0 then union {(i3, i4) in ODE_SUBGRID[i1]} i3..i4 - 1
  else if i2 = 'K' then 1..ODE_K[i1]
  else if i2 = 'BEG' then setof {(i3, i4) in ODE_SUBGRID[i1]} i3
  else if i2 = 'END' then setof {(i3, i4) in ODE_SUBGRID[i1]} i4
  else {};

var ODE_delta {i1 in SUBPHASES, i2 in ODE_SOLVERS, i3 in ODE[i2, 0] : i3 in GRID[i1]} = (t[subphases[i1, 'END']] - t[subphases[i1, 'BEG']]) / (subphases[i1, 'END'] - subphases[i1, 'BEG']);

# Butcher tableau parameters
param ODE_AK {i1 in ODE_SOLVERS, ODE[i1, 'K'], ODE[i1, 'K']};
param ODE_bK {i1 in ODE_SOLVERS, ODE[i1, 'K']};
param ODE_cK {i1 in ODE_SOLVERS, ODE[i1, 'K']};

# decision variable: step size
var ODE_hK {i1 in ODE_SOLVERS, ODE[i1, 0]} >= 0; # time step size

param ODE_a0 {i1 in ODE_SOLVERS} = if 0 = sum {i2 in ODE[i1, 'K']} ODE_AK[i1, 1, i2] then 1;
param ODE_aK {i1 in ODE_SOLVERS} = if 0 = sum {i2 in ODE[i1, 'K']} (ODE_AK[i1, ODE_K[i1], i2] - ODE_bK[i1, i2]) then 1;

set ODE_GRID ordered by [0, GRID_MAX] := union {i2 in ODE_SOLVERS} 
  (setof {i4 in ODE[i2, 0], i5 in ODE[i2, 'K']: 
    1 + ODE_a0[i2] <= i5 <= ODE_K[i2] - ODE_aK[i2]} i4 + i5/10^ceil(log10(ODE_K[i2]+1)) 
  union ODE[i2, 0] union ODE[i2, 'END']);

# q defect constraints
subject to ODE_DEFQ {i1 in ODE_SOLVERS, i2 in Q, i3 in ODE[i1, 0]} : q[i2,next(i3,ODE_GRID,ODE_K[i1]+1-ODE_a0[i1]-ODE_aK[i1])] = q[i2,i3] + ODE_hK[i1,i3] * sum {i4 in ODE[i1,'K']} ODE_bK[i1,i4] * v[i2,next(i3,ODE_GRID,i4-ODE_a0[i1])];

# q collocation constraints
subject to ODE_COLQ {i1 in ODE_SOLVERS, i2 in Q, i3 in ODE[i1, 0], i4 in ODE[i1, 'K']: 1 + ODE_a0[i1] <= i4 <= ODE_K[i1] - ODE_aK[i1]} : q[i2, next(i3,ODE_GRID,i4-ODE_a0[i1])] = q[i2,i3] + ODE_hK[i1,i3] * sum {i5 in ODE[i1, 'K']} ODE_AK[i1,i4,i5] * v[i2,next(i3,ODE_GRID,i5-ODE_a0[i1])];

# v defect constraints
subject to ODE_DEFV {i1 in ODE_SOLVERS, i2 in Q, i3 in ODE[i1, 0]} : v[i2,next(i3,ODE_GRID,ODE_K[i1]+1-ODE_a0[i1]-ODE_aK[i1])] = v[i2,i3] + ODE_hK[i1,i3] * sum {i4 in ODE[i1,'K']} ODE_bK[i1,i4] * a[i2,next(i3,ODE_GRID,i4-ODE_a0[i1])];

# v collocation constraints
subject to ODE_COLV {i1 in ODE_SOLVERS, i2 in Q, i3 in ODE[i1, 0], i4 in ODE[i1, 'K']: 1 + ODE_a0[i1] <= i4 <= ODE_K[i1] - ODE_aK[i1]} : v[i2, next(i3,ODE_GRID,i4-ODE_a0[i1])] = v[i2,i3] + ODE_hK[i1,i3] * sum {i5 in ODE[i1, 'K']} ODE_AK[i1,i4,i5] * a[i2,next(i3,ODE_GRID,i5-ODE_a0[i1])];

# time defects constraints
subject to ODE_DEFTIME {i1 in ODE_SOLVERS, i2 in ODE[i1, 0]} : t[next(i2, ODE_GRID, ODE_K[i1]+1-ODE_a0[i1]-ODE_aK[i1])] = t[i2] + ODE_hK[i1, i2];

# time collocation constraints
subject to ODE_COLTIME {i1 in ODE_SOLVERS, i2 in ODE[i1, 0], i3 in ODE[i1, 'K']: 1 + ODE_a0[i1] <= i3 <= ODE_K[i1] - ODE_aK[i1]} : t[next(i2, ODE_GRID, i3-ODE_a0[i1])] = t[i2] + ODE_hK[i1, i2] * ODE_cK[i1, i3];

# step size constraints
subject to ODE_DEFSTEP {i1 in SUBPHASES, i2 in ODE_SOLVERS, i3 in ODE[i2, 0]: i3 in GRID[i1]} : ODE_hK[i2, i3] = (next(i3, ODE_GRID, ODE_K[i2]+1-ODE_a0[i2]-ODE_aK[i2]) - i3) * ODE_delta[i1, i2, i3];

# knots points must be integers
check {i1 in ODE_SOLVERS, (i2, i3) in ODE_SUBGRID[i1]} : i2 in Integers and i3 in Integers;
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* Runge-Kutta!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* grid
# model has been processed, so now add data specific rules for modules in the model file
redeclare set MOD_GRID {i1 in MOD_MODULES} := 
  if i1 = 'ODE' then ODE_GRID
  else if i1 = 'PIM' then union {(i2,i3) in PIM} {i2, i3}
  else if i1 = 'BEZ' then union {i2 in BEZ_LBLS, i3 in BEZ_SUBGRID[i2]} bez_subgrid[i2, i3, 'BEG']..bez_subgrid[i2, i3, 'END']
  else {};
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* grid!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* hc
# this module assumes 
#  1) indices used for g_des and friends can be reused for 
#     indexing into bez_curve and friends,
#  2) only one phase with value 1, and
#  3) GRID[1] = BEZ_GRID[1, 1]

#-- virtual holonomic constraints
param vhc_c {Q};
param vhc_Kp {CON_V[1], CON_V[1]};
param vhc_Kv {CON_V[1], CON_V[1]};
  # Kp and Kv are premultiplied by 1/epsilon^2 and 1/epsilon

# clock variable (state-based w.r.t. x hip position)
var vhc_tau0 = sum {i2 in Q} vhc_c[i2] * q[i2, 0];
var vhc_tauf = sum {i2 in Q} vhc_c[i2] * q[i2, GRID_MAX];
var vhc_taudot {i1 in GRID[1]} = (sum {i2 in Q} vhc_c[i2] * v[i2, i1])/(vhc_tauf - vhc_tau0);
var vhc_tauddot {i1 in GRID[1]} = (sum {i2 in Q} vhc_c[i2] * a[i2, i1])/(vhc_tauf - vhc_tau0);

var vhc_v {i2 in CON_V[1], i3 in GRID[1]} = sum {i4 in CON_V[1]} (vhc_Kp[i2, i4] * (g[1, i4, i3] - g_des[1, i4, i3]) + vhc_Kv[i2, i4] * (Jv[1, i4, i3] - gdot_des[1, i4, i3]));

subject to VHC_CLCK {i1 in GRID[1]} : bez_tau[1, 1, i1] = ((sum {i2 in Q} vhc_c[i2] * q[i2, i1]) - vhc_tau0)/(vhc_tauf - vhc_tau0);

subject to VHC_Q {i2 in CON_V[1], i3 in GRID[1]} : g_des[1, i2, i3] = bez_curve[1, 1, 0, i2 - 5, i3];

subject to VHC_V {i2 in CON_V[1], i3 in GRID[1]} : gdot_des[1, i2, i3] = bez_curve[1, 1, 1, i2 - 5, i3] * vhc_taudot[i3];

subject to VHC_A {i2 in CON_V[1], i3 in GRID[1]} : gddot_des[1, i2, i3] = bez_curve[1, 1, 2, i2 - 5, i3] * vhc_taudot[i3]^2 + bez_curve[1, 1, 1, i2 - 5, i3] * vhc_tauddot[i3] - vhc_v[i2, i3];

#-- physical holonomic constraints
param phc_Kp {CON_P[1], CON_P[1]};
param phc_Kv {CON_P[1], CON_P[1]};

# clock variable (time-based w.r.t. gait period)
var phc_tau0 = t[0];
var phc_tauf = t[GRID_MAX];
var phc_taudot {i1 in GRID[1]} = 1 / (phc_tauf - phc_tau0);
param phc_tauddot {i1 in GRID[1]} = 0;

var phc_v {i2 in CON_P[1], i3 in GRID[1]: i2 <= 2} = sum {i4 in CON_P[1]: i2 <= 2} (phc_Kp[i2, i4] * (g[1, i4, i3] - g_des[1, i4, i3]) + phc_Kv[i2, i4] * (Jv[1, i4, i3] - gdot_des[1, i4, i3]));

subject to PHC_CLCK {i1 in GRID[1]} : bez_tau[0, 1, i1] = (t[i1] - phc_tau0)/(phc_tauf - phc_tau0);

subject to PHC_Q {i2 in CON_P[1], i3 in GRID[1]: i2 <= 2} : g_des[1, i2, i3] = bez_curve[0, 1, 0, i2 - 1, i3];

subject to PHC_V {i2 in CON_P[1], i3 in GRID[1]: i2 <= 2} : gdot_des[1, i2, i3] = bez_curve[0, 1, 1, i2 - 1, i3] * phc_taudot[i3];

subject to PHC_A {i2 in CON_P[1], i3 in GRID[1]: i2 <= 2} : gddot_des[1, i2, i3] = bez_curve[0, 1, 2, i2 - 1, i3] * phc_taudot[i3]^2 - phc_v[i2, i3];
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* vhc!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* gait
# constraints are set up so that
# 1-2 => left leg (x, z)
# 3-4 => right leg (x, z) [swing leg for grid points 0..GRID_MAX]
# 5-8 => vhc q[4, *]-q[7,*]
# 9-12 => vhc transmission matrix B[4, *]-B[7, *]

set GAIT_COORDS;

param mu;
param step_length_des >= 0;
param step_time_des >= 0;
param GAIT_A {GAIT_COORDS, GAIT_COORDS};

#-- step period
subject to T0: t[0] = 0;
subject to TF: 0.25 <= t[GRID_MAX] <= 0.75;

#-- step length
subject to STEP_LENGTH : 0.1 <= g[1, 3, GRID_MAX] <= 0.5;

#-- step height
subject to STEP_HEIGHT : g[1, 4, GRID_MAX] = 0;

#-- walking speed
subject to WALKING_SPEED : g[1, 3, GRID_MAX] = step_length_des / step_time_des * t[GRID_MAX];

#-- theta dot
subject to THETA_DOT {i1 in GRID[1]} : 0.05 <= vhc_taudot[i1];

#-- swing foot height
subject to SWING_FOOT_HEIGHT : 0.1 <= g[1, 4, member(ceil(card(GRID[1]) / 2), GRID[1])];

#-- ground clearance
subject to GROUND_CLEARANCE {i1 in GRID[1]} : g[1, 4, i1] >= 0;

#-- stance position at 0
subject to STANCE_POSITION_AT_0 {i1 in CON_P[1]: i1 <= 2} : g[1, i1, 0] = 0;

#-- stance velocity at 0
subject to STANCE_VELOCITY_AT_0 {i1 in CON_P[1]: i1 <= 2} : Jv[1, i1, 0] = 0;

#-- swing-leg impact velocity
subject to SWING_IMPACT_VELX : -0.01 <= Jv[1, 3, GRID_MAX] <= 0.01;
subject to SWING_IMPACT_VELZ : Jv[1, 4, GRID_MAX] <= -0.25;

#-- bezier coefficients
subject to BEZIER_COEFFS {i1 in BEZ_LBLS, i2 in BEZ[i1, 'CP_ROW'], i3 in BEZ[i1, 'CP_COL']} : -5 <= bez_ctls[i1, 1, i2, i3] <= 5;

#-- periodicity constraints
subject to PERIODICITY_CONSTRAINT_Q {i1 in Q: i1 > 1} : q[i1, 0] - sum {i2 in Q: i1 > 1} GAIT_A[i1, i2] * q[i2, GRID_MAX] = 0;

var M0 {i1 in Q, i2 in Q} = M[i1, i2, 0];
var Rvf {i1 in Q} = sum {i2 in Q} GAIT_A[i1, i2] * v[i2, GRID_MAX];
var J0I {i1 in Q} = sum {i4 in CON_P[1]: i4 <= 2} J[1, i4, i1, 0] * I[1, i4, GRID_MAX];
var MOM0 {i1 in Q} = sum {i2 in Q} M0[i1, i2] * (v[i2, 0] - Rvf[i2]);
subject to PERIODICITY_CONSTRAINT_IME {i1 in Q} : MOM0[i1] - J0I[i1] = 0;

#-- friction
subject to PYRAMID_POINT_PLANAR_LB {i1 in GRID[1]} : -mu * f[1, 2, i1] <= f[1, 1, i1];

subject to PYRAMID_POINT_PLANAR_UB {i1 in GRID[1]} : f[1, 1, i1] <= mu * f[1, 2, i1];

subject to NORMAL_FORCE {i1 in GRID[1]} : f[1, 2, i1] >= 0;

#-- state and input constraints
subject to V_BNDS {i1 in Q, i2 in GRID[1]} : -20 <= v[i1, i2] <= 20;
subject to A_BNDS {i1 in Q, i2 in GRID[1]} : -100 <= a[i1, i2] <= 100;

subject to FC_BNDS {i1 in CON_P[1], i2 in GRID[1]: i1 <= 2} : -1000 <= f[1, i1, i2] <= 1000;
subject to F_ZERO {i1 in CON_P[1], i2 in GRID[1]: 2 < i1} : f[1, i1, i2] = 0;

subject to U_BNDS {i1 in CON_U[1], i2 in GRID[1]} : -100 <= f[1, i1, i2] <= 100;
subject to U_ZERO {i1 in Q, i2 in GRID[1]} : u[i1, i2] = 0;

subject to TORSO_PX {i1 in GRID[1]} : -0.5 <= q[1, i1] <= 0.5;
subject to TORSO_PZ {i1 in GRID[1]} : 0.5 <= q[2, i1] <= 1.25;
subject to TORSO_RY {i1 in GRID[1]} : -acos(sqrt(3)/2) <= q[3, i1] <= acos(sqrt(3)/2);

subject to HIPS {i1 in Q, i2 in GRID[1]: 4 <= i1 <= 5} : -acos(-1) <= q[i1, i2] <= acos(-1);
subject to KNEES {i1 in Q, i2 in GRID[1]: i1 >= 6} : 5 * acos(-1) / 180 <= q[i1, i2] <= acos(-1);
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* gait!

#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* objective
var uhzd = sum {i1 in ODE_SOLVERS, i2 in ODE[i1, 0]} ODE_hK[i1, i2] * sum {i3 in ODE[i1, 'K']} (ODE_bK[i1, i3] * (sum {i4 in PHASES, i5 in CON_U[i4]: i2 in GRID[i4]} f[i4, i5, next(i2,ODE_GRID,i3-ODE_a0[i1])]^2));

minimize UHZD : uhzd;
#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#* objective!
