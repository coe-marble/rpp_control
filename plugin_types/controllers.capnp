@0xbadbadbadbadbad1;
using Msgs = import "rpp_common/msgs.capnp";
using Anot = import "rpp_common/anot.capnp";

interface Controller(RefState, State, Enabler, Command) {
  step     @0 (ref_state: RefState, state :State, enabler :Enabler, dt :Float64) -> (command :Command);
}

struct Enabler2D {
  enableX @0 :Bool;
  enableY @1 :Bool;
  enableN @2 :Bool;
}

struct Enabler3D {
  enableX @0 :Bool;
  enableY @1 :Bool;
  enableZ @2 :Bool;
  enableK @3 :Bool;
  enableM @4 :Bool;
  enableN @5 :Bool;
}

struct EnablerOdometry2D {
  pose @0 :Enabler2D;
  twist @1 :Enabler2D;
}

struct EnablerOdometry3D {
  pose @0 :Enabler3D;
  twist @1 :Enabler3D;
}

# Error history is retained by every controller for each controlled axis.
# Axis ordering follows the concrete controller interface:
# pose, twist, and pose-then-twist for motion controllers.
struct ControllerAxisErrorInfo {
  integralError @0 :Float64;
  previousError @1 :Float64;
  previousDerivative @2 :Float64;
}

# Lifecycle handover state shared by all controller interfaces. `state` is
# controller-owned numeric state: e.g. an MPC horizon, NMPC solver warm start,
# or MPPI nominal command sequence. A receiving controller may use it when its
# layout is compatible, while every controller preserves `axisErrors`.
struct ControllerHandoverState {
  axisErrors @0 :List(ControllerAxisErrorInfo);
  state @1 :Msgs.FloatMatrix;
}

# Lifecycle handover state shared by allocator interfaces. Allocators do not
# have controller error axes; `state` retains any rate-limit or optimizer data.
struct AllocatorHandoverState {
  state @0 :Msgs.FloatMatrix;
}

interface MotionController2D extends(Controller(Msgs.Odometry2D, Msgs.Odometry2D, EnablerOdometry2D, Msgs.Wrench2D))
$Anot.plugin("MotionController2D")
$Anot.lifecycleState("ControllerHandoverState"){}

interface MotionController3D extends(Controller(Msgs.Odometry3D, Msgs.Odometry3D, EnablerOdometry3D, Msgs.Wrench3D))
$Anot.plugin("MotionController3D")
$Anot.lifecycleState("ControllerHandoverState") {}


interface TwistController2D extends(Controller(Msgs.Twist2D, Msgs.Twist2D, Enabler2D, Msgs.Wrench2D))
$Anot.plugin("TwistController2D")
$Anot.lifecycleState("ControllerHandoverState"){}


interface TwistController3D extends(Controller(Msgs.Twist3D, Msgs.Twist3D, Enabler3D, Msgs.Wrench3D))
$Anot.plugin("TwistController3D")
$Anot.lifecycleState("ControllerHandoverState"){}

interface PoseController2D extends(Controller(Msgs.Pose2D, Msgs.Pose2D, Enabler2D, Msgs.Twist2D))
$Anot.plugin("PoseController2D")
$Anot.lifecycleState("ControllerHandoverState") {}

interface PoseController3D extends(Controller(Msgs.Pose3D, Msgs.Pose3D, Enabler3D, Msgs.Twist3D))
$Anot.plugin("PoseController3D")
$Anot.lifecycleState("ControllerHandoverState") {}


interface Allocator(ReferenceTau, State, Enabler, AllocatedTau) {
  allocate @0 (reference :ReferenceTau, state :State, enabler :Enabler, dt :Float64)
    -> (commands :AllocatedTau, realized :ReferenceTau);
  outputSize @1 () -> (dim :UInt32);
}

interface MotionControllerAllocator2D extends(Allocator(Msgs.Wrench2D, Msgs.Odometry2D, Enabler2D, Msgs.Command))
$Anot.plugin("MotionControllerAllocator2D")
$Anot.lifecycleState("AllocatorHandoverState"){}

interface MotionControllerAllocator3D extends(Allocator(Msgs.Wrench3D, Msgs.Odometry3D, Enabler3D, Msgs.Command))
$Anot.plugin("MotionControllerAllocator3D")
$Anot.lifecycleState("AllocatorHandoverState"){}
