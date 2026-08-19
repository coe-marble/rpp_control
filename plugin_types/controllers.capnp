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

interface MotionController2D extends(Controller(Msgs.Odometry2D, Msgs.Odometry2D, EnablerOdometry2D, Msgs.Wrench2D))
$Anot.plugin("MotionController2D"){}

interface MotionController3D extends(Controller(Msgs.Odometry3D, Msgs.Odometry3D, EnablerOdometry3D, Msgs.Wrench3D))
$Anot.plugin("MotionController3D") {}


interface TwistController2D extends(Controller(Msgs.Twist2D, Msgs.Twist2D, Enabler2D, Msgs.Wrench2D))
$Anot.plugin("TwistController2D"){}


interface TwistController3D extends(Controller(Msgs.Twist3D, Msgs.Twist3D, Enabler3D, Msgs.Wrench3D))
$Anot.plugin("TwistController3D"){}

interface PoseController2D extends(Controller(Msgs.Pose2D, Msgs.Pose2D, Enabler2D, Msgs.Twist2D))
$Anot.plugin("PoseController2D") {}

interface PoseController3D extends(Controller(Msgs.Pose3D, Msgs.Pose3D, Enabler3D, Msgs.Twist3D))
$Anot.plugin("PoseController3D") {}


interface Allocator(ReferenceTau, State, Enabler, AllocatedTau) {
  allocate @0 (reference :ReferenceTau, state :State, enabler :Enabler, dt :Float64)
    -> (commands :AllocatedTau, realized :ReferenceTau);
  outputSize @1 () -> (dim :UInt32);
}

interface MotionControllerAllocator2D extends(Allocator(Msgs.Wrench2D, Msgs.Odometry2D, Enabler2D, Msgs.Command))
$Anot.plugin("MotionControllerAllocator2D"){}

interface MotionControllerAllocator3D extends(Allocator(Msgs.Wrench3D, Msgs.Odometry3D, Enabler3D, Msgs.Command))
$Anot.plugin("MotionControllerAllocator3D"){}
