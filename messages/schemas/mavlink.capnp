@0xe49de075f1994b84;

using Geometry = import "geometry_msgs.capnp";
using Point = Geometry.Point;
using Vector3 = Geometry.Vector3;
using Quaternion = Geometry.Quaternion;

struct GPS {
  latitude @0 :Float32;
  longitude @1 :Float32;
  altitude @2 :Float32;
}

struct BatteryLevel {
  percentage @0 :UInt8;
}

struct Telemetry {
  battery @0 :BatteryLevel;
  mode    @1 :Text;
  inAir   @2 :Bool;
  armed   @3 :Bool;
}

struct HomePosition {
  pos @0 :Point;
  gps @1 :GPS;
}

struct Altitude{
  local     @0 :Float32;
  relative  @1 :Float32;
  monotonic @2 :Float32;
  avg       @3 :Float32;
}

struct Takeoff {
  desiredAltitude @0 :Float32;
}

struct MissionCommand {
  union {
    takeoff @0 :Takeoff;
    land    @1 :Void;
  }
}
