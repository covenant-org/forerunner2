@0xe49de075f1994b84;

struct Position {
  x @0 :Float32;
  y @1 :Float32;
  z @2 :Float32;
}

struct GPS {
  latitude @0 :Float32;
  longitude @1 :Float32;
  altitude @2 :Float32;
}

struct Quartenion{
  x @0 :Float32;
  y @1 :Float32;
  z @2 :Float32;
  w @3 :Float32;
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


struct Odometry {
  position @0 :Position;
  velocity @1 :Position;
  angular  @2 :Position;
  q        @3 :Quartenion;
  heading  @4 :Float32;
}

struct HomePosition {
  pos @0 :Position;
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
