@0xe49de075f1994b84;

struct Position {
  x @0 :Float32;
  y @1 :Float32;
  z @2 :Float32;
}

struct BatteryLevel {
  percentage @0 :UInt8;
}

struct Telemetry {
  battery @0 :BatteryLevel;
}


struct Odometry {
  position @0 :Position;
  velocity @1 :Position;
  angular  @2 :Position;
}

struct HomePosition {
  pos @0 :Position;
}
