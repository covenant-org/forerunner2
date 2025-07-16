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
  pos @0 :Position;
}

struct Takeoff {
  altitude @0 :Float32;
}

struct Land {}

enum OffboardOperation {
  start @0;
  stop @1;
}

struct Offboard {
  operation @0 :OffboardOperation;
  union {
    start :group {
      pos @1: Position;
    }
    stop @2: Void;
  }
}

