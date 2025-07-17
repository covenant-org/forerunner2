@0xcf066f32651a0194;

struct Waypoint{
  x @0 :UInt32;
  y @1 :UInt32;
  z @2 :UInt32;
}

struct VelocityYaw{
  x   @0 :Float;
  y   @1 :Float;
  z   @2 :Float;
  r   @3 :Float;
}

struct Command{
 union {
    waypoint  @1 :Waypoint;
    takeoff   :group{
      height  @2 :UInt8;
    }
    land      @3 :Void;
    arm       @4 :Void;
    disarm    @5 :Void;
    accel     @6 :Waypoint;
    velocity  @7 :VelocityYaw;
    manual    @8 :VelocityYaw;
}
}
