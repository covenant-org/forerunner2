@0xcf066f32651a0194;

struct PointYaw{
  x   @0 :Float32;
  y   @1 :Float32;
  z   @2 :Float32;
  r   @3 :Float32;
}

struct Command{
 union {
    waypoint  @0 :PointYaw;
    takeoff   :group{
      altitude  @1 :UInt8;
    }
    land      @2 :Void;
    arm       @3 :Void;
    disarm    @4 :Void;
    accel     @5 :PointYaw;
    velocity  @6 :PointYaw;
    manual    @7 :PointYaw;
    offboard   :group{
      enable  @8 :Bool;
    }
}
}

struct Goal{
  union{
    relative  @0 :PointYaw;
    coords    @1 :PointYaw;
    latlon    @2 :PointYaw;
  }
}
