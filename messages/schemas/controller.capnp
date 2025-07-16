@0xcf066f32651a0194;

enum CommandType{
  takeoff   @0;
  land      @1;
  waypoint  @2;
}

struct Waypoint{
  x @0 :UInt32;
  y @1 :UInt32;
  z @2 :UInt32;
}

struct Command{
 type         @0 :CommandType; 
 union {
    waypoint  @1 :Waypoint;
    takeoff   :group{
      height  @2 :UInt8;
    }
    land      @3 :Void;
}
}
