@0x9571201fccc02aa8;

using StdMsgs = import "std_msgs.capnp";
using Header = StdMsgs.Header;

# geometry_msgs/Point
struct Point {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
}

# geometry_msgs/PointStamped
struct PointStamped {
  header @0 :Header;
  point  @1 :Point;
}

# geometry_msgs/Quaternion
struct Quaternion {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
  w @3 :Float64;
}

# geometry_msgs/Pose
struct Pose {
  position    @0 :Point;
  orientation @1 :Quaternion;
}

# geometry_msgs/PoseStamped
struct PoseStamped {
  header @0 :Header;
  pose   @1 :Pose;
}
