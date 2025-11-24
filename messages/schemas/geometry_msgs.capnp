@0x9571201fccc02aa8;

using StdMsgs = import "std_msgs.capnp";
using Header = StdMsgs.Header;

# geometry_msgs/Point
struct Point {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
}

struct Vector3 {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
}

# TODO: reconsiderar si siguen valiendo la pena los mensajes stamped
struct PointStamped {
  header @0 :Header;
  point  @1 :Point;
}

struct Quaternion {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
  w @3 :Float64;
}

struct Pose {
  position    @0 :Point;
  orientation @1 :Quaternion;
}

struct PoseWithCovariance {
  pose       @0 :Pose;
  covariance @1 :List(Float64);  # Debe contener 36 elementos (matriz 6x6)
}

struct Twist {
  linear  @0 :Vector3;
  angular @1 :Vector3;
}

struct TwistWithCovariance {
  twist  @0 :Twist;
  covariance @1 :List(Float64);
}

# TODO: reconsiderar si siguen valiendo la pena los mensajes stamped
struct PoseStamped {
  header @0 :Header;
  pose   @1 :Pose;
}

struct Transform {
  translation @0 :Vector3;
  rotation    @1 :Quaternion;
}

# TODO: reconsiderar si siguen valiendo la pena los mensajes stamped
struct TransformStamped {
  header    @0 :Header;
  transform @1 :Transform;
}


