@0xb5633d5661ec75b4;  # Identificador único de este schema

using StdMsgs = import "std_msgs.capnp";
using Header = StdMsgs.Header;

using GeometryMsgs = import "geometry_msgs.capnp";
using PoseStamped = GeometryMsgs.PoseStamped;
using PoseWithCovariance = GeometryMsgs.PoseWithCovariance;
using TwistWithCovariance = GeometryMsgs.TwistWithCovariance;

# TODO: Considerar si los 'Header' siguen siendo necesarios 
struct Path {
  header @0 :Header;
  poses  @1 :List(PoseStamped);
}

# TODO: Se borro el campo 'GPS', considera agregarlo en una variante de mavlink.capnp
struct Odometry {
  pose @0 :PoseWithCovariance;
  twist @1 :TwistWithCovariance;
}