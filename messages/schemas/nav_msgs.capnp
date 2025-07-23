@0xb5633d5661ec75b4;  # Identificador único de este schema

using StdMsgs = import "std_msgs.capnp";
using Header = StdMsgs.Header;

using GeometryMsgs = import "geometry_msgs.capnp";
using PoseStamped = GeometryMsgs.PoseStamped;

# nav_msgs/Path
struct Path {
  header @0 :Header;
  poses  @1 :List(PoseStamped);
}
