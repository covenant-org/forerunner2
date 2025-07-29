@0x982a10798be0ee7b;

using StdMsgs = import "std_msgs.capnp";
using Header = StdMsgs.Header;
using ColorRGBA = StdMsgs.ColorRGBA;

using GeometryMsgs = import "geometry_msgs.capnp";
using Pose = GeometryMsgs.Pose;
using Vector3d = GeometryMsgs.Vector3d;

enum MarkerType {
  cube @0;
  sphere @1;
}

# visualization_msgs/Marker (mínimo)
struct Marker {
  header    @0 :Header;
  ns        @1 :Text;
  id        @2 :UInt32;
  lifetime  @3 :Float64;    # seconds
  pose      @4 :Pose;
  color     @5 :ColorRGBA;
  shape     @6 :MarkerType;
  scale     @7 :Vector3d;    # size of the shape
}

# visualization_msgs/MarkerArray
struct MarkerArray {
  markers @0 :List(Marker);
}
