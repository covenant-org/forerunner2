@0xe512af596e03edb2;

using GeoMsgs = import "geometry_msgs.capnp";
using Pose = GeoMsgs.Pose;
using Point = GeoMsgs.Point;

struct TrackingMarker {
  id    @0  :UInt8; 
  point @1  :Point;
}

struct RigidBody {
  id        @0  :UInt8; 
  pose  @1  :Pose;
  name      @2  :Text;
}
