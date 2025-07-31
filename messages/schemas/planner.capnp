@0xf1906ddef3fc6991;

using GeometryMsgs = import "geometry_msgs.capnp";
using PoseStamped = GeometryMsgs.PoseStamped;

using NavMsgs = import "nav_msgs.capnp";
using Path = NavMsgs.Path;

using Generics = import "generics.capnp";
using GenericResponse = Generics.GenericResponse;


struct ReplanRequest {
  union {
    start :group {
      pose             @0 :PoseStamped;
      currentPathIndex @1 :UInt8;
      pathSequence     @2 :UInt32;
    }
    stop               @3 :Void;
  }
}
