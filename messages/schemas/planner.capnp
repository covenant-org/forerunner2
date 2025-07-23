@0xf1906ddef3fc6991;

using GeometryMsgs = import "geometry_msgs.capnp";
using PoseStamped = GeometryMsgs.PoseStamped;

using NavMsgs = import "nav_msgs.capnp";
using Path = NavMsgs.Path;

struct PlanRequest {
	pose @0 :PoseStamped;
}

struct PlanResponse {
	path @0 :Path;
	pathId @1 :UInt32;
	success @2 :Bool;
}

struct ReplanRequest {
	pose @0 :PoseStamped;
	currentPathIndex @1 :UInt8;
	pathSequencce @2 :UInt32;
}

struct ReplanResponse {
	path @0 :Path;
	pathSequence @1 :UInt32;
	success @2 :Bool;
}
