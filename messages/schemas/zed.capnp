@0xf7d08bd9a8e2fe17;

using Sensors = import "sensors.capnp";
using PointCloud = Sensors.PointCloud;

struct PointCloudChunk {
	index @0 :UInt16;
	cloud @1 :PointCloud;
}
