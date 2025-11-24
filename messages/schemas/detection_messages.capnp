@0xa8b7c6d5e4f39281;

using Sensors = import "sensors.capnp";
using Geometry = import "geometry_msgs.capnp";
using Point = Geometry.Point;

struct DetectionImage {
  image @0 :Sensors.Image;        # Cropped image of the detection
  coordinates @1 :Point;              # 3D coordinates (x, y, z)
  description @2 :Text;               # Description to validate with the LLM
  objectId @3 :UInt32;                # Unique ID for this detection
}

struct LLMResult {
  objectId @0 :UInt32;        # ID of the processed detection
  isValidPerson @1 :Bool;     # LLM result (true/false)
  coordinates @2 :Point;      # 3D coordinates if validated
  llmResponse @3 :Text;       # Full LLM response
}

struct DetectionRawImage {
  id @0 :Int32;
  width @1 :UInt32;
  height @2 :UInt32;
  type @3 :UInt32;  # OpenCV type (e.g., CV_8UC3, CV_32FC1, etc.)
  data @4 :Data;    # Raw pixel bytes
}
