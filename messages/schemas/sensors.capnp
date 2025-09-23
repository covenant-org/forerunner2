@0xfc8ada61b7644e99;

struct StereoMic{
  left  @0 :Float32;
  right @1 :Float32;
}

struct ImageData {
  width @0 :UInt32;
  height @1 :UInt32;
  type @2 :UInt32;  # OpenCV type (e.g., CV_8UC3, CV_32FC1, etc.)
  data @3 :Data;    # Raw pixel bytes
}

struct DetectionRawImage {
  id @0 :Int32;
  width @1 :UInt32;
  height @2 :UInt32;
  type @3 :UInt32;  # OpenCV type (e.g., CV_8UC3, CV_32FC1, etc.)
  data @4 :Data;    # Raw pixel bytes
}
