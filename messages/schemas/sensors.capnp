@0xfc8ada61b7644e99;

struct StereoMic{
  left  @0 :Float32;
  right @1 :Float32;
}

struct DetectionRawImage {
  id @0 :Int32;
  width @2 :UInt32;
  height @3 :UInt32;
  type @4 :UInt32;  # OpenCV type (e.g., CV_8UC3, CV_32FC1, etc.)
  data @5 :Data;    # Raw pixel bytes
}
