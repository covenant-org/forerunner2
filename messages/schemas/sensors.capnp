@0xfc8ada61b7644e99;

struct StereoMic{
  left  @0 :Float32;
  right @1 :Float32;
}

struct Image {
  width @0 :UInt32;
  height @1 :UInt32;
  type @2 :UInt32;  # OpenCV type (e.g., CV_8UC3, CV_32FC1, etc.)
  data @3 :Data;    # Raw pixel bytes
}

struct PointCloud { # TODO: Reestructurar (opcional)
	width @0 :UInt16;
	height @1 :UInt16;
	data @2 :Data;
	size @3 :UInt32;
}

