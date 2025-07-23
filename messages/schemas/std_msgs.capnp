@0xe657b4c9455930c0;

# std_msgs/Header
struct Header {
  seq      @0 :UInt32;
  stampSec @1 :UInt64;   # secs since epoch
  stampNsec @2 :UInt32;  # nanosecs remainder
  frameId  @3 :Text;
}

# std_msgs/ColorRGBA
struct ColorRGBA {
  r @0 :Float32;
  g @1 :Float32;
  b @2 :Float32;
  a @3 :Float32;
}
