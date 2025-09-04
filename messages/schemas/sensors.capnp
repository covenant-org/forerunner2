@0xfc8ada61b7644e99;

struct StereoMic{
  left  @0 :Float32;
  right @1 :Float32;
}

struct ImageData {
    width @0 :UInt16;
    height @1 :UInt16;
    encoding @2 :Text;      # Ej: "jpeg", "png", "raw", etc.
    data @3 :Data;          # Imagen en formato binario
    timestamp @4 :UInt64;   # Marca de tiempo para control
}