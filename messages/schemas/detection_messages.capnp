@0xa8b7c6d5e4f39281;

using Sensors = import "sensors.capnp";
using Geometry = import "geometry_msgs.capnp";
using Point = Geometry.Point;

struct DetectionImage {
    image @0 :Sensors.ImageData;        # Imagen recortada de la detección
    coordinates @1 :Point;              # Coordenadas 3D (x, y, z)
    description @2 :Text;               # Descripción a validar con LLM
    objectId @3 :UInt32;                # ID único de la detección
}

struct LLMResult {
    objectId @0 :UInt32;        # ID de la detección procesada
    isValidPerson @1 :Bool;     # Resultado del LLM (true/false)
    coordinates @2 :Point;      # Coordenadas 3D si es válida
    llmResponse @3 :Text;       # Respuesta completa del LLM
}
