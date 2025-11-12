@0xf0c9bede8a1f3e61;

# Generic envelope for transporting Cap'n Proto structs together with their
# publishing metadata, para no depender de catálogos externos.

struct Envelope {
  typeId @0 :UInt64;          # ID @0x... del struct real (opcional pero útil)
  typeName @1 :Text;          # Nombre legible, ej. "std_msgs/Header"
  topic @2 :Text;             # Tópico donde se publicó
  timestampUsec @3 :UInt64;   # Marca de tiempo (µs desde epoch, por ejemplo)

  schemaPath @4 :Text;        # Ruta/identificador del schema en disco
  schemaText @5 :Text;        # Schema embebido opcional

  payload @6 :AnyPointer;     # Mensaje real envuelto
}
