@0xf0c9bede8a1f3e61;


# TODO: Estandarizar 'documentacion' de capnp a ingles

struct Envelope {
  typeId @0 :UInt64;          # ID @0x... of the real struct (optional but useful)
  typeName @1 :Text;          # Capnp type name, e.g. "Header"
  topic @2 :Text;             # Topic where it was published
  timestampUsec @3 :UInt64;   # Timestamp in microseconds from epoch

  schemaPath @4 :Text;        # Path/identifier of the schema on project
  schemaText @5 :Text;        # Optional embedded schema

  payload @6 :AnyPointer;     # Actual wrapped message
}
