@0x9246579439031e06;

enum RequestType {
  addNode @0;
  queryNode @1;
  addHost @2;
}

struct RegistryRequest {
  type @0 :RequestType;
  path @1 :Text;
  union {
    addNode @2 :Void;
    queryNode @3 :Void;
    addHost :group{
      address @4 :Text;
      port    @5 :UInt32;
    }
  }
}

struct RegistryResponse {
  code @0 :UInt32;
  union {
    errorMessage @1 :Text;
    host :group{
        port    @2 :UInt32;
        address @3 :Text;
    }
  }
} 

enum RegistryNotificationType {
  nodeAdded @0;
  nodeDeleted @1;
}

struct RegistryNotification {
  type @0 :RegistryNotificationType;
  union {
    nodeAdded :group {
      path @1 :Text;
      port @2 :UInt32;
    }
    nodeDeleted :group {
      path @3 :Text;
      port @4 :UInt32;
    }
  }
}
