@0x9246579439031e06;

enum RequestType {
  addNode @0;
  queryNode @1;
}

struct RegistryRequest {
  type @0 :RequestType;
  union {
    addNode :group {
      path @1 :Text;
      port @2 :UInt32;
    }

    queryNode :group {
      path @3 :Text;
    }
  }
}

struct RegistryResponse {
  code @0 :UInt32;
  errorMessage @1 :Text;
  union {
    queryNode :group {
      port @2 :UInt32;
    }
    addNode :group {
      port @3 :UInt32;
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
