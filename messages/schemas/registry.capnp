@0x9246579439031e06;

enum RequestType {
  addNode @0;
  queryNode @1;
  addHost @2;
  heartbeat @3;
}

struct TopicEntry {
  name @0 :Text;
  host @1 :Text;
  port @2 :UInt32;
}

struct TopicsListRequest {
  includeInternal @0 :Bool;
}

struct TopicsListResponse {
  topics @0 :List(TopicEntry);
}

struct Interface {
  ip      @0 :UInt32;
  network @1 :UInt32;
}

struct RegistryRequest {
  type     @0 :RequestType;
  path     @1 :Text;
  networks @6 :List(Interface);

  union {
    addNode @2 :Void;
    queryNode @3 :Void;
    heartbeat @7 :Void;
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
  heartbeat @2;
  nodeChange @3;
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
    heartbeat @5 :Void;
  }
}
