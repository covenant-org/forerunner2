@0xcf066f32651a0194;

struct PointYaw{
  x   @0 :Float32;
  y   @1 :Float32;
  z   @2 :Float32;
  r   @3 :Float32;
}

struct Attitude{
    q      @0 :List(Float32);
    thrust @1 :Float32;
}

struct Command{
 union {
    waypoint  @0 :PointYaw;
    takeoff   :group{
      altitude  @1 :UInt8;
    }
    land      @2 :Void;
    arm       @3 :Void;
    disarm    @4 :Void;
    accel     @5 :PointYaw;
    velocity  @6 :PointYaw;
    manual    @7 :PointYaw;
    offboard   :group{
      enable  @8 :Bool;
    }
    gotoLocation :group{
      latitude  @9 :Float64;
      longitude @10 :Float64;
      altitude  @11 :Float32;
      yaw       @12 :Float32;
    }
    uploadMission @13 :UploadMission;
    startMission @14 :Void;
    pauseMission @15 :Void;
    clearMission @16 :Void;
    setActuators @17 :List(Float32);
    setAttitude  @18 :Attitude;
  }
}


struct UploadMission {
  waypoints @0 :List(MissionItem);
}

struct MissionItem {
  latitude @0 :Float64;
  longitude @1 :Float64;
  relativeAltitude @2 :Float32;
  speed @3 :Float32;
  isFlyThrough @4 :Bool;
  gimbalPitch @5 :Float32;
  gimbalYaw @6 :Float32;
  cameraAction @7 :CameraAction;
  loiterTime @8 :Float32;
  cameraPhotoInterval @9 :Float32;
  
  enum CameraAction {
    none @0;
    takePhoto @1;
    startPhotoInterval @2;
    stopPhotoInterval @3;
    startVideo @4;
    stopVideo @5;
  }
}

struct Goal{
  union{
    relative  @0 :PointYaw;
    coords    @1 :PointYaw;
    latlon    @2 :PointYaw;
  }
}

struct ControlMetrics{
    qe      @0 :List(Float32);
    qd      @1 :List(Float32);
    thrust  @2 :List(Float32);
    pwm     @3 :List(Float32);
    fu      @4 :List(Float32);
    error   @5 :List(Float32);
  }
