@0x96ac190771c9b4e8;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("matrix");

struct GetRequest {
  id @0 :UInt32;
  x @1 :UInt32;
  y @2 :UInt32;
}

struct SetRequest {
  id @0 :UInt32;
  x @1 :UInt32;
  y @2 :UInt32;
  val @3 :Float64;
}

struct RandomizeRequest {
  id @0 :UInt32;
}

struct SendDataRequest {
  row @0 :UInt32;
  col @1 :UInt32;
  id @2 :UInt32;
  left @3 :Bool;
  nodes @4 :List(Data);
}

struct SendData {
  row @0 :UInt32;
  col @1 :UInt32;
  left @2 :UInt32;
  matrix @3 :Data;
}       


struct GetReply {
  id @0 :UInt32;
  val @1 :Float64;
}

struct SetReply {
  id @0 :UInt32;
}

struct RandomizeReply {
  id @0 :UInt32;
}

struct MultiplyReply {
  v @0 :Void;
}

struct Request {
  union {
    getRequest @0 :GetRequest;
    setRequest @1 :SetRequest;
    randomizeRequest @2 :RandomizeRequest;
    sendDataRequest @3 :SendDataRequest;
    sendData @4 :SendData;
  }
}

struct Reply {
  union {
    getReply @0 :GetReply;
    setReply @1 :SetReply;
    randomizeReply @2 :RandomizeReply;
    multiplyReply @3 :MultiplyReply;
  }
}

struct GlobalData {
  networkAddress @0 :Data;
  xDim @1 :UInt64;
  yDim @2 :UInt64;
  xTile @3 :UInt64;
  yTile @4 :UInt64;
}