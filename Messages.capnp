@0x96ac190771c9b4e8;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("matrix");

struct GetRequest {
  id @0 :UInt32;
  x @1 :UInt32;
  y @2 :UInt32;
}

struct RandomizeRequest {
  id @0 :UInt32;
}

struct GetReply {
  id @0 :UInt32;
  val @1 :Float64;
}

struct RandomizeReply {
  id @0 :UInt32;
}

struct Request {
  union {
    getRequest @0 :GetRequest;
    randomizeRequest @1 :RandomizeRequest;
  }
}

struct Reply {
  union {
    getReply @0 :GetReply;
    randomizeReply @1 :RandomizeReply;
  }
}

struct GlobalData {
  networkAddress @0 :Data;
  xDim @1 :UInt64;
  yDim @2 :UInt64;
  xTile @3 :UInt64;
  yTile @4 :UInt64;
}