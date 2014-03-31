//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Matrix.h"

#include <capnp/serialize.h>

#include <ebbrt/CapnpMessage.h>
#include <ebbrt/EbbAllocator.h>
#include <ebbrt/GlobalIdMap.h>
#include <ebbrt/LocalIdMap.h>

#include "Messages.capnp.h"

EBBRT_PUBLISH_TYPE(, Matrix);

Matrix::Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
               size_t y_tile)
  : ebbrt::Messagable<Matrix>(id), x_dim_(x_dim), y_dim_(y_dim),
    x_tile_(x_tile), y_tile_(y_tile) {}

Matrix& Matrix::HandleFault(ebbrt::EbbId id) {
  {
    ebbrt::LocalIdMap::ConstAccessor accessor;
    auto found = ebbrt::local_id_map->Find(accessor, id);
    if (found) {
      auto& m = *boost::any_cast<Matrix*>(accessor->second);
      ebbrt::EbbRef<Matrix>::CacheRef(id, m);
      return m;
    }
  }

  auto f = ebbrt::global_id_map->Get(id).Block();
  auto& str = f.Get();
  auto aptr = kj::ArrayPtr<const capnp::word>(
      reinterpret_cast<const capnp::word*>(str.data()),
      str.length() / sizeof(const capnp::word));
  auto reader = capnp::FlatArrayMessageReader(aptr);
  auto data = reader.getRoot<matrix::GlobalData>();

  // try to construct
  auto p = new Matrix(id, data.getXDim(), data.getYDim(), data.getXTile(),
                      data.getYTile());

  auto inserted = ebbrt::local_id_map->Insert(std::make_pair(id, p));
  if (inserted) {
    ebbrt::EbbRef<Matrix>::CacheRef(id, *p);
    return *p;
  }

  // raced, delete the new matrix
  delete p;
  ebbrt::LocalIdMap::ConstAccessor accessor;
  auto found = ebbrt::local_id_map->Find(accessor, id);
  assert(found);
  auto& m = *boost::any_cast<Matrix*>(accessor->second);
  ebbrt::EbbRef<Matrix>::CacheRef(id, m);
  return m;
}

ebbrt::Future<double> Matrix::Get(size_t x, size_t y) {
  return ebbrt::MakeReadyFuture<double>(0);
}

// ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNode(size_t nid) {
//   auto it = node_map_.find(nid);
//   if (it != node_map_.end()) {
//     return it->second;
//   }

//   // Allocate new node
//   auto pname = getenv("EBB_MATRIX_BM_PATH");
//   if (!pname)
//     throw std::runtime_error("EBB_MATRIX_BM_PATH environment variable not
// set");

//   ebbrt::node_allocator->AllocateNode(pname);
// }

// ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNodeByXY(size_t x,
//                                                               size_t y) {
//   // just makes sure to round up
//   auto nodes_per_xdim = x_dim % x_tile ? x_dim / x_tile + 1 : x_dim / x_tile;
//   auto index = (y / y_tile) * nodes_per_xdim + (x / x_tile);
//   return GetNode(index)
// }

void Matrix::ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                            std::unique_ptr<ebbrt::IOBuf>&& buffer) {
  auto reader = ebbrt::IOBufMessageReader(std::move(buffer));
  auto request = reader.getRoot<matrix::Request>();
  switch (request.which()) {
  case matrix::Request::Which::GET_REQUEST: {
    auto get_request = request.getGetRequest();
    auto id = get_request.getId();
    auto x = get_request.getX();
    auto y = get_request.getY();
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto get_builder = builder.initGetReply();
    get_builder.setId(id);
    get_builder.setVal(x * y);
    SendMessage(nid, AppendHeader(message));
    break;
  }
  case matrix::Request::Which::RANDOMIZE_REQUEST: { break; }
  }
}
