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
#include <ebbrt/NodeAllocator.h>

#include "Messages.capnp.h"

EBBRT_PUBLISH_TYPE(, Matrix);

Matrix::Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
               size_t y_tile)
    : ebbrt::Messagable<Matrix>(id), x_dim_(x_dim), y_dim_(y_dim),
      x_tile_(x_tile), y_tile_(y_tile), message_id_(0) {}

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

namespace {
class StringOutputStream : public kj::OutputStream {
 public:
  void write(const void* buffer, size_t size) override {
    str_.append(static_cast<const char*>(buffer), size);
  }
  void write(kj::ArrayPtr<const kj::ArrayPtr<const kj::byte>> pieces) override {
    for (auto& piece : pieces) {
      write(piece.begin(), piece.size());
    }
  }

  std::string& String() { return str_; }

 private:
  std::string str_;
};
}

ebbrt::Future<ebbrt::EbbRef<Matrix>>
Matrix::Create(size_t x_dim, size_t y_dim, size_t x_tile, size_t y_tile) {
  capnp::MallocMessageBuilder message;
  auto builder = message.initRoot<matrix::GlobalData>();
  auto netaddr = ebbrt::messenger->LocalNetworkId().ToBytes();
  builder.setNetworkAddress(capnp::Data::Reader(
      reinterpret_cast<const capnp::byte*>(netaddr.data()), netaddr.length()));
  builder.setXDim(x_dim);
  builder.setYDim(y_dim);
  builder.setXTile(x_tile);
  builder.setYTile(y_tile);
  auto id = ebbrt::ebb_allocator->Allocate();
  // FIXME(dschatz): Gmap should probably talk IOBufs
  StringOutputStream stream;
  capnp::writeMessage(stream, message);

  return ebbrt::global_id_map->Set(id, std::move(stream.String()))
      .Then([id](ebbrt::Future<void> f) {
        f.Get();
        return ebbrt::EbbRef<Matrix>(id);
      });
}

ebbrt::Future<double> Matrix::Get(size_t x, size_t y) {
  auto& f = GetNodeByXY(x, y);
  return f.Then([this, x, y](
      ebbrt::SharedFuture<ebbrt::Messenger::NetworkId> fut) {
    auto nid = fut.Get();
    lock_.lock();
    auto v = message_id_++;
    auto& p = get_map_[v];
    lock_.unlock();
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Request>();
    auto get_builder = builder.initGetRequest();
    get_builder.setId(v);
    get_builder.setX(x);
    get_builder.setY(y);
    SendMessage(nid, ebbrt::AppendHeader(message));
    return p.GetFuture();
  });
}

ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& Matrix::GetNode(size_t nid) {
  std::lock_guard<std::mutex> lock(lock_);
  auto it = node_map_.find(nid);
  if (it != node_map_.end()) {
    return it->second;
  }

  // Allocate new node
  auto pname = getenv("EBB_MATRIX_BM_PATH");
  if (!pname)
    throw std::runtime_error("EBB_MATRIX_BM_PATH environment variable not set");

  auto f = ebbrt::node_allocator->AllocateNode(pname);
  auto p = node_map_.emplace(nid, f.Share());
  return p.first->second;
}

ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>&
Matrix::GetNodeByXY(size_t x, size_t y) {
  // just makes sure to round up
  auto nodes_per_xdim =
      x_dim_ % x_tile_ ? x_dim_ / x_tile_ + 1 : x_dim_ / x_tile_;
  auto index = (y / y_tile_) * nodes_per_xdim + (x / x_tile_);
  return GetNode(index);
}

void Matrix::ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                            std::unique_ptr<ebbrt::IOBuf>&& buffer) {
  auto reader = ebbrt::IOBufMessageReader(std::move(buffer));
  auto reply = reader.getRoot<matrix::Reply>();

  switch (reply.which()) {
  case matrix::Reply::Which::GET_REPLY: {
    auto get_reply = reply.getGetReply();
    auto id = get_reply.getId();
    lock_.lock();
    auto it = get_map_.find(id);
    assert(it != get_map_.end());
    it->second.SetValue(get_reply.getVal());
    break;
  }
  case matrix::Reply::Which::RANDOMIZE_REPLY: { break; }
  }
}
