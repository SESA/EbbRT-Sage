//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "../Matrix.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <boost/numeric/ublas/operation.hpp>
#pragma GCC diagnostic pop
#include <capnp/serialize.h>

#include <ebbrt/CapnpMessage.h>
#include <ebbrt/EbbAllocator.h>
#include <ebbrt/GlobalIdMap.h>
#include <ebbrt/LocalIdMap.h>
#include <ebbrt/Random.h>

#include "Messages.capnp.h"

EBBRT_PUBLISH_TYPE(, Matrix);

Matrix::Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
               size_t y_tile, ebbrt::Messenger::NetworkId frontend_id)
    : ebbrt::Messagable<Matrix>(id), my_id_(id), x_dim_(x_dim), y_dim_(y_dim),
      x_tile_(x_tile), y_tile_(y_tile),
      matrix_(boost::numeric::ublas::zero_matrix<double>(
          std::min(x_tile, x_dim), std::min(y_tile, y_dim))),
      frontend_id_(frontend_id) {}

void Matrix::LocalTileDelete() { delete this; }

Matrix& Matrix::LocalTileCreate(ebbrt::EbbId id, size_t x_dim, size_t y_dim,
                                size_t x_tile, size_t y_tile,
                                ebbrt::Messenger::NetworkId frontend_id) {
  // try to construct
  auto p = new Matrix(id, x_dim, y_dim, x_tile, y_tile, frontend_id);

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
  (void)found;  // unused variable
  auto& m = *boost::any_cast<Matrix*>(accessor->second);
  ebbrt::EbbRef<Matrix>::CacheRef(id, m);
  return m;
}

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

  const auto& net_addr = data.getNetworkAddress();
  auto nid = ebbrt::Messenger::NetworkId(std::string(
      reinterpret_cast<const char*>(net_addr.begin()), net_addr.size()));
  // try to construct
  return Matrix::LocalTileCreate(id, data.getXDim(), data.getYDim(),
                                 data.getXTile(), data.getYTile(), nid);
}

double Matrix::LocalTileGet(size_t x, size_t y) { return matrix_(x, y); }

void Matrix::LocalTileSet(size_t x, size_t y, double val) {
  matrix_(x, y) = val;
}

void Matrix::LocalTileRandomize() {
  std::default_random_engine generator(13);
  std::uniform_real_distribution<double> distribution(-1, 1);
  for (auto& d : matrix_.data()) {
    d = distribution(generator);
  }
}

void Matrix::LocalTileTouch(double v) {
  static double val = 1;
  for (auto it = matrix_.begin1(); it < matrix_.end1();
       std::advance(it, 512)) {
    const_cast<volatile double&>(*it) = v + val++;
  }
}

double Matrix::LocalTileSum() {
  double v = 0;
  for (auto& d : matrix_.data()) {
    v += d;
  }
  return v;
}

#if 0
ebbrt::EbbRef<Matrix> LocalTileMultiply(ebbrt::EbbRef<Matrix> matrix)
{
}
#endif

void Matrix::ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                            std::unique_ptr<ebbrt::IOBuf>&& buffer) {
  auto reader = ebbrt::IOBufMessageReader(std::move(buffer));
#ifdef __FRONTEND__
  auto reply = reader.getRoot<matrix::Reply>();
  switch (reply.which()) {
  case matrix::Reply::Which::GET_REPLY: {
    auto get_reply = reply.getGetReply();
    auto id = get_reply.getId();
    std::lock_guard<std::mutex> lock(lock_);
    auto it = get_map_.find(id);
    assert(it != get_map_.end());
    it->second.SetValue(get_reply.getVal());
    get_map_.erase(it);
    break;
  }
  case matrix::Reply::Which::SET_REPLY: {
    auto set_reply = reply.getSetReply();
    auto id = set_reply.getId();
    std::lock_guard<std::mutex> lock(lock_);
    auto it = set_map_.find(id);
    assert(it != set_map_.end());
    it->second.SetValue();
    set_map_.erase(it);
    break;
  }
  case matrix::Reply::Which::RANDOMIZE_REPLY: {
    auto randomize_reply = reply.getRandomizeReply();
    auto id = randomize_reply.getId();
    std::lock_guard<std::mutex> lock(lock_);
    auto it = randomize_map_.find(id);
    assert(it != randomize_map_.end());
    auto v = --it->second.second;
    if (v == 0) {
      it->second.first.SetValue();
      randomize_map_.erase(it);
    }
    break;
  }
  case matrix::Reply::Which::SUM_REPLY: {
    auto sum_reply = reply.getSumReply();
    auto id = sum_reply.getId();
    std::lock_guard<std::mutex> lock(lock_);
    auto it = sum_map_.find(id);
    assert(it != sum_map_.end());
    auto v = --std::get<1>(it->second);
    std::get<2>(it->second) += sum_reply.getVal();
    if (v == 0) {
      std::get<0>(it->second).SetValue(std::get<2>(it->second));
      sum_map_.erase(it);
    }
    break;
  }
  case matrix::Reply::Which::MULTIPLY_REPLY: {
    auto v = multiply_completes.fetch_sub(1);
    if (v == 1)
      ebbrt::event_manager->ActivateContext(std::move(*multiply_activate_));
    break;
  }
  }
#else
  auto request = reader.getRoot<matrix::Request>();
  switch (request.which()) {
  case matrix::Request::Which::GET_REQUEST: {
    auto get_request = request.getGetRequest();
    auto id = get_request.getId();
    auto x = get_request.getX() % x_tile_;
    auto y = get_request.getY() % y_tile_;
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto get_builder = builder.initGetReply();
    get_builder.setId(id);

    get_builder.setVal(LocalTileGet(x, y));

    SendMessage(nid, AppendHeader(message));
    break;
  }
  case matrix::Request::Which::SET_REQUEST: {
    auto set_request = request.getSetRequest();
    auto id = set_request.getId();
    auto x = set_request.getX() % x_tile_;
    auto y = set_request.getY() % y_tile_;

    LocalTileSet(x, y, set_request.getVal());

    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto set_builder = builder.initSetReply();
    set_builder.setId(id);
    SendMessage(nid, AppendHeader(message));
    break;
  }
  case matrix::Request::Which::RANDOMIZE_REQUEST: {
    auto randomize_request = request.getRandomizeRequest();
    auto id = randomize_request.getId();

    LocalTileRandomize();

    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto randomize_builder = builder.initRandomizeReply();
    randomize_builder.setId(id);
    SendMessage(nid, AppendHeader(message));
    break;
  }
  case matrix::Request::Which::SEND_DATA_REQUEST: {
    auto send_request = request.getSendDataRequest();
    auto row = send_request.getRow();
    auto col = send_request.getCol();
    auto id = send_request.getId();
    auto left = send_request.getLeft();
    auto nodes = send_request.getNodes();
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Request>();
    auto send_data = builder.initSendData();
    send_data.setRow(row);
    send_data.setCol(col);
    send_data.setLeft(left);
    send_data.setMatrix(capnp::Data::Reader(
        reinterpret_cast<const capnp::byte*>(&(*matrix_.data().begin())),
        matrix_.data().size() * sizeof(double)));
    auto buf = AppendHeader(message);
    for (const auto& node : nodes) {
      auto nid = ebbrt::Messenger::NetworkId(std::string(
          reinterpret_cast<const char*>(node.begin()), node.size()));
      SendMessage(id, nid, buf->Clone());
    }
    // SendMessage(frontend_id_, std::move(buf));
    break;
  }
  case matrix::Request::Which::SEND_DATA: {
    auto send_data = request.getSendData();
    if (send_data.getLeft()) {
      left_data = reader.GetBuf();
    } else {
      right_data = reader.GetBuf();
    }

    if (left_data && right_data) {
      auto left_reader = ebbrt::IOBufMessageReader(std::move(left_data));
      auto left_request = left_reader.getRoot<matrix::Request>();
      auto left_send_data = left_request.getSendData();
      auto left_mat_data =
          reinterpret_cast<const double*>(left_send_data.getMatrix().begin());
      boost::numeric::ublas::unbounded_array<double> left_a(x_tile_ * y_tile_);
      for (auto& d : left_a) {
        d = *left_mat_data++;
      }
      boost::numeric::ublas::matrix<double> left_mat(x_tile_, y_tile_, left_a);
      auto right_reader = ebbrt::IOBufMessageReader(std::move(right_data));
      auto right_request = right_reader.getRoot<matrix::Request>();
      auto right_send_data = right_request.getSendData();
      auto right_mat_data =
          reinterpret_cast<const double*>(right_send_data.getMatrix().begin());
      boost::numeric::ublas::unbounded_array<double> right_a(x_tile_ * y_tile_);
      for (auto& d : right_a) {
        d = *right_mat_data++;
      }
      boost::numeric::ublas::matrix<double> right_mat(x_tile_, y_tile_,
                                                      right_a);
      boost::numeric::ublas::opb_prod(left_mat, right_mat, matrix_, false);
      ebbrt::IOBufMessageBuilder message;
      auto builder = message.initRoot<matrix::Reply>();
      builder.initMultiplyReply();
      SendMessage(frontend_id_, AppendHeader(message));
    }
    break;
  }
  case matrix::Request::Which::SUM_REQUEST: {
    auto sum_request = request.getSumRequest();
    auto id = sum_request.getId();

    double v = LocalTileSum();

    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto sum_builder = builder.initSumReply();
    sum_builder.setId(id);
    sum_builder.setVal(v);
    SendMessage(nid, AppendHeader(message));
    break;
  }
  }
#endif  // #ifdef __FRONTEND__ #else
}

#ifdef __FRONTEND__
#include <ebbrt/NodeAllocator.h>

Matrix::Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
               size_t y_tile)
    : ebbrt::Messagable<Matrix>(id), my_id_(id), x_dim_(x_dim), y_dim_(y_dim),
      x_tile_(x_tile), y_tile_(y_tile), message_id_(0) {}

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

ebbrt::Future<void> Matrix::Set(size_t x, size_t y, double val) {
  auto& f = GetNodeByXY(x, y);
  return f.Then([this, x, y, val](
      ebbrt::SharedFuture<ebbrt::Messenger::NetworkId> fut) {
    auto nid = fut.Get();
    lock_.lock();
    auto v = message_id_++;
    auto& p = set_map_[v];
    lock_.unlock();
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Request>();
    auto set_builder = builder.initSetRequest();
    set_builder.setId(v);
    set_builder.setX(x);
    set_builder.setY(y);
    set_builder.setVal(val);
    SendMessage(nid, ebbrt::AppendHeader(message));
    return p.GetFuture();
  });
}

ebbrt::Future<void> Matrix::Randomize() {
  lock_.lock();
  auto v = message_id_++;
  auto& p = randomize_map_[v];
  lock_.unlock();
  p.second = Tiles();
  ebbrt::IOBufMessageBuilder message;
  auto builder = message.initRoot<matrix::Request>();
  auto randomize_builder = builder.initRandomizeRequest();
  randomize_builder.setId(v);
  auto buf = ebbrt::AppendHeader(message);
  for (size_t i = 0; i < Tiles(); ++i) {
    GetNode(i).Then(
        MoveBind([this](std::unique_ptr<ebbrt::IOBuf> m,
                        ebbrt::SharedFuture<ebbrt::Messenger::NetworkId> fut) {
                   auto nid = fut.Get();
                   SendMessage(nid, std::move(m));
                 },
                 buf->Clone()));
  }
  return p.first.GetFuture();
}

ebbrt::Future<double> Matrix::Sum() {
  lock_.lock();
  auto v = message_id_++;
  auto& p = sum_map_[v];
  lock_.unlock();
  std::get<1>(p) = Tiles();
  ebbrt::IOBufMessageBuilder message;
  auto builder = message.initRoot<matrix::Request>();
  auto sum_builder = builder.initSumRequest();
  sum_builder.setId(v);
  auto buf = ebbrt::AppendHeader(message);
  for (size_t i = 0; i < Tiles(); ++i) {
    GetNode(i).Then(
        MoveBind([this](std::unique_ptr<ebbrt::IOBuf> m,
                        ebbrt::SharedFuture<ebbrt::Messenger::NetworkId> fut) {
                   auto nid = fut.Get();
                   SendMessage(nid, std::move(m));
                 },
                 buf->Clone()));
  }
  return std::get<0>(p).GetFuture();
}

void Matrix::Destroy() {
  for (auto& n : nodes_) {
    ebbrt::node_allocator->FreeNode(n);
  }
}

ebbrt::Future<void> Matrix::AllocNodes() {
  return GetNodes().Then([](
      ebbrt::Future<std::vector<ebbrt::Messenger::NetworkId>> f) { f.Get(); });
}

ebbrt::Future<ebbrt::EbbRef<Matrix>>
Matrix::Multiply(ebbrt::EbbRef<Matrix> matrix) {
  if (y_dim_ != matrix->x_dim_)
    return ebbrt::MakeFailedFuture<ebbrt::EbbRef<Matrix>>(
        std::make_exception_ptr(std::runtime_error("Dimension mismatch")));

  if (y_tile_ != matrix->x_tile_)
    return ebbrt::MakeFailedFuture<ebbrt::EbbRef<Matrix>>(
        std::make_exception_ptr(std::runtime_error("Tiling mismatch")));

  auto e = Matrix::Create(x_dim_, matrix->y_dim_, x_tile_, matrix->y_tile_);
  return e.Then([this, matrix](ebbrt::Future<ebbrt::EbbRef<Matrix>> f) {
    auto new_mat = f.Get();
    return new_mat->MultiplyInternal(ebbrt::EbbRef<Matrix>(my_id_), matrix)
        .Then([new_mat](ebbrt::Future<void> f) {
          f.Get();
          return new_mat;
        });
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

  auto node_desc = ebbrt::node_allocator->AllocateNode(pname);
  nodes_.emplace_front(node_desc.NodeId());
  auto p = node_map_.emplace(nid, node_desc.NetworkId().Share());
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

ebbrt::Future<void> Matrix::MultiplyInternal(ebbrt::EbbRef<Matrix> a,
                                             ebbrt::EbbRef<Matrix> b) {
  auto f1 = a->GetNodes();
  auto f2 = b->GetNodes();
  return ebbrt::WhenAll(std::move(f1), std::move(f2), GetNodes()).Then([a, b,
                                                                        this](
      ebbrt::Future<std::vector<std::vector<ebbrt::Messenger::NetworkId>>> f) {
    auto& v = f.Get();
    for (auto& inner_v : v) {
      std::cout << "Matrix nodes: " << std::endl;
      for (auto& nid : inner_v) {
        std::cout << nid.ToString() << std::endl;
      }
    }
    auto n_rounds = a->TilesPerRow();
    ebbrt::EventManager::EventContext context;
    multiply_activate_ = &context;
    for (size_t i = 0; i < n_rounds; ++i) {
      multiply_completes = Tiles();
      for (size_t j = 0; j < a->TilesPerCol(); ++j) {
        ebbrt::IOBufMessageBuilder message;
        auto builder = message.initRoot<matrix::Request>();
        auto send_builder = builder.initSendDataRequest();
        send_builder.setRow(j);
        send_builder.setCol(i);
        send_builder.setId(my_id_);
        send_builder.setLeft(true);
        auto node_builder = send_builder.initNodes(TilesPerRow());
        for (size_t k = 0; k < TilesPerRow(); ++k) {
          auto netaddr = v[2][k * TilesPerCol() + j].ToBytes();
          node_builder.set(
              k, capnp::Data::Reader(
                     reinterpret_cast<const capnp::byte*>(netaddr.data()),
                     netaddr.length()));
        }
        SendMessage(a->my_id_, v[0][i * a->TilesPerCol() + j],
                    ebbrt::AppendHeader(message));
      }
      for (size_t j = 0; j < b->TilesPerRow(); ++j) {
        ebbrt::IOBufMessageBuilder message;
        auto builder = message.initRoot<matrix::Request>();
        auto send_builder = builder.initSendDataRequest();
        send_builder.setRow(i);
        send_builder.setCol(j);
        send_builder.setId(my_id_);
        send_builder.setLeft(false);
        auto node_builder = send_builder.initNodes(TilesPerCol());
        for (size_t k = 0; k < TilesPerCol(); ++k) {
          auto netaddr = v[2][j * TilesPerCol() + k].ToBytes();
          node_builder.set(
              k, capnp::Data::Reader(
                     reinterpret_cast<const capnp::byte*>(netaddr.data()),
                     netaddr.length()));
        }
        SendMessage(b->my_id_, v[1][j * b->TilesPerCol() + i],
                    ebbrt::AppendHeader(message));
      }
      ebbrt::event_manager->SaveContext(*multiply_activate_);
    }
  });
}

ebbrt::Future<std::vector<ebbrt::Messenger::NetworkId>> Matrix::GetNodes() {
  auto num_nodes = Tiles();
  auto v = std::vector<ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>>();
  v.reserve(num_nodes);
  for (unsigned i = 0; i < num_nodes; ++i) {
    v.emplace_back(GetNode(i));
  }
  return WhenAll(v.begin(), v.end());
}

size_t Matrix::TilesPerRow() const { return 1 + ((y_dim_ - 1) / y_tile_); }
size_t Matrix::TilesPerCol() const { return 1 + ((x_dim_ - 1) / x_tile_); }
size_t Matrix::Tiles() const { return TilesPerRow() * TilesPerCol(); }
#endif  // #ifdef __FRONTEND__
