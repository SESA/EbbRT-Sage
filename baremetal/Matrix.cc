//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Matrix.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <boost/numeric/ublas/operation.hpp>
#pragma GCC diagnostic pop
#include <capnp/serialize.h>

#include <ebbrt/Acpi.h>
#include <ebbrt/Debug.h>
#include <ebbrt/CapnpMessage.h>
#include <ebbrt/EbbAllocator.h>
#include <ebbrt/GlobalIdMap.h>
#include <ebbrt/LocalIdMap.h>
#include <ebbrt/Random.h>

#include "Messages.capnp.h"

EBBRT_PUBLISH_TYPE(, Matrix);

void AppStart(){
  ebbrt::kprintf("App Started\n");

}

Matrix::Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
               size_t y_tile, ebbrt::Messenger::NetworkId frontend_id)
    : ebbrt::Messagable<Matrix>(id), x_dim_(x_dim), y_dim_(y_dim),
      x_tile_(x_tile), y_tile_(y_tile),
      matrix_(boost::numeric::ublas::zero_matrix<double>(
          std::min(x_tile, x_dim), std::min(y_tile, y_dim))),
      frontend_id_(frontend_id) {}

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
  auto p = new Matrix(id, data.getXDim(), data.getYDim(), data.getXTile(),
                      data.getYTile(), nid);

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

void Matrix::DoRandom(){
    std::default_random_engine generator(ebbrt::random::Get());
    std::uniform_real_distribution<double> distribution(-1, 1);
    for (auto& d : matrix_.data()) {
      d = distribution(generator);
    }
}

double Matrix::DoSum() {

  double v = 0;
  for (auto& d : matrix_.data()) {
    v += d;
  }
  return v;
}

void Matrix::ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                            std::unique_ptr<ebbrt::IOBuf>&& buffer) {
  auto reader = ebbrt::IOBufMessageReader(std::move(buffer));
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
    get_builder.setVal(matrix_(x, y));
    SendMessage(nid, AppendHeader(message));
    break;
  }
  case matrix::Request::Which::SET_REQUEST: {
    auto set_request = request.getSetRequest();
    auto id = set_request.getId();
    auto x = set_request.getX() % x_tile_;
    auto y = set_request.getY() % y_tile_;
    matrix_(x, y) = set_request.getVal();
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
    DoRandom();
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
  case matrix::Request::Which::TEST_REQUEST: {
    DoRandom();
    DoSum();
    ebbrt::acpi::PowerOff();
  }
  case matrix::Request::Which::SUM_REQUEST: {
    auto sum_request = request.getSumRequest();
    auto id = sum_request.getId();
    double v = DoSum();
    ebbrt::IOBufMessageBuilder message;
    auto builder = message.initRoot<matrix::Reply>();
    auto sum_builder = builder.initSumReply();
    sum_builder.setId(id);
    sum_builder.setVal(v);
    SendMessage(nid, AppendHeader(message));
    break;
  }
   default: {
    ebbrt::kprintf("request not found");
    break;
  }
  }
}
