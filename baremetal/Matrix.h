//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef MATRIX_H_
#define MATRIX_H_

#include <ebbrt/Message.h>

class Matrix : public ebbrt::Messagable<Matrix> {
 public:
  Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
         size_t y_tile);
  static Matrix& HandleFault(ebbrt::EbbId id);
  static ebbrt::Future<ebbrt::EbbRef<Matrix>>
  Create(size_t x_dim, size_t y_dim, size_t x_tile, size_t y_tile);
  ebbrt::Future<double> Get(size_t x, size_t y);

 private:
  ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNode(size_t nid);
  ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNodeByXY(size_t x,
                                                                size_t y);
  void ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                      std::unique_ptr<ebbrt::IOBuf>&& buffer);

  size_t x_dim_;
  size_t y_dim_;
  size_t x_tile_;
  size_t y_tile_;
  std::unordered_map<size_t, ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>>
  node_map_;

  friend ebbrt::Messagable<Matrix>;
};

#endif  // MATRIX_H_
