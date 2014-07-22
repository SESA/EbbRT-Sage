//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef MATRIX_H_
#define MATRIX_H_

#include <forward_list>

#include <ebbrt/Message.h>

class Matrix : public ebbrt::Messagable<Matrix> {
 public:
  Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
         size_t y_tile);
  static Matrix& HandleFault(ebbrt::EbbId id);
  static ebbrt::Future<ebbrt::EbbRef<Matrix>>
  Create(size_t x_dim, size_t y_dim, size_t x_tile, size_t y_tile);
  ebbrt::Future<double> Get(size_t x, size_t y);
  ebbrt::Future<void> Set(size_t x, size_t y, double val);
  ebbrt::Future<ebbrt::EbbRef<Matrix>> Multiply(ebbrt::EbbRef<Matrix> matrix);
  ebbrt::Future<void> Randomize();
  void Test();
  ebbrt::Future<double> Sum();
  ebbrt::Future<void> AllocNodes();
  void Destroy();

 private:
  ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNode(size_t nid);
  ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>& GetNodeByXY(size_t x,
                                                                size_t y);
  void ReceiveMessage(ebbrt::Messenger::NetworkId nid,
                      std::unique_ptr<ebbrt::IOBuf>&& buffer);
  ebbrt::Future<void> MultiplyInternal(ebbrt::EbbRef<Matrix> a,
                                       ebbrt::EbbRef<Matrix> b);
  ebbrt::Future<std::vector<ebbrt::Messenger::NetworkId>> GetNodes();
  size_t TilesPerRow() const;
  size_t TilesPerCol() const;
  size_t Tiles() const;

  ebbrt::EbbId my_id_;
  size_t x_dim_;
  size_t y_dim_;
  size_t x_tile_;
  size_t y_tile_;
  std::unordered_map<size_t, ebbrt::SharedFuture<ebbrt::Messenger::NetworkId>>
  node_map_;
  std::unordered_map<uint32_t, ebbrt::Promise<double>> get_map_;
  std::unordered_map<uint32_t, ebbrt::Promise<void>> set_map_;
  std::unordered_map<uint32_t, std::pair<ebbrt::Promise<void>, size_t>>
  randomize_map_;
  std::unordered_map<
      uint32_t, std::tuple<ebbrt::Promise<double>, size_t, size_t>> sum_map_;
  std::forward_list<uint16_t> nodes_;
  std::mutex lock_;
  uint32_t message_id_;
  ebbrt::EventManager::EventContext* multiply_activate_{nullptr};
  std::atomic_size_t multiply_completes;

  friend ebbrt::Messagable<Matrix>;
};

#endif  // MATRIX_H_
