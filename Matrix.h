//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef MATRIX_H_
#define MATRIX_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <boost/numeric/ublas/matrix.hpp>
#pragma GCC diagnostic pop

#include <ebbrt/Message.h>

#ifdef __FRONTEND__
#include <forward_list>
#endif // #ifdef __FRONTEND__

class Matrix : public ebbrt::Messagable<Matrix> {


 public:
  Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
         size_t y_tile, ebbrt::Messenger::NetworkId frontend_id);

  static Matrix& HandleFault(ebbrt::EbbId id);
  static ebbrt::Future<ebbrt::EbbRef<Matrix>>
  Create(size_t x_dim, size_t y_dim, size_t x_tile, size_t y_tile);

  static Matrix& LocalTileCreate(ebbrt::EbbId id, size_t x_dim,
				 size_t y_dim, size_t x_tile, size_t y_tile, 
				 ebbrt::Messenger::NetworkId frontend_id);

  double LocalTileGet(size_t x, size_t y);
  void  LocalTileSet(size_t x, size_t y, double val);
  void LocalTileRandomize();
  double LocalTileSum();
#if 0
  ebbrt::EbbRef<Matrix> LocalTileMultiply(ebbrt::EbbRef<Matrix> matrix);
#endif

  ebbrt::Future<double> Get(size_t x, size_t y);
  ebbrt::Future<void> Set(size_t x, size_t y, double val);
  ebbrt::Future<ebbrt::EbbRef<Matrix>> Multiply(ebbrt::EbbRef<Matrix> matrix);
  ebbrt::Future<void> Randomize();
  ebbrt::Future<double> Sum();

  ebbrt::Future<void> AllocNodes();
  void Destroy();

 private:
  ebbrt::EbbId my_id_;

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

  boost::numeric::ublas::matrix<double> matrix_;
  std::unique_ptr<ebbrt::IOBuf> left_data;
  std::unique_ptr<ebbrt::IOBuf> right_data;
  ebbrt::Messenger::NetworkId frontend_id_;

  friend ebbrt::Messagable<Matrix>;

#ifdef __FRONTEND__
  // FRONT END ONLY 
  Matrix(ebbrt::EbbId id, size_t x_dim, size_t y_dim, size_t x_tile,
         size_t y_tile);

  ebbrt::Future<void> MultiplyInternal(ebbrt::EbbRef<Matrix> a,
                                       ebbrt::EbbRef<Matrix> b);
  ebbrt::Future<std::vector<ebbrt::Messenger::NetworkId>> GetNodes();
  size_t TilesPerRow() const;
  size_t TilesPerCol() const;
  size_t Tiles() const;

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
#endif // #ifdef __FRONTEND__
};

#endif // #ifndef MATRIX_H_
