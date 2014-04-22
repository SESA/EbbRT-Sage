//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "LocalMatrix.h"

#include <ebbrt/EbbAllocator.h>

// KLUDGE
namespace {
size_t x, y;
}

LocalMatrix::LocalMatrix(size_t x_dim, size_t y_dim)
    : x_dim_(x_dim), y_dim_(y_dim),
      matrix_(boost::numeric::ublas::zero_matrix<double>(x_dim_, y_dim_)) {}

LocalMatrix& LocalMatrix::HandleFault(ebbrt::EbbId id) {
  auto p = new LocalMatrix(x, y);
  ebbrt::EbbRef<LocalMatrix>::CacheRef(id, *p);
  return *p;
}

ebbrt::Future<ebbrt::EbbRef<LocalMatrix>>
LocalMatrix::CreateLocal(size_t x_dim, size_t y_dim) {
  x = x_dim;
  y = y_dim;
  auto id = ebbrt::ebb_allocator->Allocate();
  return ebbrt::MakeReadyFuture<ebbrt::EbbRef<LocalMatrix>>(id);
}

ebbrt::Future<void> LocalMatrix::Randomize() {
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1, 1);
  for (auto& d : matrix_.data()) {
    d = distribution(generator);
  }
  return ebbrt::MakeReadyFuture<void>();
}

void LocalMatrix::Destroy() {
  delete this;
}
