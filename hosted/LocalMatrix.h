//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#ifndef LOCALMATRIX_H_
#define LOCALMATRIX_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <boost/numeric/ublas/matrix.hpp>
#pragma GCC diagnostic pop

#include <ebbrt/EbbRef.h>
#include <ebbrt/Future.h>

class LocalMatrix {
 public:
  LocalMatrix(size_t x_dim, size_t y_dim);
  static LocalMatrix& HandleFault(ebbrt::EbbId id);
  static ebbrt::Future<ebbrt::EbbRef<LocalMatrix>> CreateLocal(size_t x_dim,
                                                               size_t y_dim);
  ebbrt::Future<void> Randomize();
  void Destroy();

 private:
  size_t x_dim_;
  size_t y_dim_;
  boost::numeric::ublas::matrix<double> matrix_;
};

#endif  // LOCALMATRIX_H_
