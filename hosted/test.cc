//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include <iostream>

#include "Matrix.h"
#include "ebb_matrix_helper.h"

int main() {
  activate_context();
  auto f = Matrix::Create(5, 5, 5, 5);
  auto m = wait_for_future<ebbrt::EbbRef<Matrix>>(&f);
  deactivate_context();

  activate_context();
  auto fv1 = m->Get(0, 0);
  auto v1 = wait_for_future<double>(&fv1);
  std::cout << v1 << std::endl;
  deactivate_context();
  

  activate_context();
  auto f2 = Matrix::Create(5, 5, 5, 5);
  auto m2 = wait_for_future<ebbrt::EbbRef<Matrix>>(&f2);
  deactivate_context();

  activate_context();
  auto f3 = m->Multiply(m2);
  auto m3 = wait_for_future<ebbrt::EbbRef<Matrix>>(&f3);
  deactivate_context();
  return 0;
}
