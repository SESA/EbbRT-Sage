//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include <iostream>

#include "Matrix.h"
#include "ebb_matrix_helper.h"

int main() {
  activate_context();
  auto f = Matrix::Create(1000, 1000, 200, 200);
  auto m = wait_for_future<ebbrt::EbbRef<Matrix>>(&f);
  deactivate_context();

  activate_context();
  auto ref = m.GetPointer();
  auto fut = ref->Get(3, 4);
  deactivate_context();
  auto ret = wait_for_future<double>(&fut);
  std::cout << ret << std::endl;
  
  return 0;
}
