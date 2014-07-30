//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
#define MATRIX_SQUARE 0 
#define ITERATIONS 500

#ifndef __linux__
#include <ebbrt/Acpi.h>
#include <ebbrt/Clock.h>
void terminate() {   ebbrt::acpi::PowerOff(); }
#else
#include <iostream>
#include <memory>
#include <ebbrt/Context.h>
void terminate() { ebbrt::active_context->io_service_.stop(); }
#endif
#include <ebbrt/Debug.h>
#include <ebbrt/EbbAllocator.h>
#include "../Matrix.h"

void AppMain() 
{
  ebbrt::kprintf("%s", "Standalone Matrix App: START\n");

#ifndef __linux__
  auto uptime1_ns = ebbrt::clock::Uptime();
  auto uptime1_ms = std::chrono::duration_cast<std::chrono::milliseconds>(uptime1_ns).count();
  ebbrt::force_kprintf("Updtime pre-work: %d\n",uptime1_ms);
#endif

  if(MATRIX_SQUARE > 0){

    int count = 0;
    while( count < ITERATIONS){
      auto id = ebbrt::ebb_allocator->AllocateLocal();
      Matrix::LocalTileCreate(id, MATRIX_SQUARE, MATRIX_SQUARE, MATRIX_SQUARE, MATRIX_SQUARE,
                              ebbrt::Messenger::NetworkId("0000"));
      ebbrt::EbbRef<Matrix> m = ebbrt::EbbRef<Matrix>(id);
      m->LocalTileRandomize();
      m->LocalTileSum();
      count++;
    }
  }
  terminate();
}
