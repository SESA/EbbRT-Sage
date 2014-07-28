//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef __linux__
#include <ebbrt/Acpi.h>
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
  auto id = ebbrt::ebb_allocator->AllocateLocal();

  Matrix::LocalTileCreate(id, 10,10,10,10,
			    ebbrt::Messenger::NetworkId("0000"));
  ebbrt::EbbRef<Matrix> m = ebbrt::EbbRef<Matrix>(id);

  ebbrt::kprintf("0: m[0,0]=%f\n", m->LocalTileGet(0,0));
  m->LocalTileSet(0,0,1.0);
  ebbrt::kprintf("1: Post Set: m[0,0]=%f\n", m->LocalTileGet(0,0));

  m->LocalTileRandomize();
  ebbrt::kprintf("2: Post Randomize: m[0,0]=%f\n", m->LocalTileGet(0,0));

  ebbrt::kprintf("3: Sum: %f\n", m->LocalTileSum());

  ebbrt::kprintf("%s", "Standalone Matrix App: END");

  terminate();
}


