//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
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

char *nextArg(char *str) {
  char *aptr = str;
  
  while (aptr && *aptr != 0  && *aptr != ' ') aptr++;

  while (aptr && *aptr != 0  && *aptr == ' ') aptr++;
  
  return aptr;
}

void AppMain() 
{
  ebbrt::kprintf("%s %s\n", "Standalone Matrix App: START: ",ebbrt::runtime::bootcmdline);
#ifndef __linux__
  auto uptime1_ns = ebbrt::clock::Uptime();
  auto uptime1_ms = std::chrono::duration_cast<std::chrono::milliseconds>(uptime1_ns).count();
  ebbrt::force_kprintf("Updtime pre-work: %d\n",uptime1_ms);
#endif

  char *arg = nextArg(ebbrt::runtime::bootcmdline);
  int matsize = atoi(arg);
  arg = nextArg(arg);
  int repcnt = atoi(arg);

  ebbrt::kprintf("matsize=%d repcnt=%d\n", matsize, repcnt);

  if (matsize > 0) {
    int count = 0;
    while( count < repcnt){
      auto id = ebbrt::ebb_allocator->AllocateLocal();
      Matrix::LocalTileCreate(id, matsize, matsize, matsize, matsize,
                              ebbrt::Messenger::NetworkId("0000"));
      ebbrt::EbbRef<Matrix> m = ebbrt::EbbRef<Matrix>(id);
      m->LocalTileRandomize();
      m->LocalTileSum();
      count++;
    }
  }
  terminate();
}
