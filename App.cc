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

#define SAGE_STANDALONE_DEFAULT_MATSIZE 0
#define SAGE_STANDALONE_DEFAULT_REPCNT 0

char *nextArg(char *str) {
  char *aptr = str;
  
  while (aptr && *aptr != 0  && *aptr != ' ') aptr++;

  while (aptr && *aptr != 0  && *aptr == ' ') aptr++;
  
  return aptr;
}

void AppMain() 
{
  ebbrt::kprintf("%s", "Standalone Matrix App: START: ");

  int matsize = SAGE_STANDALONE_DEFAULT_MATSIZE;
  int repcnt  = SAGE_STANDALONE_DEFAULT_REPCNT;

  if (ebbrt::runtime::bootcmdline) {
    char *arg = nextArg(ebbrt::runtime::bootcmdline);
    matsize = atoi(arg);
    arg = nextArg(arg);
    repcnt = atoi(arg);
    ebbrt::kprintf(" %s: ", ebbrt::runtime::bootcmdline);
  };

  ebbrt::kprintf("matsize=%d repcnt=%d\n", matsize, repcnt);

  int count = 0;
  while (count < repcnt) {
    auto id = ebbrt::ebb_allocator->AllocateLocal();
    Matrix::LocalTileCreate(id, matsize, matsize, matsize, matsize,
                            ebbrt::Messenger::NetworkId("0000"));
    ebbrt::EbbRef<Matrix> m = ebbrt::EbbRef<Matrix>(id);
    m->LocalTileRandomize();
    count++;
  }
  ebbrt::kprintf("%s", "Standalone Matrix App: END.\n");
  terminate();
}
