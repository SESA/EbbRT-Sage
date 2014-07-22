//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include <iostream>
#include <memory>

#include <ebbrt/Context.h>
#include <ebbrt/ContextActivation.h>
#include <ebbrt/Runtime.h>
#include <ebbrt/Debug.h>
#include <ebbrt/EbbAllocator.h>

#include "../Matrix.h"

void 
AppMain() 
{
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

  //  ebbrt::kprintf("Standalone Matrix App: END");
}

int main() {
  printf("EbbRT-Sage:hosted/standalone-backend: START\n");
  ebbrt::Runtime runtime;
  ebbrt::Context c(runtime);
  boost::asio::signal_set sig(c.io_service_, SIGINT);
  { // scope to trigger automatic deactivate on exit
    ebbrt::ContextActivation activation(c);

    // ensure clean quit on ctrl-c
    sig.async_wait([&c](const boost::system::error_code& ec,
                        int signal_number) { c.io_service_.stop(); });
    AppMain();
  }
  //  c.Run();
  printf("EbbRT-Sage:hosted/standalone-backend: END\n");
}

