//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include <iostream>
#include <memory>
#include <sys/sysinfo.h>

#include <ebbrt/Context.h>
#include <ebbrt/ContextActivation.h>
#include <ebbrt/Runtime.h>
#include <ebbrt/Debug.h>
#include <ebbrt/EbbAllocator.h>

#include "../Matrix.h"

extern void AppMain() __attribute__((weak));

int
getBootCmdline(char **cmdline)
{
  int fd=-1;

  *cmdline = NULL;

  if((fd=open("/proc/cmdline", O_RDONLY)) < -1)
    return 0;
 
  ssize_t n=0;
  ssize_t rc=0;
  *cmdline =  (char *)malloc(4096);
  while ((rc=read(fd, &((*cmdline)[n]), 4096-n)>=0)) {
    n += rc;
    if (n==4096) break;
  }
  (*cmdline)[n]=0;

  return 0;
}

int main() {

  //printf("EbbRT-Sage:hosted/standalone-backend: START\n");
  ebbrt::Runtime runtime;
  ebbrt::Context c(runtime);
  boost::asio::signal_set sig(c.io_service_, SIGINT);
  { // scope to trigger automatic deactivate on exit
    ebbrt::ContextActivation activation(c);

    // ensure clean quit on ctrl-c
    sig.async_wait([&c](const boost::system::error_code& ec,
                        int signal_number) { c.io_service_.stop(); });
    if (AppMain) {
      getBootCmdline(&ebbrt::runtime::bootcmdline);
      ebbrt::event_manager->Spawn([=]() { AppMain(); });
    }
  }
  c.Run();
  //printf("EbbRT-Sage:hosted/standalone-backend: END\n");
}

