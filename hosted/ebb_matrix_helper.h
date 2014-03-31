#include <memory>

#include <ebbrt/Context.h>
#include <ebbrt/Future.h>
#include <ebbrt/Runtime.h>

extern ebbrt::Runtime ebb_runtime;
extern ebbrt::Context ebb_context;

void activate_context();

void deactivate_context();

template <typename T> T wait_for_future(ebbrt::Future<T>* fut) {
  ebb_context.Activate();
  ebbrt::Future<T> filled_future;
  fut->Then(ebbrt::Launch::Async, [&filled_future](ebbrt::Future<T> f) {
    filled_future = std::move(f);
    ebb_context.Stop();
  });
  ebb_context.Deactivate();
  ebb_context.Run();
  ebb_context.Reset();
  return std::move(filled_future.Get());
}
