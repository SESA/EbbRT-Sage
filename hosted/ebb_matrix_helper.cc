#include "ebb_matrix_helper.h"

ebbrt::Runtime ebb_runtime;

ebbrt::Context ebb_context(ebb_runtime);

void activate_context() {
  ebb_context.Activate();
}

void deactivate_context() {
  ebb_context.Deactivate();
}
