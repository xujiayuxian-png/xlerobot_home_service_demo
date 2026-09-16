// AidLite scans /usr/local/lib even when LD_LIBRARY_PATH selects a private SDK.
// Redirect only its QNN240 plugin load in the VLM process. Other workers retain
// the system SDK and plugin; no model data or inference functions are changed.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdlib.h>
#include <string.h>

void *dlopen(const char *filename, int flags) {
  void *(*original)(const char *, int) = dlsym(RTLD_NEXT, "dlopen");
  if (!original) return NULL;
  const char *override = getenv("XLEROBOT_AIDLITE_QNN240_LIBRARY");
  if (filename && override && *override &&
      strcmp(filename, "/usr/local/lib/libaidlite_qnn240.so") == 0) {
    filename = override;
  }
  return original(filename, flags);
}
