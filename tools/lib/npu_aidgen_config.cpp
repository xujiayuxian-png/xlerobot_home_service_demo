// Process-local compatibility shim for AidGen: use its public configuration
// API before the vendor HTTP server creates the generator. No binary patches.
// Recompile against the installed SDK after an SDK upgrade; do not reuse an
// older binary across ContextProperties ABI changes.
#include <aidlux/aidgen/aidgen.hpp>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>

namespace aplux::aidgen {
std::shared_ptr<Context> Context::create_instance(
    const std::string &config, const ContextProperties &properties, BackendType backend) {
  using Create = std::shared_ptr<Context> (*)(const std::string &, const ContextProperties &, BackendType);
  static auto original = reinterpret_cast<Create>(dlsym(RTLD_NEXT,
    "_ZN5aplux6aidgen7Context15create_instanceERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS0_17ContextPropertiesENS0_11BackendTypeE"));
  if (!original) { return nullptr; }
  // Raw, unencrypted QNN bundles need the separate Genie dialog configuration.
  // The HTTP server still reads its original VLM/vision metadata configuration.
  const char *raw_config = std::getenv("XLEROBOT_AIDGEN_RAW_CONFIG");
  return original(raw_config && *raw_config ? std::string(raw_config) : config, properties, backend);
}

ErrorCode Context::initialize(void *reserve) {
  using Initialize = ErrorCode (*)(Context *, void *);
  static auto original = reinterpret_cast<Initialize>(
    dlsym(RTLD_NEXT, "_ZN5aplux6aidgen7Context10initializeEPv"));
  if (!original) { return ErrorCode::BACKEND_NOT_FOUND; }
  auto result = original(this, reserve);
  if (result != ErrorCode::SUCCESS) { return result; }
  result = set_config("generator_n-threads", "0");
  if (result == ErrorCode::SUCCESS) {
    std::fprintf(stderr, "X1 AidGen: generator_n-threads=0 via public Context API\n");
  }
  return result;
}
}  // namespace aplux::aidgen
