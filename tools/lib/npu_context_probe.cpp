// Inspect the installed AidGen configuration API without creating a generator.
#include <aidlux/aidgen/aidgen.hpp>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2 && argc != 3) { return 2; }
  using namespace aplux::aidgen;
  ContextProperties properties;
  auto context = Context::create_instance(argv[1], properties, BackendType::QNN240);
  if (!context || context->initialize() != ErrorCode::SUCCESS) { return 1; }
  std::cout << context->get_config() << std::endl;
  if (argc == 3) {
    auto result = context->set_config("generator_n-threads", argv[2]);
    std::cout << "set_result=" << static_cast<int>(result) << '\n'
              << context->get_config() << std::endl;
  }
  return 0;
}
