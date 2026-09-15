// Offline AidGen experiment. Reads prompts from a file; has no ROS or device control.
#include <aidlux/aidgen/aidgen.hpp>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <sys/resource.h>
#include <thread>

using namespace aplux::aidgen;
using Clock = std::chrono::steady_clock;

static double cpu_seconds() {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
    (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) * 1e-6;
}

int main(int argc, char ** argv) {
  if (argc < 3 || argc > 4 || (argc == 4 && std::string(argv[3]) != "--embedding")) {
    std::cerr << "usage: npu_text_probe model-config.json prompts.txt [--embedding]\n";
    return 2;
  }
  std::ifstream input(argv[2]);
  if (!input) {return 2;}
  ContextProperties properties;
  properties.enable_profiler = true;
  properties.enable_prompt_cache = false;
  auto context = Context::create_instance(argv[1], properties, BackendType::QNN236);
  if (!context || context->initialize() != ErrorCode::SUCCESS) {return 1;}
  auto tokenizer = Tokenizer::create_instance(context);
  if (!tokenizer || tokenizer->initialize() != ErrorCode::SUCCESS) {return 1;}
  std::unique_ptr<EmbeddingExtractor> extractor;
  if (argc == 4) {
    extractor = EmbeddingExtractor::create_instance(context);
    if (!extractor || extractor->initialize() != ErrorCode::SUCCESS) {return 1;}
  }
  auto generator = Generator::create_instance(context);
  if (!generator || generator->initialize() != ErrorCode::SUCCESS) {return 1;}
  if (generator->set_property("temp", "0.1") != ErrorCode::SUCCESS) {return 1;}
  if (generator->set_property("top-k", "1") != ErrorCode::SUCCESS) {return 1;}
  std::cout << "PROPERTIES " << generator->get_property() << std::endl;
  std::string command;
  int index = 0;
  while (std::getline(input, command)) {
    if (command.empty()) {continue;}
    if (generator->reset() != ErrorCode::SUCCESS) {return 1;}
    const std::string prompt =
      "<|im_start|>system\n你是家庭服务机器人的意图解析器。只输出JSON。"
      "支持fetch_deliver取物递送。物品名保持用户原意，不限定物品类别。"
      "取消输出cancel，闲聊输出unknown，没有明确物品输出clarify。"
      "格式：{\"intent\":\"fetch_deliver/cancel/unknown/clarify\",\"object\":\"物品名或空字符串\"}。"
      "不要解释。<|im_end|>\n<|im_start|>user\n" + command +
      "<|im_end|>\n<|im_start|>assistant\n<think>\n\n</think>\n";
    std::mutex mutex;
    std::condition_variable cv;
    bool finished = false;
    std::thread deadline([&]() {
      std::unique_lock<std::mutex> lock(mutex);
      if (!cv.wait_for(lock, std::chrono::seconds(20), [&]() {return finished;})) {
        generator->abort();
      }
    });
    std::string text;
    const auto started = Clock::now();
    const double cpu_start = cpu_seconds();
    auto callback = [&](const GenEvent & event, void *) {
      text += event.text;
    };
    ErrorCode result;
    if (extractor) {
      Tensor embedding;
      result = extractor->encode(prompt, embedding);
      if (result == ErrorCode::SUCCESS) {
        result = generator->run(embedding, callback);
      }
    } else {
      result = generator->run(prompt, callback);
    }
    const double cpu = cpu_seconds() - cpu_start;
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    {
      std::lock_guard<std::mutex> lock(mutex);
      finished = true;
    }
    cv.notify_one();
    deadline.join();
    std::cout << "@@RESULT " << index++ << " " << static_cast<int>(result)
              << " " << elapsed << " " << cpu << "\n" << text << "\n@@END\n" << std::flush;
  }
  return generator->finalize() == ErrorCode::SUCCESS ? 0 : 1;
}
