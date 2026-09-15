// EXPERIMENT ONLY: AidGenSE 1.4 / AidGen 2.3 raw Qwen3-VL bundle compatibility.
// The vendor server asks an exported SE bridge for the vocabulary embedding;
// its raw Genie loader returns an empty buffer. Supply the unmodified local
// FP32 table through that version-specific bridge, with an exact size guard.
#include <cstddef>
#include <cstdlib>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aplux::aidgen {
class ContextImpl;
class RawEmbedding {
 public:
  const char *data = nullptr;
  std::size_t size = 0;
  explicit RawEmbedding(const char *path) {
    constexpr std::size_t expected = 151936ULL * 2560 * sizeof(float);
    const int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) { return; }
    struct stat info{};
    if (fstat(fd, &info) == 0 && info.st_size == expected) {
      void *mapped = mmap(nullptr, expected, PROT_READ, MAP_PRIVATE, fd, 0);
      if (mapped != MAP_FAILED) { data = static_cast<const char *>(mapped); size = expected; }
    }
    close(fd);
  }
  ~RawEmbedding() { if (data) { munmap(const_cast<char *>(data), size); } }
};

int get_embedding_buff_se(ContextImpl *context, const char *&buffer, std::size_t &size) {
  const char *path = std::getenv("XLEROBOT_AIDGEN_VOCAB_EMBEDDING");
  if (path && *path) {
    static RawEmbedding table(path);
    buffer = table.data;
    size = table.size;
    return table.data ? 0 : -20;
  }
  using Original = int (*)(ContextImpl *, const char *&, std::size_t &);
  static auto original = reinterpret_cast<Original>(dlsym(RTLD_NEXT,
      "_ZN5aplux6aidgen21get_embedding_buff_seEPNS0_11ContextImplERPKcRm"));
  return original ? original(context, buffer, size) : -20;
}
}  // namespace aplux::aidgen
