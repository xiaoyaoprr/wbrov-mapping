#pragma once

#include <openssl/evp.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace wbrov_safe_reloc_ipc
{

namespace fs = std::filesystem;

constexpr std::array<std::uint8_t, 8> kMagic{{'W', 'B', 'R', 'S', 'R', 'I', 'P', 'C'}};
constexpr std::uint16_t kVersion = 1U;
constexpr std::size_t kHeaderBytes = 84U;
constexpr std::size_t kFragmentBytes = 49152U;
constexpr std::size_t kMaxPayloadBytes = 16U * 1024U * 1024U;
constexpr std::uint32_t kMaxFragments = 1024U;

enum class MessageType : std::uint16_t
{
  Hello = 1U,
  Metadata = 10U,
  Info = 11U,
  Odom = 12U,
  CoreState = 13U,
  CoreStatus = 14U,
  Proposal = 20U,
  Ack = 21U,
  Detach = 22U,
  Error = 23U,
  Quiesce = 24U,
  Quiesced = 25U,
};

struct LogicalMessage
{
  MessageType type{};
  std::uint64_t sequence{0U};
  std::uint64_t ack_sequence{0U};
  std::uint64_t stamp_ns{0U};
  std::array<std::uint8_t, 32> digest{};
  std::string payload;
};

inline std::string systemError(const std::string & prefix)
{
  return prefix + ": " + std::string(std::strerror(errno));
}

inline void put16(std::uint8_t * output, std::uint16_t value)
{
  output[0] = static_cast<std::uint8_t>(value >> 8U);
  output[1] = static_cast<std::uint8_t>(value);
}

inline void put32(std::uint8_t * output, std::uint32_t value)
{
  for (int index = 3; index >= 0; --index) {
    output[3 - index] = static_cast<std::uint8_t>(value >> (index * 8));
  }
}

inline void put64(std::uint8_t * output, std::uint64_t value)
{
  for (int index = 7; index >= 0; --index) {
    output[7 - index] = static_cast<std::uint8_t>(value >> (index * 8));
  }
}

inline std::uint16_t get16(const std::uint8_t * input)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(input[0]) << 8U) |
    static_cast<std::uint16_t>(input[1]));
}

inline std::uint32_t get32(const std::uint8_t * input)
{
  std::uint32_t result = 0U;
  for (int index = 0; index < 4; ++index) {
    result = (result << 8U) | input[index];
  }
  return result;
}

inline std::uint64_t get64(const std::uint8_t * input)
{
  std::uint64_t result = 0U;
  for (int index = 0; index < 8; ++index) {
    result = (result << 8U) | input[index];
  }
  return result;
}

inline std::array<std::uint8_t, 32> sha256(const std::string & value)
{
  std::array<std::uint8_t, 32> result{};
  unsigned int result_size = 0U;
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr) {
    throw std::runtime_error("EVP_MD_CTX_new failed");
  }
  const bool ok = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1 &&
    EVP_DigestUpdate(context, value.data(), value.size()) == 1 &&
    EVP_DigestFinal_ex(context, result.data(), &result_size) == 1;
  EVP_MD_CTX_free(context);
  if (!ok || result_size != result.size()) {
    throw std::runtime_error("SHA256 computation failed");
  }
  return result;
}

inline std::string hex(const std::uint8_t * data, std::size_t size)
{
  static constexpr char kHex[] = "0123456789abcdef";
  std::string result;
  result.reserve(size * 2U);
  for (std::size_t index = 0U; index < size; ++index) {
    result.push_back(kHex[(data[index] >> 4U) & 0x0fU]);
    result.push_back(kHex[data[index] & 0x0fU]);
  }
  return result;
}

inline std::string hex(const std::array<std::uint8_t, 32> & value)
{
  return hex(value.data(), value.size());
}

inline void sendDatagram(int descriptor, const std::vector<std::uint8_t> & datagram)
{
  for (;;) {
    const ssize_t sent = ::send(
      descriptor, datagram.data(), datagram.size(), MSG_NOSIGNAL);
    if (sent == static_cast<ssize_t>(datagram.size())) {
      return;
    }
    if (sent < 0 && errno == EINTR) {
      continue;
    }
    if (sent >= 0) {
      throw std::runtime_error("SOCK_SEQPACKET datagram was partially sent");
    }
    throw std::runtime_error(systemError("SOCK_SEQPACKET send failed"));
  }
}

inline void sendLogical(
  int descriptor, MessageType type, std::uint64_t sequence,
  std::uint64_t ack_sequence, std::uint64_t stamp_ns,
  const std::string & payload)
{
  if (payload.size() > kMaxPayloadBytes) {
    throw std::runtime_error("IPC payload exceeds strict limit");
  }
  if (type != MessageType::Ack && sequence == 0U) {
    throw std::runtime_error("non-ACK IPC message needs a positive sequence");
  }
  const auto digest = sha256(payload);
  const std::uint32_t count = static_cast<std::uint32_t>(
    std::max<std::size_t>(1U, (payload.size() + kFragmentBytes - 1U) / kFragmentBytes));
  if (count > kMaxFragments) {
    throw std::runtime_error("IPC fragment count exceeds strict limit");
  }
  for (std::uint32_t index = 0U; index < count; ++index) {
    const std::size_t offset = static_cast<std::size_t>(index) * kFragmentBytes;
    const std::size_t fragment_size = std::min(kFragmentBytes, payload.size() - offset);
    std::vector<std::uint8_t> datagram(kHeaderBytes + fragment_size, 0U);
    std::copy(kMagic.begin(), kMagic.end(), datagram.begin());
    put16(datagram.data() + 8U, kVersion);
    put16(datagram.data() + 10U, static_cast<std::uint16_t>(type));
    put64(datagram.data() + 12U, sequence);
    put64(datagram.data() + 20U, ack_sequence);
    put64(datagram.data() + 28U, stamp_ns);
    put32(datagram.data() + 36U, index);
    put32(datagram.data() + 40U, count);
    put32(datagram.data() + 44U, static_cast<std::uint32_t>(payload.size()));
    put32(datagram.data() + 48U, static_cast<std::uint32_t>(fragment_size));
    std::copy(digest.begin(), digest.end(), datagram.begin() + 52U);
    if (fragment_size > 0U) {
      std::copy(
        payload.begin() + static_cast<std::ptrdiff_t>(offset),
        payload.begin() + static_cast<std::ptrdiff_t>(offset + fragment_size),
        datagram.begin() + static_cast<std::ptrdiff_t>(kHeaderBytes));
    }
    sendDatagram(descriptor, datagram);
  }
}

inline void sendAck(int descriptor, std::uint64_t acknowledged_sequence)
{
  sendLogical(descriptor, MessageType::Ack, 0U, acknowledged_sequence, 0U, "");
}

struct PartialMessage
{
  MessageType type{};
  std::uint64_t ack_sequence{0U};
  std::uint64_t stamp_ns{0U};
  std::uint32_t count{0U};
  std::uint32_t total_size{0U};
  std::array<std::uint8_t, 32> digest{};
  std::map<std::uint32_t, std::string> fragments;
};

class Receiver
{
public:
  std::optional<LogicalMessage> receiveOne(int descriptor, bool nonblocking)
  {
    std::vector<std::uint8_t> buffer(kHeaderBytes + kFragmentBytes);
    struct iovec io_vector {};
    io_vector.iov_base = buffer.data();
    io_vector.iov_len = buffer.size();
    struct msghdr header {};
    header.msg_iov = &io_vector;
    header.msg_iovlen = 1U;
    int flags = MSG_TRUNC;
    if (nonblocking) {
      flags |= MSG_DONTWAIT;
    }
    ssize_t received = -1;
    do {
      received = ::recvmsg(descriptor, &header, flags);
    } while (received < 0 && errno == EINTR);
    if (received < 0 && nonblocking && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return std::nullopt;
    }
    if (received == 0) {
      throw std::runtime_error("IPC peer disconnected");
    }
    if (received < 0) {
      throw std::runtime_error(systemError("SOCK_SEQPACKET receive failed"));
    }
    if ((header.msg_flags & MSG_TRUNC) != 0 ||
      static_cast<std::size_t>(received) > buffer.size())
    {
      throw std::runtime_error("oversized IPC datagram rejected");
    }
    buffer.resize(static_cast<std::size_t>(received));
    if (buffer.size() < kHeaderBytes ||
      !std::equal(kMagic.begin(), kMagic.end(), buffer.begin()))
    {
      throw std::runtime_error("invalid IPC magic or short header");
    }
    if (get16(buffer.data() + 8U) != kVersion) {
      throw std::runtime_error("unsupported IPC protocol version");
    }
    const auto type = static_cast<MessageType>(get16(buffer.data() + 10U));
    if (!validType(type)) {
      throw std::runtime_error("unknown IPC message type");
    }
    const std::uint64_t sequence = get64(buffer.data() + 12U);
    const std::uint64_t ack_sequence = get64(buffer.data() + 20U);
    const std::uint64_t stamp_ns = get64(buffer.data() + 28U);
    const std::uint32_t index = get32(buffer.data() + 36U);
    const std::uint32_t count = get32(buffer.data() + 40U);
    const std::uint32_t total_size = get32(buffer.data() + 44U);
    const std::uint32_t fragment_size = get32(buffer.data() + 48U);
    std::array<std::uint8_t, 32> digest{};
    std::copy(buffer.begin() + 52, buffer.begin() + 84, digest.begin());
    if (count == 0U || count > kMaxFragments || index >= count ||
      total_size > kMaxPayloadBytes || fragment_size > kFragmentBytes ||
      buffer.size() != kHeaderBytes + fragment_size ||
      (type != MessageType::Ack && sequence == 0U) ||
      (type == MessageType::Ack && (sequence != 0U || ack_sequence == 0U ||
      total_size != 0U || count != 1U || index != 0U)))
    {
      throw std::runtime_error("invalid IPC fragment metadata");
    }
    if (type == MessageType::Ack) {
      if (digest != sha256("")) {
        throw std::runtime_error("invalid ACK digest");
      }
      return LogicalMessage{type, sequence, ack_sequence, stamp_ns, digest, ""};
    }

    auto & partial = partial_[sequence];
    if (partial.fragments.empty()) {
      partial = PartialMessage{type, ack_sequence, stamp_ns, count, total_size, digest, {}};
    } else if (partial.type != type || partial.ack_sequence != ack_sequence ||
      partial.stamp_ns != stamp_ns || partial.count != count ||
      partial.total_size != total_size || partial.digest != digest)
    {
      throw std::runtime_error("one IPC sequence changed fragment identity");
    }
    const std::string fragment(
      reinterpret_cast<const char *>(buffer.data() + kHeaderBytes), fragment_size);
    const auto existing = partial.fragments.find(index);
    if (existing != partial.fragments.end() && existing->second != fragment) {
      throw std::runtime_error("one IPC fragment index changed payload");
    }
    partial.fragments[index] = fragment;
    if (partial.fragments.size() != count) {
      return std::nullopt;
    }
    std::string payload;
    payload.reserve(total_size);
    for (std::uint32_t part = 0U; part < count; ++part) {
      const auto found = partial.fragments.find(part);
      if (found == partial.fragments.end()) {
        throw std::runtime_error("IPC fragment sequence is incomplete");
      }
      payload += found->second;
    }
    if (payload.size() != total_size || sha256(payload) != digest) {
      throw std::runtime_error("IPC logical payload digest/size mismatch");
    }
    partial_.erase(sequence);
    return LogicalMessage{type, sequence, ack_sequence, stamp_ns, digest, std::move(payload)};
  }

private:
  static bool validType(MessageType value)
  {
    switch (value) {
      case MessageType::Hello:
      case MessageType::Metadata:
      case MessageType::Info:
      case MessageType::Odom:
      case MessageType::CoreState:
      case MessageType::CoreStatus:
      case MessageType::Proposal:
      case MessageType::Ack:
      case MessageType::Detach:
      case MessageType::Error:
      case MessageType::Quiesce:
      case MessageType::Quiesced:
        return true;
    }
    return false;
  }

  std::map<std::uint64_t, PartialMessage> partial_;
};

inline bool waitReadable(int descriptor, int timeout_ms)
{
  struct pollfd item {};
  item.fd = descriptor;
  item.events = POLLIN;
  int result = -1;
  do {
    result = ::poll(&item, 1U, timeout_ms);
  } while (result < 0 && errno == EINTR);
  if (result < 0) {
    throw std::runtime_error(systemError("IPC poll failed"));
  }
  if (result == 0) {
    return false;
  }
  if ((item.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 &&
    (item.revents & POLLIN) == 0)
  {
    throw std::runtime_error("IPC peer hangup/error");
  }
  return (item.revents & POLLIN) != 0;
}

inline std::uint64_t processStartTicks(pid_t pid)
{
  std::ifstream input("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("cannot read peer /proc stat");
  }
  const std::size_t close = line.rfind(')');
  if (close == std::string::npos || close + 2U >= line.size()) {
    throw std::runtime_error("malformed peer /proc stat");
  }
  std::istringstream fields(line.substr(close + 2U));
  std::string token;
  for (int field = 3; field <= 22; ++field) {
    if (!(fields >> token)) {
      throw std::runtime_error("short peer /proc stat");
    }
    if (field == 22) {
      std::size_t used = 0U;
      const unsigned long long parsed = std::stoull(token, &used, 10);
      if (used != token.size() || parsed == 0ULL) {
        throw std::runtime_error("invalid peer process start ticks");
      }
      return static_cast<std::uint64_t>(parsed);
    }
  }
  throw std::runtime_error("peer process start ticks missing");
}

struct FileIdentity
{
  std::uint64_t device{0U};
  std::uint64_t inode{0U};
  std::string canonical_path;
};

inline FileIdentity fileIdentity(const fs::path & input, bool require_regular = true)
{
  const fs::path canonical = fs::canonical(input);
  struct stat info {};
  if (::stat(canonical.c_str(), &info) != 0) {
    throw std::runtime_error(systemError("cannot stat identity path"));
  }
  if (require_regular && (!S_ISREG(info.st_mode) || info.st_nlink != 1)) {
    throw std::runtime_error("identity path is not a single-link regular file");
  }
  return FileIdentity{
    static_cast<std::uint64_t>(info.st_dev),
    static_cast<std::uint64_t>(info.st_ino), canonical.string()};
}

inline FileIdentity processExeIdentity(pid_t pid)
{
  return fileIdentity(fs::path("/proc") / std::to_string(pid) / "exe");
}

inline std::string jsonEscape(const std::string & value)
{
  std::ostringstream output;
  output << '"';
  static constexpr char kHex[] = "0123456789abcdef";
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u00" << kHex[(character >> 4U) & 0x0fU] << kHex[character & 0x0fU];
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

inline void writeAll(int descriptor, const std::string & value)
{
  std::size_t offset = 0U;
  while (offset < value.size()) {
    const ssize_t written = ::write(descriptor, value.data() + offset, value.size() - offset);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      throw std::runtime_error(systemError("file write failed"));
    }
    offset += static_cast<std::size_t>(written);
  }
}

class AtomicStateWriter
{
public:
  explicit AtomicStateWriter(const fs::path & path)
  {
    if (!path.is_absolute() || path.filename().empty()) {
      throw std::invalid_argument("state path must be an absolute file path");
    }
    directory_fd_ = ::open(
      path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (directory_fd_ < 0) {
      throw std::runtime_error(systemError("cannot open state directory"));
    }
    struct stat info {};
    if (::fstat(directory_fd_, &info) != 0 || !S_ISDIR(info.st_mode) ||
      info.st_uid != ::geteuid())
    {
      ::close(directory_fd_);
      directory_fd_ = -1;
      throw std::runtime_error("state parent is not a current-user directory");
    }
    name_ = path.filename().string();
    verifyExisting();
  }

  ~AtomicStateWriter()
  {
    if (directory_fd_ >= 0) {
      ::close(directory_fd_);
    }
  }

  AtomicStateWriter(const AtomicStateWriter &) = delete;
  AtomicStateWriter & operator=(const AtomicStateWriter &) = delete;

  void write(const std::string & json)
  {
    verifyExisting();
    const std::string temporary = "." + name_ + ".tmp." +
      std::to_string(::getpid()) + "." + std::to_string(++sequence_);
    int descriptor = ::openat(
      directory_fd_, temporary.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (descriptor < 0) {
      throw std::runtime_error(systemError("cannot create state temporary"));
    }
    bool renamed = false;
    try {
      const std::string encoded = json + "\n";
      if (::fchmod(descriptor, 0600) != 0) {
        throw std::runtime_error(systemError("state chmod failed"));
      }
      writeAll(descriptor, encoded);
      if (::fsync(descriptor) != 0) {
        throw std::runtime_error(systemError("state fsync failed"));
      }
      struct stat info {};
      if (::fstat(descriptor, &info) != 0 || !S_ISREG(info.st_mode) ||
        info.st_uid != ::geteuid() || info.st_nlink != 1 ||
        (info.st_mode & 07777) != 0600)
      {
        throw std::runtime_error("state temporary identity/mode mismatch");
      }
      if (::close(descriptor) != 0) {
        descriptor = -1;
        throw std::runtime_error(systemError("state close failed"));
      }
      descriptor = -1;
      if (::renameat(directory_fd_, temporary.c_str(), directory_fd_, name_.c_str()) != 0) {
        throw std::runtime_error(systemError("state atomic rename failed"));
      }
      renamed = true;
      if (::fsync(directory_fd_) != 0) {
        throw std::runtime_error(systemError("state directory fsync failed"));
      }
      verifyExisting();
    } catch (...) {
      if (descriptor >= 0) {
        ::close(descriptor);
      }
      if (!renamed) {
        ::unlinkat(directory_fd_, temporary.c_str(), 0);
      }
      throw;
    }
  }

private:
  void verifyExisting() const
  {
    struct stat info {};
    if (::fstatat(directory_fd_, name_.c_str(), &info, AT_SYMLINK_NOFOLLOW) == 0) {
      if (!S_ISREG(info.st_mode) || info.st_uid != ::geteuid() ||
        info.st_nlink != 1 || (info.st_mode & 07777) != 0600)
      {
        throw std::runtime_error("unsafe existing state target");
      }
    } else if (errno != ENOENT) {
      throw std::runtime_error(systemError("state target stat failed"));
    }
  }

  int directory_fd_{-1};
  std::string name_;
  std::uint64_t sequence_{0U};
};

}  // namespace wbrov_safe_reloc_ipc
