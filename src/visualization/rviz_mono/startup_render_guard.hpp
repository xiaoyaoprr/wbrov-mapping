#ifndef WBROV_STARTUP_RENDER_GUARD_HPP
#define WBROV_STARTUP_RENDER_GUARD_HPP
#include <string>

namespace wbrov {
inline bool isContextCreationFailure(const std::string & message)
{
  return message.find("Failed to create an OpenGL context") != std::string::npos ||
         message.find("Unable to create the rendering window") != std::string::npos ||
         message.find("error creating render window") != std::string::npos;
}
inline unsigned startupAttempt(const char * value)
{
  // Unknown values cannot expand the retry budget.
  if (!value) return 0;
  if (std::string(value) == "1") return 1;
  return 2;
}
constexpr unsigned maxRenderRetries = 2;
}
#endif
