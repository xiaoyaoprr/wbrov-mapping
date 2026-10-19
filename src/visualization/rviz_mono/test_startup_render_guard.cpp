#include "startup_render_guard.hpp"
#include <cassert>
int main()
{
  using namespace wbrov;
  assert(startupAttempt(nullptr) == 0);
  assert(startupAttempt("1") == 1);
  assert(startupAttempt("2") == maxRenderRetries);
  assert(startupAttempt("-1") == maxRenderRetries);
  assert(startupAttempt("999") == maxRenderRetries);
  assert(isContextCreationFailure("Failed to create an OpenGL context. GLXBadDrawable"));
  assert(isContextCreationFailure("Unable to create the rendering window after 100 tries"));
  assert(isContextCreationFailure("rviz::RenderSystem: error creating render window: error"));
  assert(!isContextCreationFailure("Frame [map] does not exist"));
  assert(!isContextCreationFailure("Stereo is NOT SUPPORTED"));
}
