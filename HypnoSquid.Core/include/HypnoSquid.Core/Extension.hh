#pragma once

namespace hs {
namespace core {

/**
 * BitMask for extension invocation schedule.
 *
 * * AFTER_ASYNC_SYSTEM - Invoke the extension after every asynchronous system, this will block the enqueueing of
 * new threads.
 * * AFTER_SYNC_SYSTEM - Invoke after every synchronized system.
 * * AFTER_ALL_SYSTEMS - Invoke after all systems have run.
 */
enum ExtensionInvocationSchedule { AFTER_ASYNC_SYSTEM = 0b001, AFTER_SYNC_SYSTEM = 0b010, AFTER_ALL_SYSTEMS = 0b100 };

/**
 * Interface for extensions, not necessarily required as extensions are compile time evaluated.
 */
class Extension {
public:
  virtual void invoke(InvocationID id, ExtensionInvocationSchedule schedule_state) = 0;
  virtual bool can_system_start(SystemState &system_state) = 0;
  virtual void on_system_start(SystemState &system_state) = 0;
  virtual void on_system_end(SystemState &system_state) = 0;
};

} // namespace core
} // namespace hs