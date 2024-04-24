#pragma once

/**
 * @class EventLoopGuard
 * @brief A class that provides a guard for controlling the event loop.
 *
 * The EventLoopGuard class is used to control the event loop by setting a flag
 * that indicates whether the event loop is active or not. It provides a way to
 * automatically set the flag to false when an instance of the class is created,
 * and set the flag to true when the instance is destroyed.
 * @note Doesn't take ownership of the pointer event_loop_activeFlag
 */
class EventLoopGuard {
   private:
    bool* event_loop_active;
    bool event_loop_active_init;

   public:
    /**
     * @brief Constructs an EventLoopGuard object.
     *
     * This constructor initializes an EventLoopGuard object with the given
     * parameters.
     *
     * @param event_loop_activeFlag A pointer to the active flag of the event
     * loop.
     * @param new_state The new state to set for the event loop. Will revert
     * back to the opposite state on destruction.
     */
    EventLoopGuard(bool* event_loop_activeFlag, bool new_state)
        : event_loop_active(event_loop_activeFlag) {
        event_loop_active = event_loop_activeFlag;
        *event_loop_active = new_state;

        event_loop_active_init = new_state;
    }

    /**
     * @brief Destructor for the EventLoopGuard class.
     *
     * This destructor is responsible for updating the value of the event loop
     * active flag. It sets the value of the event loop active flag to the
     * negation of the initial event loop active flag.
     *
     * @note This destructor assumes that the event loop active flag is a
     * pointer to a boolean variable.
     *
     * @see EventLoopGuard
     */
    ~EventLoopGuard() { *event_loop_active = !event_loop_active_init; }
};