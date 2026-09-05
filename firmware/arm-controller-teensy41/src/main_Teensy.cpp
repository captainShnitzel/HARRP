//HARRP V2.0 Teensy 4.1 FreeRTOS control software v1.05
#include <Arduino.h>
#include <math.h>

#include "arduino_freertos.h"
#include "queue.h"
#include "Dynamixel2Arduino.h"




// ============================================================
// Configuration
// ============================================================

namespace Config
{
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr TickType_t SAFETY_PERIOD =
    pdMS_TO_TICKS(5);       // 200 Hz

constexpr TickType_t CONTROL_PERIOD =
    pdMS_TO_TICKS(10);      // 100 Hz

constexpr TickType_t MOTOR_PERIOD =
    pdMS_TO_TICKS(10);      // 100 Hz

constexpr TickType_t DIAGNOSTIC_PERIOD =
    pdMS_TO_TICKS(1000);    // 1 Hz

// FreeRTOS stack depths are expressed in StackType_t units,
// not bytes. On Teensy 4.1 this will normally mean 32-bit words.
constexpr uint16_t SAFETY_STACK_DEPTH = 512;
constexpr uint16_t CONTROL_STACK_DEPTH = 1024;
constexpr uint16_t MOTOR_STACK_DEPTH = 768;
constexpr uint16_t DIAGNOSTIC_STACK_DEPTH = 512;

constexpr uint8_t JOINT_COUNT = 7;
constexpr uint8_t ALL_JOINTS_ONLINE_MASK = 0x7F;

// A remote sender cannot keep a target valid indefinitely.
constexpr uint32_t MAX_TARGET_VALIDITY_MS = 250;

// A joint command must be regenerated regularly.
constexpr uint32_t JOINT_COMMAND_VALIDITY_MS = 50;

// Maximum age of measured motor feedback.
constexpr uint32_t JOINT_STATE_MAX_AGE_MS = 100;


// Keep false until all of these are implemented and tested:
// - emergency-stop input
// - hardware motor-enable output
// - joint feedback
// - joint limits
// - inverse kinematics
// - Dynamixel communication
constexpr bool ENABLE_MOTOR_TRANSMISSION = false;
}

// ============================================================
// Robot states
// ============================================================

enum class RobotState : uint8_t
{
    Boot,
    SelfTest,
    Idle,
    Ready,
    Active,
    Hold,
    Fault,
    EmergencyStop
};

enum class FaultCode : uint16_t
{
    None,
    StartupFailure,
    CommandTimeout,
    InvalidTarget,
    IkFailure,
    InvalidJointCommand,
    JointLimitViolation,
    JointStateTimeout,
    MotorCommunicationFailure,
    EmergencyStopActive
};

// ============================================================
// Shared control structures
// ============================================================

struct Vector3
{
    float x;
    float y;
    float z;
};

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct TargetPose
{
    // Position in metres, expressed in the robot base frame.
    Vector3 position;

    // Unit quaternion, also expressed in the robot base frame.
    Quaternion orientation;

    float maximumLinearVelocity;   // m/s
    float maximumAngularVelocity;  // rad/s

    uint32_t sequence;

    // Timestamp supplied by the sender. Diagnostic use only.
    uint32_t sourceTimestampMs;

    // Assigned locally by the Teensy after receiving the target.
    uint32_t receivedAtMs;

    uint32_t validityMs;
    bool valid;
};

struct JointState
{
    float positionRad[Config::JOINT_COUNT];
    float velocityRadPerSecond[Config::JOINT_COUNT];

    // One bit per joint. Bits 0-6 correspond to Q1-Q7.
    uint8_t onlineMask;

    uint32_t measuredAtMs;
    bool valid;
};

struct JointCommand
{
    float positionRad[Config::JOINT_COUNT];
    float velocityLimitRadPerSecond[Config::JOINT_COUNT];

    uint32_t sourceSequence;
    uint32_t generatedAtMs;
    uint32_t validityMs;

    bool valid;
};

struct SystemStatus
{
    RobotState state;
    FaultCode fault;

    uint32_t stateChangedAtMs;
    uint32_t faultRaisedAtMs;

    // This means permission to transmit motion commands.
    // It does not mean that the physical motor-power relay is closed.
    bool motionCommandPermitted;
};

// ============================================================
// RTOS objects
// ============================================================

TaskHandle_t safetyTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t diagnosticTaskHandle = nullptr;

// Length-one queues act as latest-value mailboxes.
QueueHandle_t targetMailbox = nullptr;
QueueHandle_t jointStateMailbox = nullptr;
QueueHandle_t jointCommandMailbox = nullptr;

SystemStatus systemStatus{
    RobotState::Boot,
    FaultCode::None,
    0,
    0,
    false
};

// ============================================================
// Forward declarations
// ============================================================

void safetyTask(void* parameter);
void controlTask(void* parameter);
void motorTask(void* parameter);
void diagnosticTask(void* parameter);

void initializeSystem();
bool createRtosObjects();
bool createTasks();

[[noreturn]] void stopOnStartupFailure();

SystemStatus getSystemStatus();
void setMotionCommandPermission(bool permitted);
void enterHold();
void raiseFault(FaultCode fault);

bool validateTargetFields(const TargetPose& target);
bool targetIsFresh(const TargetPose& target, uint32_t nowMs);

bool jointStateIsFresh(
    const JointState& state,
    uint32_t nowMs
);

bool validateJointCommand(
    const JointCommand& command
);

bool jointCommandIsFresh(
    const JointCommand& command,
    uint32_t nowMs
);

bool commandMatchesCurrentData(
    const JointCommand& command,
    uint32_t nowMs
);

bool controlDataReady(uint32_t nowMs);

bool movementPermitted(
    const JointCommand& command,
    uint32_t nowMs
);

bool publishTarget(TargetPose target);
bool publishJointState(JointState state);

bool calculateInverseKinematics(
    const TargetPose& target,
    const JointState& measuredState,
    JointCommand& command
);

bool sendJointCommandToMotors(
    const JointCommand& command
);

void applyNonActiveMotorPolicy(RobotState state);

// ============================================================
// Arduino entry points
// ============================================================

void setup()
{
    Serial.begin(Config::SERIAL_BAUD);

    initializeSystem();

    if (!createRtosObjects())
    {
        stopOnStartupFailure();
    }

    if (!createTasks())
    {
        stopOnStartupFailure();
    }

    Serial.println("Starting FreeRTOS scheduler");
    Serial.flush();

    vTaskStartScheduler();

    // Returning means that the scheduler could not start.
    stopOnStartupFailure();
}

void loop()
{
    // Unused after the scheduler starts.
}

// ============================================================
// Initialization
// ============================================================

void initializeSystem()
{
    /*
     * Hardware initialization to be added later:
     *
     * 1. Configure the normally-closed emergency-stop input.
     * 2. Force the physical motor-enable output into its safe state.
     * 3. Initialize the Dynamixel serial bus.
     * 4. Ping all required motors.
     * 5. Read their initial positions.
     *
     * This is currently a structure-only build.
     * ENABLE_MOTOR_TRANSMISSION remains false, so transitioning
     * to Idle cannot permit motor commands.
     */
    systemStatus.state = RobotState::SelfTest;
    systemStatus.fault = FaultCode::None;
    systemStatus.stateChangedAtMs = millis();
    systemStatus.faultRaisedAtMs = 0;
    systemStatus.motionCommandPermitted = false;
    systemStatus.state = RobotState::Idle;
    systemStatus.stateChangedAtMs = millis();
}

bool createRtosObjects()
{
    targetMailbox =
        xQueueCreate(1, sizeof(TargetPose));

    jointStateMailbox =
        xQueueCreate(1, sizeof(JointState));

    jointCommandMailbox =
        xQueueCreate(1, sizeof(JointCommand));

    if (
        targetMailbox == nullptr ||
        jointStateMailbox == nullptr ||
        jointCommandMailbox == nullptr
    )
    {
        return false;
    }

    // Put known invalid values in all mailboxes.
    const TargetPose invalidTarget{};
    const JointState invalidJointState{};
    const JointCommand invalidJointCommand{};

    xQueueOverwrite(
        targetMailbox,
        &invalidTarget
    );

    xQueueOverwrite(
        jointStateMailbox,
        &invalidJointState
    );

    xQueueOverwrite(
        jointCommandMailbox,
        &invalidJointCommand
    );

    return true;
}

bool createTasks()
{
 /*
 * Control has higher priority than Motor to reduce command
 * generation latency. Correctness does not depend on task
 * wake order; timestamps and sequence checks enforce command
 * validity.
 */
    if (
        xTaskCreate(
            safetyTask,
            "Safety",
            Config::SAFETY_STACK_DEPTH,
            nullptr,
            4,
            &safetyTaskHandle
        ) != pdPASS
    )
    {
        return false;
    }

    if (
        xTaskCreate(
            controlTask,
            "Control",
            Config::CONTROL_STACK_DEPTH,
            nullptr,
            3,
            &controlTaskHandle
        ) != pdPASS
    )
    {
        return false;
    }

    if (
        xTaskCreate(
            motorTask,
            "Motor",
            Config::MOTOR_STACK_DEPTH,
            nullptr,
            2,
            &motorTaskHandle
        ) != pdPASS
    )
    {
        return false;
    }

    if (
        xTaskCreate(
            diagnosticTask,
            "Diag",
            Config::DIAGNOSTIC_STACK_DEPTH,
            nullptr,
            1,
            &diagnosticTaskHandle
        ) != pdPASS
    )
    {
        return false;
    }

    return true;
}

[[noreturn]] void stopOnStartupFailure()
{
    systemStatus.state = RobotState::Fault;
    systemStatus.fault = FaultCode::StartupFailure;
    systemStatus.stateChangedAtMs = millis();
    systemStatus.faultRaisedAtMs = millis();
    systemStatus.motionCommandPermitted = false;

    /*
     * TODO:
     * Force the physical motor-enable output low here.
     */

    Serial.println(
        "FATAL: RTOS startup failed; motors remain disabled"
    );
    Serial.flush();

    while (true)
    {
        delay(1000);
    }
}

// ============================================================
// RTOS tasks
// ============================================================

void safetyTask(void*)
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    while (true)
    {
        const uint32_t nowMs = millis();

        const SystemStatus status =
            getSystemStatus();

        /*
         * TODO:
         * Replace this with the real normally-closed
         * emergency-stop input.
         */
        const bool emergencyStopActive = false;

        if (emergencyStopActive)
        {
            raiseFault(
                FaultCode::EmergencyStopActive
            );
        }
        else if (
            status.state == RobotState::Active
        )
        {
            TargetPose target{};

            const bool targetAvailable =
                xQueuePeek(
                    targetMailbox,
                    &target,
                    0
                ) == pdPASS;

            if (
                !targetAvailable ||
                !targetIsFresh(target, nowMs)
            )
            {
                /*
                 * Loss of the live command stream stops new
                 * movement immediately.
                 */
                enterHold();
            }
            else
            {
                setMotionCommandPermission(
                    controlDataReady(nowMs)
                );
            }
        }
        else
        {
            setMotionCommandPermission(false);
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::SAFETY_PERIOD
        );
    }
}

void controlTask(void*)
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    while (true)
    {
        const uint32_t nowMs = millis();

        const SystemStatus status =
            getSystemStatus();

        if (
            status.state == RobotState::Ready ||
            status.state == RobotState::Active
        )
        {
            TargetPose target{};
            JointState measuredState{};

            const bool targetAvailable =
                xQueuePeek(
                    targetMailbox,
                    &target,
                    0
                ) == pdPASS &&
                targetIsFresh(target, nowMs);

            const bool jointStateAvailable =
                xQueuePeek(
                    jointStateMailbox,
                    &measuredState,
                    0
                ) == pdPASS &&
                jointStateIsFresh(
                    measuredState,
                    nowMs
                );

            if (
                targetAvailable &&
                jointStateAvailable
            )
            {
                JointCommand nextCommand{};

                if (
                    !calculateInverseKinematics(
                        target,
                        measuredState,
                        nextCommand
                    )
                )
                {
                    raiseFault(
                        FaultCode::IkFailure
                    );
                }
                else
                {
                    nextCommand.sourceSequence =
                        target.sequence;

                    nextCommand.generatedAtMs =
                        nowMs;

                    nextCommand.validityMs =
                        Config::
                            JOINT_COMMAND_VALIDITY_MS;

                    nextCommand.valid = true;

                    if (
                        !validateJointCommand(
                            nextCommand
                        )
                    )
                    {
                        raiseFault(
                            FaultCode::
                                InvalidJointCommand
                        );
                    }
                    else
                    {
                        xQueueOverwrite(
                            jointCommandMailbox,
                            &nextCommand
                        );
                    }
                }
            }
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::CONTROL_PERIOD
        );
    }
}

void motorTask(void*)
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    uint8_t consecutiveCommunicationFailures = 0;

    while (true)
    {
        const uint32_t nowMs = millis();

        JointCommand command{};

        const bool commandAvailable =
            xQueuePeek(
                jointCommandMailbox,
                &command,
                0
            ) == pdPASS;

        if (
            commandAvailable &&
            movementPermitted(command, nowMs)
        )
        {
            if (sendJointCommandToMotors(command))
            {
                consecutiveCommunicationFailures = 0;
            }
            else
            {
                ++consecutiveCommunicationFailures;

                if (
                    consecutiveCommunicationFailures >= 3
                )
                {
                    raiseFault(
                        FaultCode::
                            MotorCommunicationFailure
                    );
                }
            }
        }
        else
{
    consecutiveCommunicationFailures = 0;

    const RobotState state =
        getSystemStatus().state;

    /*
     * While Active, a temporary target/command mismatch simply
     * prevents transmission for this cycle. It must not invoke
     * the policy intended for Hold, Fault, or Idle.
     */
    if (state != RobotState::Active)
    {
        applyNonActiveMotorPolicy(state);
    }
}

        vTaskDelayUntil(
            &previousWakeTime,
            Config::MOTOR_PERIOD
        );
    }
}

void diagnosticTask(void*)
{
    TickType_t previousWakeTime =
        xTaskGetTickCount();

    while (true)
    {
        const uint32_t nowMs = millis();

        const SystemStatus status =
            getSystemStatus();

        TargetPose target{};
        JointState jointState{};
        JointCommand command{};

        xQueuePeek(
            targetMailbox,
            &target,
            0
        );

        xQueuePeek(
            jointStateMailbox,
            &jointState,
            0
        );

        xQueuePeek(
            jointCommandMailbox,
            &command,
            0
        );

        Serial.print("State: ");
        Serial.print(
            static_cast<uint8_t>(status.state)
        );

        Serial.print(" | Fault: ");
        Serial.print(
            static_cast<uint16_t>(status.fault)
        );

        Serial.print(" | Motion permitted: ");
        Serial.print(
            status.motionCommandPermitted
                ? "YES"
                : "NO"
        );

        Serial.print(" | Target age: ");

        if (target.valid)
        {
            Serial.print(
                nowMs - target.receivedAtMs
            );
        }
        else
        {
            Serial.print("INVALID");
        }

        Serial.print(" | Joint state age: ");

        if (jointState.valid)
        {
            Serial.print(
                nowMs - jointState.measuredAtMs
            );
        }
        else
        {
            Serial.print("INVALID");
        }

        Serial.print(" | Command age: ");

        if (command.valid)
        {
            Serial.println(
                nowMs - command.generatedAtMs
            );
        }
        else
        {
            Serial.println("INVALID");
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::DIAGNOSTIC_PERIOD
        );
    }
}

bool publishTarget(TargetPose target)
{
    // This function must have only one producer:
    // the future communication task.
    static bool havePreviousSequence = false;
    static uint32_t previousSequence = 0;

    if (targetMailbox == nullptr)
    {
        return false;
    }

    // Always use the Teensy's clock for timeout enforcement.
    target.receivedAtMs = millis();
    target.valid = true;

    if (!validateTargetFields(target))
    {
        return false;
    }

    /*
     * Signed subtraction permits normal uint32_t wraparound
     * while rejecting duplicate and older packets.
     */
    if (
        havePreviousSequence &&
        static_cast<int32_t>(
            target.sequence - previousSequence
        ) <= 0
    )
    {
        return false;
    }

    if (
        xQueueOverwrite(
            targetMailbox,
            &target
        ) != pdPASS
    )
    {
        return false;
    }

    // Commit the sequence only after publication succeeds.
    previousSequence = target.sequence;
    havePreviousSequence = true;

    return true;
}

bool publishJointState(JointState state)
{
    if (jointStateMailbox == nullptr)
    {
        return false;
    }

    state.measuredAtMs = millis();
    state.valid = true;

    return
        xQueueOverwrite(
            jointStateMailbox,
            &state
        ) == pdPASS;
}

// ============================================================
// Protected system-state access
// ============================================================

SystemStatus getSystemStatus()
{
    SystemStatus snapshot;

    taskENTER_CRITICAL();
    snapshot = systemStatus;
    taskEXIT_CRITICAL();

    return snapshot;
}

void setMotionCommandPermission(bool permitted)
{
    taskENTER_CRITICAL();

    systemStatus.motionCommandPermitted =
        Config::ENABLE_MOTOR_TRANSMISSION &&
        permitted &&
        systemStatus.state == RobotState::Active &&
        systemStatus.fault == FaultCode::None;

    taskEXIT_CRITICAL();
}

void enterHold()
{
    taskENTER_CRITICAL();

    if (
        systemStatus.state == RobotState::Active &&
        systemStatus.fault == FaultCode::None
    )
    {
        systemStatus.state = RobotState::Hold;
        systemStatus.stateChangedAtMs = millis();
    }

    systemStatus.motionCommandPermitted = false;

    taskEXIT_CRITICAL();

    /*
     * Invalidate the old command. Returning to Active will
     * therefore require a newly calculated command.
     */
    if (jointCommandMailbox != nullptr)
    {
        const JointCommand invalidCommand{};

        xQueueOverwrite(
            jointCommandMailbox,
            &invalidCommand
        );
    }
}

void raiseFault(FaultCode fault)
{
    if (fault == FaultCode::None)
    {
        return;
    }

    taskENTER_CRITICAL();

    /*
     * Preserve the first detected fault. Otherwise a secondary
     * failure could hide the event that started the problem.
     */
    if (systemStatus.fault == FaultCode::None)
    {
        systemStatus.fault = fault;
        systemStatus.faultRaisedAtMs = millis();
    }

    if (
        fault == FaultCode::EmergencyStopActive ||
        systemStatus.state == RobotState::EmergencyStop
    )
    {
        systemStatus.state = RobotState::EmergencyStop;
    }
    else
    {
        systemStatus.state = RobotState::Fault;
    }

    systemStatus.stateChangedAtMs = millis();
    systemStatus.motionCommandPermitted = false;

    taskEXIT_CRITICAL();

    if (jointCommandMailbox != nullptr)
    {
        const JointCommand invalidCommand{};

        xQueueOverwrite(
            jointCommandMailbox,
            &invalidCommand
        );
    }

    /*
     * TODO:
     * Directly operate the physical motor-enable output here.
     *
     * The eventual implementation must distinguish between:
     * - controlled hold,
     * - torque disable,
     * - motor-power relay opening.
     */
}

// ============================================================
// Target and command validation
// ============================================================

bool validateTargetFields(const TargetPose& target)
{
    const bool positionFinite =
        isfinite(target.position.x) &&
        isfinite(target.position.y) &&
        isfinite(target.position.z);

    const bool orientationFinite =
        isfinite(target.orientation.w) &&
        isfinite(target.orientation.x) &&
        isfinite(target.orientation.y) &&
        isfinite(target.orientation.z);

    if (!positionFinite || !orientationFinite)
    {
        return false;
    }

    const float quaternionNormSquared =
        target.orientation.w * target.orientation.w +
        target.orientation.x * target.orientation.x +
        target.orientation.y * target.orientation.y +
        target.orientation.z * target.orientation.z;

    /*
     * This permits a small numerical error but rejects a clearly
     * invalid orientation. Later, the communication layer can
     * normalize accepted quaternions before publication.
     */
    if (
        quaternionNormSquared < 0.95f ||
        quaternionNormSquared > 1.05f
    )
    {
        return false;
    }

    if (
        !isfinite(target.maximumLinearVelocity) ||
        !isfinite(target.maximumAngularVelocity) ||
        target.maximumLinearVelocity < 0.0f ||
        target.maximumAngularVelocity < 0.0f
    )
    {
        return false;
    }

    if (
        target.validityMs == 0 ||
        target.validityMs >
            Config::MAX_TARGET_VALIDITY_MS
    )
    {
        return false;
    }

    /*
     * TODO:
     * Add the true Cartesian workspace limits after the final
     * base frame and V2 link dimensions are fixed.
     */

    return true;
}

bool targetIsFresh(
    const TargetPose& target,
    uint32_t nowMs
)
{
    return
        target.valid &&
        validateTargetFields(target) &&
        static_cast<uint32_t>(
            nowMs - target.receivedAtMs
        ) <= target.validityMs;
}

bool jointStateIsFresh(
    const JointState& state,
    uint32_t nowMs
)
{
    if (
        !state.valid ||
        state.onlineMask !=
            Config::ALL_JOINTS_ONLINE_MASK
    )
    {
        return false;
    }

    if (
        static_cast<uint32_t>(
            nowMs - state.measuredAtMs
        ) > Config::JOINT_STATE_MAX_AGE_MS
    )
    {
        return false;
    }

    for (
        uint8_t joint = 0;
        joint < Config::JOINT_COUNT;
        ++joint
    )
    {
        if (
            !isfinite(state.positionRad[joint]) ||
            !isfinite(
                state.velocityRadPerSecond[joint]
            )
        )
        {
            return false;
        }
    }

    return true;
}

bool validateJointCommand(
    const JointCommand& command
)
{
    if (
        command.validityMs == 0 ||
        command.validityMs >
            Config::JOINT_COMMAND_VALIDITY_MS
    )
    {
        return false;
    }

    for (
        uint8_t joint = 0;
        joint < Config::JOINT_COUNT;
        ++joint
    )
    {
        if (
            !isfinite(command.positionRad[joint]) ||
            !isfinite(
                command.velocityLimitRadPerSecond[joint]
            ) ||
            command.velocityLimitRadPerSecond[joint] < 0.0f
        )
        {
            return false;
        }

        /*
         * TODO:
         * Apply final V2 limits for every joint here.
         */
    }

    return true;
}

bool jointCommandIsFresh(
    const JointCommand& command,
    uint32_t nowMs
)
{
    return
        command.valid &&
        validateJointCommand(command) &&
        static_cast<uint32_t>(
            nowMs - command.generatedAtMs
        ) <= command.validityMs;
}

bool commandMatchesCurrentData(
    const JointCommand& command,
    uint32_t nowMs
)
{
    TargetPose target{};
    JointState measuredState{};

    if (
        targetMailbox == nullptr ||
        jointStateMailbox == nullptr
    )
    {
        return false;
    }

    if (
        xQueuePeek(
            targetMailbox,
            &target,
            0
        ) != pdPASS
    )
    {
        return false;
    }

    if (
        xQueuePeek(
            jointStateMailbox,
            &measuredState,
            0
        ) != pdPASS
    )
    {
        return false;
    }

    return
        targetIsFresh(target, nowMs) &&
        jointStateIsFresh(measuredState, nowMs) &&
        jointCommandIsFresh(command, nowMs) &&
        command.sourceSequence == target.sequence;
}

bool controlDataReady(uint32_t nowMs)
{
    JointCommand command{};

    if (
        jointCommandMailbox == nullptr ||
        xQueuePeek(
            jointCommandMailbox,
            &command,
            0
        ) != pdPASS
    )
    {
        return false;
    }

    return commandMatchesCurrentData(
        command,
        nowMs
    );
}

bool movementPermitted(
    const JointCommand& command,
    uint32_t nowMs
)
{
    const SystemStatus status =
        getSystemStatus();

    return
        Config::ENABLE_MOTOR_TRANSMISSION &&
        status.state == RobotState::Active &&
        status.fault == FaultCode::None &&
        status.motionCommandPermitted &&
        commandMatchesCurrentData(
            command,
            nowMs
        );
}

// ============================================================
// Kinematics and motor placeholders
// ============================================================

bool calculateInverseKinematics(
    const TargetPose& target,
    const JointState& measuredState,
    JointCommand& command
)
{
    (void)target;
    (void)measuredState;

    /*
     * Keep the command invalid until the 7-DOF IK,
     * joint limits, and redundancy policy exist.
     */
    command = {};

    return false;
}

bool sendJointCommandToMotors(
    const JointCommand& command
)
{
    (void)command;

    /*
     * Deliberately disabled.
     *
     * Returning false ensures that accidental entry into
     * Active cannot look like successful motor operation.
     */
    return false;
}

void applyNonActiveMotorPolicy(
    RobotState state
)
{
    (void)state;

    /*
     * This will be implemented after the Dynamixel layer:
     *
     * Idle/Ready:
     *     Commissioning torque policy.
     *
     * Hold:
     *     Hold the last safe measured joint positions.
     *
     * Fault:
     *     Controlled hold or torque-off depending on cause.
     *
     * EmergencyStop:
     *     Open the physical normally-closed safety relay.
     */
}