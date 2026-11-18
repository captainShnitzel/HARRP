//HARRP V2.0 Teensy 4.1 FreeRTOS control software v1.06
// Software-only motion pipeline. DO NOT enable motors until hardware validation.
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

constexpr TickType_t TRAJECTORY_PERIOD =
    pdMS_TO_TICKS(10);  // 100 Hz

constexpr TickType_t MOTOR_PERIOD =
    pdMS_TO_TICKS(10);      // 100 Hz

constexpr TickType_t DIAGNOSTIC_PERIOD =
    pdMS_TO_TICKS(1000);    // 1 Hz

// FreeRTOS stack depths are expressed in StackType_t units,
// not bytes. On Teensy 4.1 this will normally mean 32-bit words.
constexpr uint16_t SAFETY_STACK_DEPTH = 512;
constexpr uint16_t CONTROL_STACK_DEPTH = 1024;
constexpr uint16_t TRAJECTORY_STACK_DEPTH = 768;
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

constexpr uint32_t JOINT_GOAL_VALIDITY_MS = 50;

constexpr uint32_t TASK_WATCHDOG_MS = 300;
constexpr uint32_t TASK_WATCHDOG_BOOT_GRACE_MS = 2000;

// Hardware configuration — NOT YET VERIFIED
constexpr bool ESTOP_PIN_CONFIGURED = false;
constexpr uint8_t ESTOP_PIN = 255;
constexpr uint8_t ESTOP_RELEASED_LEVEL = arduino::LOW;

constexpr bool MOTOR_ENABLE_PIN_CONFIGURED = false;
constexpr uint8_t MOTOR_ENABLE_PIN = 255;
constexpr uint8_t MOTOR_ENABLE_DISABLED_LEVEL = arduino::LOW;

// Physical joint limits — NOT YET CONFIGURED
constexpr bool JOINT_LIMITS_CONFIGURED = false;
constexpr bool KINEMATICS_CONFIGURED = false;

constexpr float JOINT_MIN_RAD[7] =
    {0, 0, 0, 0, 0, 0, 0};

constexpr float JOINT_MAX_RAD[7] =
    {0, 0, 0, 0, 0, 0, 0};

constexpr float JOINT_MAX_VELOCITY_RAD_S[7] =
    {0, 0, 0, 0, 0, 0, 0};

constexpr float JOINT_MAX_ACCEL_RAD_S2[7] =
    {0, 0, 0, 0, 0, 0, 0};


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
    EmergencyStopActive,
    TaskWatchdogFailure,
    JointGoalTimeout,
    TrajectoryFailure
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

struct JointGoal
{
    float positionRad[Config::JOINT_COUNT];

    float velocityLimitRadPerSecond[Config::JOINT_COUNT];

    uint32_t sourceSequence;
    uint32_t generatedAtMs;
    uint32_t validityMs;

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
TaskHandle_t trajectoryTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t diagnosticTaskHandle = nullptr;

// Length-one queues act as latest-value mailboxes.
QueueHandle_t targetMailbox = nullptr;
QueueHandle_t jointStateMailbox = nullptr;
QueueHandle_t jointGoalMailbox = nullptr;
QueueHandle_t jointCommandMailbox = nullptr;

volatile uint32_t controlHeartbeatMs = 0;
volatile uint32_t trajectoryHeartbeatMs = 0;
volatile uint32_t motorHeartbeatMs = 0;

volatile uint32_t safetyLoopCount = 0;
volatile uint32_t controlLoopCount = 0;
volatile uint32_t trajectoryLoopCount = 0;
volatile uint32_t motorLoopCount = 0;

struct AxisProfile
{
    float positionRad;
    float velocityRadPerSecond;
};

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
void trajectoryTask(void* parameter);
void motorTask(void* parameter);
void diagnosticTask(void* parameter);

void initializeSystem();
bool createRtosObjects();
bool createTasks();

[[noreturn]] void stopOnStartupFailure();

SystemStatus getSystemStatus();
void setMotionCommandPermission(bool permitted);
void enterHold();
bool requestReady();
bool requestActive();
bool requestIdle();

void forceMotorEnableDisabled();
bool estopIsReleased();

bool runTrajectoryMathSelfTest();
void raiseFault(FaultCode fault);

bool validateTargetFields(const TargetPose& target);
bool targetIsFresh(const TargetPose& target, uint32_t nowMs);

bool jointStateIsFresh(
    const JointState& state,
    uint32_t nowMs
);

bool validateJointGoal(const JointGoal& goal);

bool jointGoalIsFresh(
    const JointGoal& goal,
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
    JointGoal& goal
);

// Trajectory generator
bool advanceAxisProfile(
    AxisProfile& axis,
    float target,
    float velocityLimit,
    float accelerationLimit,
    float dt
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

    if (!runTrajectoryMathSelfTest())
    {
    Serial.println("FATAL: trajectory math self-test failed");
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

void forceMotorEnableDisabled() //Establishes the safety interface.
{
    if (Config::MOTOR_ENABLE_PIN_CONFIGURED)
    {
        digitalWrite(
            Config::MOTOR_ENABLE_PIN,
            Config::MOTOR_ENABLE_DISABLED_LEVEL
        );
    }
}

bool estopIsReleased()
{
    if (!Config::ESTOP_PIN_CONFIGURED)
        return false;

    return digitalRead(Config::ESTOP_PIN)
        == Config::ESTOP_RELEASED_LEVEL;
}

void initializeSystem() //Initializes only verified hardware interfaces, keeps motion disabled, and starts the controller in Idle without automatically entering an armed state.
{
    if (Config::MOTOR_ENABLE_PIN_CONFIGURED)
    {
        digitalWrite(
            Config::MOTOR_ENABLE_PIN,
            Config::MOTOR_ENABLE_DISABLED_LEVEL
        );

        pinMode(Config::MOTOR_ENABLE_PIN, arduino::OUTPUT);

        forceMotorEnableDisabled();
    }

    if (Config::ESTOP_PIN_CONFIGURED)
    {
        pinMode(Config::ESTOP_PIN, arduino::INPUT_PULLUP);
    }

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

    jointGoalMailbox =
        xQueueCreate(1, sizeof(JointGoal));

    jointCommandMailbox =
        xQueueCreate(1, sizeof(JointCommand));

    if (
        targetMailbox == nullptr ||
        jointStateMailbox == nullptr ||
        jointGoalMailbox == nullptr ||
        jointCommandMailbox == nullptr
    )
    {
        return false;
    }

    // Put known invalid values in all mailboxes.
    const TargetPose invalidTarget{};
    const JointState invalidJointState{};
    const JointCommand invalidJointCommand{};
    const JointGoal invalidJointGoal{};

    xQueueOverwrite(
        jointGoalMailbox,
        &invalidJointGoal
    );

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
            trajectoryTask,
            "Trajectory",
            Config::TRAJECTORY_STACK_DEPTH,
            nullptr,
            3,
            &trajectoryTaskHandle
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

    forceMotorEnableDisabled();

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
    TickType_t previousWakeTime = xTaskGetTickCount();
    const uint32_t bootMs = millis();

    while (true)
    {
        const uint32_t nowMs = millis();

        ++safetyLoopCount;

        const SystemStatus status = getSystemStatus();

        // Never authorize movement without configured safety hardware.
        if (!Config::ESTOP_PIN_CONFIGURED ||
            !Config::MOTOR_ENABLE_PIN_CONFIGURED)
        {
            setMotionCommandPermission(false);
            forceMotorEnableDisabled();
        }
        else if (!estopIsReleased())
        {
            forceMotorEnableDisabled();
            raiseFault(FaultCode::EmergencyStopActive);
        }
        else if (status.state == RobotState::Active)
        {
            TargetPose target{};
            JointState measured{};

            const bool targetFresh =
                xQueuePeek(targetMailbox, &target, 0) == pdPASS &&
                targetIsFresh(target, nowMs);

            const bool stateFresh =
                xQueuePeek(jointStateMailbox, &measured, 0) == pdPASS &&
                jointStateIsFresh(measured, nowMs);

            if (!targetFresh || !stateFresh)
            {
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

        // Detect stalled control, trajectory, or motor tasks.
        if (
            (uint32_t)(nowMs - bootMs) >
                Config::TASK_WATCHDOG_BOOT_GRACE_MS &&
            (
                (uint32_t)(nowMs - controlHeartbeatMs) >
                    Config::TASK_WATCHDOG_MS ||

                (uint32_t)(nowMs - trajectoryHeartbeatMs) >
                    Config::TASK_WATCHDOG_MS ||

                (uint32_t)(nowMs - motorHeartbeatMs) >
                    Config::TASK_WATCHDOG_MS
            )
        )
        {
            forceMotorEnableDisabled();
            raiseFault(FaultCode::TaskWatchdogFailure);
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::SAFETY_PERIOD
        );
    }
}

void controlTask(void*)
{
    TickType_t previousWakeTime = xTaskGetTickCount();

    while (true)
    {
        const uint32_t nowMs = millis();

        controlHeartbeatMs = nowMs;
        ++controlLoopCount;

        const SystemStatus status = getSystemStatus();

        if (
            status.state == RobotState::Ready ||
            status.state == RobotState::Active
        )
        {
            TargetPose target{};
            JointState measured{};

            const bool targetFresh =
                xQueuePeek(
                    targetMailbox,
                    &target,
                    0
                ) == pdPASS &&
                targetIsFresh(target, nowMs);

            const bool feedbackFresh =
                xQueuePeek(
                    jointStateMailbox,
                    &measured,
                    0
                ) == pdPASS &&
                jointStateIsFresh(measured, nowMs);

            if (targetFresh && feedbackFresh)
            {
                JointGoal goal{};

                if (!calculateInverseKinematics(
                    target,
                    measured,
                    goal
                ))
                {
                    raiseFault(FaultCode::IkFailure);
                }
                else
                {
                    goal.sourceSequence = target.sequence;
                    goal.generatedAtMs = nowMs;
                    goal.validityMs =
                        Config::JOINT_GOAL_VALIDITY_MS;
                    goal.valid = true;

                    if (!validateJointGoal(goal))
                    {
                        raiseFault(
                            FaultCode::InvalidJointCommand
                        );
                    }
                    else
                    {
                        xQueueOverwrite(
                            jointGoalMailbox,
                            &goal
                        );
                    }
                }
            }
            else if (status.state == RobotState::Active)
            {
                enterHold();
            }
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::CONTROL_PERIOD
        );
    }
}

void trajectoryTask(void*)
{
    TickType_t previousWakeTime = xTaskGetTickCount();

    AxisProfile profile[Config::JOINT_COUNT]{};

    bool initialized = false;
    uint32_t lastUpdateMs = 0;

    while (true)
    {
        const uint32_t nowMs = millis();

        trajectoryHeartbeatMs = nowMs;
        ++trajectoryLoopCount;

        const SystemStatus status = getSystemStatus();

        JointGoal goal{};
        JointState measured{};
        TargetPose target{};

        const bool allowedState =
            status.state == RobotState::Ready ||
            status.state == RobotState::Active;

        const bool validData =
            allowedState &&
            Config::JOINT_LIMITS_CONFIGURED &&

            xQueuePeek(
                jointGoalMailbox,
                &goal,
                0
            ) == pdPASS &&

            jointGoalIsFresh(goal, nowMs) &&

            xQueuePeek(
                jointStateMailbox,
                &measured,
                0
            ) == pdPASS &&

            jointStateIsFresh(measured, nowMs) &&

            xQueuePeek(
                targetMailbox,
                &target,
                0
            ) == pdPASS &&

            targetIsFresh(target, nowMs) &&

            target.sequence == goal.sourceSequence;

        if (!validData)
        {
            initialized = false;

            const JointCommand invalid{};

            xQueueOverwrite(
                jointCommandMailbox,
                &invalid
            );
        }
        else
        {
            // Initialize from real joint feedback.
            if (!initialized)
            {
                for (
                    uint8_t j = 0;
                    j < Config::JOINT_COUNT;
                    ++j
                )
                {
                    profile[j].positionRad =
                        measured.positionRad[j];

                    profile[j].velocityRadPerSecond = 0.0f;
                }

                lastUpdateMs = nowMs;
                initialized = true;
            }

            const float dt = fminf(
                0.02f,
                fmaxf(
                    0.001f,
                    (uint32_t)(nowMs - lastUpdateMs) * 0.001f
                )
            );

            lastUpdateMs = nowMs;

            JointCommand command{};
            bool valid = true;

            for (
                uint8_t j = 0;
                j < Config::JOINT_COUNT;
                ++j
            )
            {
                const float velocityCap = fminf(
                    goal.velocityLimitRadPerSecond[j],
                    Config::JOINT_MAX_VELOCITY_RAD_S[j]
                );

                if (!advanceAxisProfile(profile[j], goal.positionRad[j], velocityCap, Config::JOINT_MAX_ACCEL_RAD_S2[j],dt))
                {
                    valid = false;
                    break;
                }

                command.positionRad[j] =
                    profile[j].positionRad;

                command.velocityLimitRadPerSecond[j] =
                    velocityCap;
            }

            if (!valid)
            {
                initialized = false;
                raiseFault(FaultCode::TrajectoryFailure);
            }
            else
            {
                command.sourceSequence = goal.sourceSequence;
                command.generatedAtMs = nowMs;
                command.validityMs =
                    Config::JOINT_COMMAND_VALIDITY_MS;
                command.valid = true;

                if (validateJointCommand(command))
                {
                    xQueueOverwrite(
                        jointCommandMailbox,
                        &command
                    );
                }
                else
                {
                    initialized = false;

                    raiseFault(
                        FaultCode::InvalidJointCommand
                    );
                }
            }
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::TRAJECTORY_PERIOD
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
        motorHeartbeatMs = nowMs;
        ++motorLoopCount;

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
        Serial.print(" | Task loops(S/C/T/M): ");

        Serial.print(safetyLoopCount);
        Serial.print('/');

        Serial.print(controlLoopCount);
        Serial.print('/');

        Serial.print(trajectoryLoopCount);
        Serial.print('/');

        Serial.print(motorLoopCount);

        Serial.print(" | ESTOP: ");

        Serial.print(
            Config::ESTOP_PIN_CONFIGURED
                ? (estopIsReleased() ? "RELEASED" : "ACTIVE")
                : "UNCONFIGURED"
        );

        Serial.print(" | Motors: ");

        Serial.print(
            Config::ENABLE_MOTOR_TRANSMISSION
                ? "ENABLED"
                : "LOCKED"
        );
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

static bool physicalSafetyConfigured()
{
    return Config::ESTOP_PIN_CONFIGURED &&
           Config::MOTOR_ENABLE_PIN_CONFIGURED &&
           Config::ESTOP_PIN != Config::MOTOR_ENABLE_PIN &&
           Config::ESTOP_PIN != 255 &&
           Config::MOTOR_ENABLE_PIN != 255;
}

static bool physicalJointLimitsConfigured()
{
    if (!Config::JOINT_LIMITS_CONFIGURED)
        return false;

    for (
        uint8_t j = 0;
        j < Config::JOINT_COUNT;
        ++j
    )
    {
        if (
            !isfinite(Config::JOINT_MIN_RAD[j]) ||
            !isfinite(Config::JOINT_MAX_RAD[j]) ||
            Config::JOINT_MIN_RAD[j] >=
                Config::JOINT_MAX_RAD[j] ||
            Config::JOINT_MAX_VELOCITY_RAD_S[j] <= 0 ||
            Config::JOINT_MAX_ACCEL_RAD_S2[j] <= 0
        )
        {
            return false;
        }
    }

    return true;
}

bool requestReady()
{
    if (
        !Config::ENABLE_MOTOR_TRANSMISSION ||
        !Config::KINEMATICS_CONFIGURED ||
        !physicalSafetyConfigured() ||
        !physicalJointLimitsConfigured() ||
        !estopIsReleased()
    )
    {
        return false;
    }

    JointState measured{};

    if (
        jointStateMailbox == nullptr ||
        xQueuePeek(
            jointStateMailbox,
            &measured,
            0
        ) != pdPASS ||
        !jointStateIsFresh(measured, millis())
    )
    {
        return false;
    }

    taskENTER_CRITICAL();

    const bool allowed =
        systemStatus.fault == FaultCode::None &&
        (
            systemStatus.state == RobotState::Idle ||
            systemStatus.state == RobotState::Hold
        );

    if (allowed)
    {
        systemStatus.state = RobotState::Ready;
        systemStatus.stateChangedAtMs = millis();
        systemStatus.motionCommandPermitted = false;
    }

    taskEXIT_CRITICAL();

    return allowed;
}

bool requestActive()
{
    if (
        !Config::ENABLE_MOTOR_TRANSMISSION ||
        !physicalSafetyConfigured() ||
        !estopIsReleased() ||
        !physicalJointLimitsConfigured() ||
        !Config::KINEMATICS_CONFIGURED ||
        !controlDataReady(millis())
    )
    {
        return false;
    }

    taskENTER_CRITICAL();

    const bool allowed =
        systemStatus.state == RobotState::Ready &&
        systemStatus.fault == FaultCode::None;

    if (allowed)
    {
        systemStatus.state = RobotState::Active;
        systemStatus.stateChangedAtMs = millis();

        systemStatus.motionCommandPermitted = false;
    }

    taskEXIT_CRITICAL();

    return allowed;
}

bool requestIdle()
{
    taskENTER_CRITICAL();

    const bool allowed =
        systemStatus.fault == FaultCode::None &&
        (
            systemStatus.state == RobotState::Ready ||
            systemStatus.state == RobotState::Hold
        );

    if (allowed)
    {
        systemStatus.state = RobotState::Idle;
        systemStatus.stateChangedAtMs = millis();
        systemStatus.motionCommandPermitted = false;
    }

    taskEXIT_CRITICAL();

    return allowed;
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

    if (jointGoalMailbox != nullptr)
    {
    const JointGoal invalidGoal{};

    xQueueOverwrite(
        jointGoalMailbox,
        &invalidGoal
    );
    }
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

    forceMotorEnableDisabled();
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

bool validateJointGoal(const JointGoal& goal)
{
    if (
        !Config::JOINT_LIMITS_CONFIGURED ||
        !goal.valid ||
        goal.validityMs == 0 ||
        goal.validityMs > Config::JOINT_GOAL_VALIDITY_MS
    )
    {
        return false;
    }

    for (
        uint8_t j = 0;
        j < Config::JOINT_COUNT;
        ++j
    )
    {
        if (
            !isfinite(goal.positionRad[j]) ||
            !isfinite(goal.velocityLimitRadPerSecond[j]) ||

            goal.positionRad[j] <
                Config::JOINT_MIN_RAD[j] ||

            goal.positionRad[j] >
                Config::JOINT_MAX_RAD[j] ||

            goal.velocityLimitRadPerSecond[j] <= 0 ||

            goal.velocityLimitRadPerSecond[j] >
                Config::JOINT_MAX_VELOCITY_RAD_S[j] ||

            Config::JOINT_MAX_ACCEL_RAD_S2[j] <= 0
        )
        {
            return false;
        }
    }

    return true;
}

bool jointGoalIsFresh(
    const JointGoal& goal,
    uint32_t nowMs
)
{
    return validateJointGoal(goal) &&
        (uint32_t)(nowMs - goal.generatedAtMs)
            <= goal.validityMs;
}

bool validateJointCommand(const JointCommand& command)
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

        if (
    !Config::JOINT_LIMITS_CONFIGURED ||

    command.positionRad[joint] <
        Config::JOINT_MIN_RAD[joint] ||

    command.positionRad[joint] >
        Config::JOINT_MAX_RAD[joint] ||

    command.velocityLimitRadPerSecond[joint] >
        Config::JOINT_MAX_VELOCITY_RAD_S[joint]
)
{
    return false;
}
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

static inline float clampFloat(
    float value,
    float lo,
    float hi
)
{
    return value < lo
        ? lo
        : (value > hi ? hi : value);
}

bool advanceAxisProfile(AxisProfile& axis, float target, float velocityLimit, float accelerationLimit, float dt)
{
    if (
        !isfinite(axis.positionRad) ||
        !isfinite(axis.velocityRadPerSecond) ||
        !isfinite(target) ||
        !isfinite(velocityLimit) ||
        !isfinite(accelerationLimit) ||
        !isfinite(dt) ||
        velocityLimit <= 0.0f ||
        accelerationLimit <= 0.0f ||
        dt <= 0.0f ||
        dt > 0.05f
    )
    {
        return false;
    }

    const float error =
        target - axis.positionRad;

    const float brakingSpeed = sqrtf(
        fmaxf(
            0.0f,
            2.0f * accelerationLimit * fabsf(error)
        )
    );

    const float brakingMargin =
        0.5f * accelerationLimit * dt;

    const float requestedSpeed =
        (
            error > 0 ? 1.0f :
            (error < 0 ? -1.0f : 0.0f)
        ) *
        fminf(
            velocityLimit,
            fmaxf(0.0f, brakingSpeed - brakingMargin)
        );

    const float deltaSpeed =
        accelerationLimit * dt;

    if (
        fabsf(error) <= accelerationLimit * dt * dt &&
        fabsf(axis.velocityRadPerSecond) <= deltaSpeed
    )
    {
        axis.positionRad = target;
        axis.velocityRadPerSecond = 0.0f;
    }
    else
    {
        const float newSpeed = clampFloat(
            requestedSpeed,

            axis.velocityRadPerSecond - deltaSpeed,

            axis.velocityRadPerSecond + deltaSpeed
        );

        axis.positionRad += newSpeed * dt;
        axis.velocityRadPerSecond = newSpeed;
    }

    return
        isfinite(axis.positionRad) &&
        isfinite(axis.velocityRadPerSecond);
}

bool runTrajectoryMathSelfTest()
{
    AxisProfile axis{0.0f, 0.0f};

    float previousPosition = axis.positionRad;
    float previousSpeed = axis.velocityRadPerSecond;

    bool reached = false;

    for (int i = 0; i < 1000; ++i)
    {
        if (!advanceAxisProfile(
            axis,
            1.0f,
            0.65f,
            1.5f,
            0.01f
        ))
        {
            return false;
        }

        if (
            fabsf(axis.velocityRadPerSecond) > 0.651f ||
            fabsf(axis.velocityRadPerSecond - previousSpeed)
                > 0.0151f ||
            axis.positionRad < previousPosition - 0.0001f ||
            axis.positionRad > 1.00001f
        )
        {
            return false;
        }

        previousPosition = axis.positionRad;
        previousSpeed = axis.velocityRadPerSecond;

        if (fabsf(axis.positionRad - 1.0f) < 0.001f)
        {
            reached = true;
            break;
        }
    }

    if (!reached)
        return false;

    // Invalid velocity limit must be rejected.
    if (advanceAxisProfile(
        axis, 2.0f, 0.0f, 1.5f, 0.01f
    ))
    {
        return false;
    }

    // NaN input must be rejected.
    if (advanceAxisProfile(
        axis, NAN, 0.65f, 1.5f, 0.01f
    ))
    {
        return false;
    }

    // Test reverse movement.
    for (int i = 0; i < 1000; ++i)
    {
        if (!advanceAxisProfile(
            axis,
            -0.5f,
            0.65f,
            1.5f,
            0.01f
        ))
        {
            return false;
        }
    }

    return fabsf(axis.positionRad + 0.5f) < 0.002f;
}

bool calculateInverseKinematics(const TargetPose& target, const JointState& measuredState, JointGoal& goal)
{
    (void)target;
    (void)measuredState;

    goal = {};

    // Real 7-DOF IK requires verified V2 joint axes,
    // link transforms, zero offsets and joint limits.

    return false;
}

bool sendJointCommandToMotors(
    const JointCommand& command
)
{
    (void)command;

    // Motor task owns the actuator communication interface.
    // Physical transmission remains disabled.

    return false;
}

// New trajectory calculation function
bool advanceAxisProfile(AxisProfile& axis, float goalPosition, float velocityLimit, float accelerationLimit, float dtSeconds);

bool sendJointCommandToMotors(const JointCommand& command);

void applyNonActiveMotorPolicy(RobotState state)
{
    (void)state;

    // Gravity-safe HOLD remains to be implemented.
    // Do not energize the motors in this build.

    forceMotorEnableDisabled();
}