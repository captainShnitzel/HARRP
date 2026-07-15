//
#include <Arduino.h>
#include "arduino_freertos.h"
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

constexpr uint16_t TASK_STACK_SIZE = 512;
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
    CommandTimeout,
    InvalidTarget,
    IkFailure,
    JointLimitViolation,
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
    Vector3 position;
    Quaternion orientation;

    float maximumLinearVelocity;
    float maximumAngularVelocity;

    uint32_t sequence;
    uint32_t timestampMs;
    uint32_t validityMs;

    bool valid;
};

struct JointState
{
    float positionRad[7];
    float velocityRadPerSecond[7];

    uint32_t timestampMs;
    bool valid;
};

struct JointCommand
{
    float positionRad[7];
    float velocityLimitRadPerSecond[7];

    uint32_t sourceSequence;
    bool valid;
};

// ============================================================
// Global RTOS and system state
// ============================================================

TaskHandle_t safetyTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t diagnosticTaskHandle = nullptr;

RobotState robotState = RobotState::Boot;
FaultCode activeFault = FaultCode::None;

TargetPose currentTarget{};
JointState currentJointState{};
JointCommand currentJointCommand{};

bool motorOutputEnabled = false;

// ============================================================
// Forward declarations
// ============================================================

void safetyTask(void* parameter);
void controlTask(void* parameter);
void motorTask(void* parameter);
void diagnosticTask(void* parameter);

void initializeSystem();
void createTasks();

bool validateTarget(const TargetPose& target);
bool calculateInverseKinematics(
    const TargetPose& target,
    JointCommand& command
);

void raiseFault(FaultCode fault);
bool movementPermitted();

// ============================================================
// Arduino entry points
// ============================================================

void setup()
{
    Serial.begin(Config::SERIAL_BAUD);

    initializeSystem();
    createTasks();

    vTaskStartScheduler();

    // The scheduler should never return.
    while (true)
    {
    }
}

void loop()
{
    // Unused after the FreeRTOS scheduler starts.
}

// ============================================================
// Initialization
// ============================================================

void initializeSystem()
{
    robotState = RobotState::SelfTest;
    activeFault = FaultCode::None;
    motorOutputEnabled = false;

    currentTarget.valid = false;
    currentJointState.valid = false;
    currentJointCommand.valid = false;

    // Hardware self-test will be added later.

    robotState = RobotState::Idle;
}

void createTasks()
{
    xTaskCreate(
        safetyTask,
        "Safety",
        Config::TASK_STACK_SIZE,
        nullptr,
        4,
        &safetyTaskHandle
    );

    xTaskCreate(
        motorTask,
        "Motor",
        Config::TASK_STACK_SIZE,
        nullptr,
        3,
        &motorTaskHandle
    );

    xTaskCreate(
        controlTask,
        "Control",
        Config::TASK_STACK_SIZE,
        nullptr,
        2,
        &controlTaskHandle
    );

    xTaskCreate(
        diagnosticTask,
        "Diagnostic",
        Config::TASK_STACK_SIZE,
        nullptr,
        1,
        &diagnosticTaskHandle
    );
}

// ============================================================
// RTOS tasks
// ============================================================

void safetyTask(void*)
{
    TickType_t previousWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Check state, command age, faults and motor permission.

        motorOutputEnabled = movementPermitted();

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
        if (
            robotState == RobotState::Active &&
            validateTarget(currentTarget)
        )
        {
            if (!calculateInverseKinematics(
                    currentTarget,
                    currentJointCommand))
            {
                raiseFault(FaultCode::IkFailure);
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
    TickType_t previousWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (
            motorOutputEnabled &&
            currentJointCommand.valid
        )
        {
            // Future:
            // send currentJointCommand to the Dynamixel motors.
        }
        else
        {
            // Remain disarmed.
        }

        vTaskDelayUntil(
            &previousWakeTime,
            Config::MOTOR_PERIOD
        );
    }
}

void diagnosticTask(void*)
{
    TickType_t previousWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.print("State: ");
        Serial.print(static_cast<uint8_t>(robotState));

        Serial.print(" | Fault: ");
        Serial.print(static_cast<uint16_t>(activeFault));

        Serial.print(" | Motor enabled: ");
        Serial.println(motorOutputEnabled ? "YES" : "NO");

        vTaskDelayUntil(
            &previousWakeTime,
            Config::DIAGNOSTIC_PERIOD
        );
    }
}

// ============================================================
// State and safety
// ============================================================

bool movementPermitted()
{
    return
        robotState == RobotState::Active &&
        activeFault == FaultCode::None &&
        currentTarget.valid &&
        currentJointCommand.valid;
}

void raiseFault(FaultCode fault)
{
    activeFault = fault;
    robotState = RobotState::Fault;

    motorOutputEnabled = false;
    currentJointCommand.valid = false;
}

// ============================================================
// Target validation
// ============================================================

bool validateTarget(const TargetPose& target)
{
    if (!target.valid)
    {
        return false;
    }

    // Workspace limits and timeout checks come later.

    return true;
}

// ============================================================
// Kinematics placeholder
// ============================================================

bool calculateInverseKinematics(
    const TargetPose& target,
    JointCommand& command)
{
    (void)target;

    // IK will be implemented later.
    command.valid = false;

    return false;
}