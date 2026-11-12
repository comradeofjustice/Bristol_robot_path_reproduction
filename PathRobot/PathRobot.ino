#include <Arduino.h>
#include "Robot.h"
#include "OdometryModel.h"
#include "PathPlan.h"

constexpr uint32_t AUTO_START_DELAY_MS = 3000;
constexpr uint32_t CONTROL_MS = 10;
constexpr uint32_t SAMPLE_MS = 50;
constexpr size_t CAPACITY = PATH_DURATION_MS / SAMPLE_MS + 2;

Robot_c robot;
OdometryModel_c odometry(16.0f, 90.0f, 358.3f);

struct Sample {
  uint32_t ms;
  size_t segment;
  int32_t left, right;
  OdometryModel_c::Pose pose;
};
Sample samples[CAPACITY];
size_t count = 0;
enum RunState { IDLE, RUNNING, STOPPING };
RunState state = IDLE;
uint32_t start_ms = 0, last_control = 0, next_sample = 0;
size_t active_segment = PATH_COUNT;
const char* outcome = "complete";

bool auto_start_pending = true;
uint32_t startup_ms = 0;

size_t segmentAt(uint32_t ms) {
  for (size_t i = 0; i < PATH_COUNT; ++i) {
    if (ms < PATH[i].duration) return i;
    ms -= PATH[i].duration;
  }
  return PATH_COUNT;
}

bool updatePose() {
  if (!robot.getEncoders()) return false;
  odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  return true;
}

void sample(uint32_t elapsed, size_t segment) {
  if (count < CAPACITY) {
    samples[count++] = {elapsed, segment, robot.getLeftEncoderCount(),
                       robot.getRightEncoderCount(), odometry.pose};
  }
}

void startRun() {
  if (!robot.setMotorPWM(0, 0)) {
    Serial.println("# REFUSED: motor stop not acknowledged");
    return;
  }
  odometry.reset();
  if (!updatePose()) {
    Serial.println("# REFUSED: no encoder baseline");
    return;
  }
  count = 0;
  sample(0, 0);
  if (!robot.setMotorPWM(PATH[0].left, PATH[0].right)) {
    outcome = "start_command_failed";
    state = STOPPING;
    return;
  }
  start_ms = last_control = millis();
  robot.setLED(0, 255, 0, 100); // Video synchronization cue near command time zero.
  active_segment = 0;
  next_sample = SAMPLE_MS;
  state = RUNNING;
}

void dumpRun() {
  Serial.println("t_ms,segment,left_pwm,right_pwm,left_count,right_count,x_mm,y_mm,theta_rad");
  for (size_t i = 0; i < count; ++i) {
    const Sample& s = samples[i];
    const Segment* p = s.segment < PATH_COUNT ? &PATH[s.segment] : nullptr;
    Serial.printf("%lu,%s,%d,%d,%ld,%ld,%.4f,%.4f,%.7f\n",
      (unsigned long)s.ms, p ? p->name : "done",
      p ? p->left : 0, p ? p->right : 0, (long)s.left, (long)s.right,
      s.pose.x, s.pose.y, s.pose.theta);
  }
  Serial.printf("# RUN_END %s; reboot to run again\n", outcome);
  state = IDLE;
}

void setup() {
  Serial.begin(9600, SERIAL_8N1, 20, 21);
  robot.initialise();
  Wire.setTimeOut(20);
  robot.setMotorPWM(0, 0);
  Serial.println("# PathRobot: auto-start in 3 seconds; x=cancel/stop.");
  startup_ms = millis();
}

void loop() {
  bool stop = false;
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    const int c = Serial.read();
    stop |= c == 'x' || c == 'X';
  }
  if (stop && state == RUNNING) {
    state = STOPPING;
    outcome = "user_stop";
  }
  if (auto_start_pending && stop) {
    auto_start_pending = false;
    Serial.println("# AUTO_START cancelled");
  }
  if (auto_start_pending && millis() - startup_ms >= AUTO_START_DELAY_MS) {
    auto_start_pending = false; // One attempt per boot, including a refused start.
    startRun();
  }

  if (state == RUNNING) {
    const uint32_t now = millis();
    const uint32_t elapsed = now - start_ms;
    if (now - last_control < CONTROL_MS) return;
    last_control = now;

    if (elapsed >= PATH_DURATION_MS) {
      state = STOPPING;
      outcome = "complete";
    } else if (!updatePose()) {
      state = STOPPING;
      outcome = "encoder_read_failed";
    } else {
      const size_t segment = segmentAt(elapsed);
      // Refresh the request each control tick, matching the template cadence.
      if (!robot.setMotorPWM(PATH[segment].left, PATH[segment].right)) {
        state = STOPPING;
        outcome = "motor_command_failed";
      } else {
        active_segment = segment;
      }
      if (state == RUNNING && elapsed >= next_sample) {
        sample(elapsed, active_segment);
        next_sample = (elapsed / SAMPLE_MS + 1) * SAMPLE_MS;
      }
    }
  }

  if (state == STOPPING) {
    // Retry zero output and do not print the log until the stop is acknowledged.
    if (!robot.setMotorPWM(0, 0)) return;
    if (count && strcmp(outcome, "complete") == 0) {
      if (updatePose()) sample(millis() - start_ms, PATH_COUNT);
      else outcome = "final_encoder_read_failed";
    }
    robot.setLED(0, 0, 0, 0);
    dumpRun();
  }
}
