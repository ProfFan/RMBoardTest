//
// Created by Fan Jiang on 2017/5/3.
//

#ifndef CHASSIS_PID_H
#define CHASSIS_PID_H
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class PID {
public:

  PID(float p, float i, float d, float dt);

  void initialize(float value);

  float run(float input);

private:
  float Kp, Ki, Kd;
  float dt;
  float lastError;
  float integral;
  float maxIntegral;
};

#endif
#endif //CHASSIS_PID_H
