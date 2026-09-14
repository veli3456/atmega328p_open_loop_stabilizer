#ifndef SERVO_H
#define SERVO_H

typedef struct {
    float roll;
    float pitch;
}servo_angle_t;

void servo_change_angle (servo_angle_t *servo_angle);
void SERVO_INIT (void);

#endif