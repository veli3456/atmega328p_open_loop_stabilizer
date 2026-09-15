#ifndef SERVO_H
#define SERVO_H

typedef enum {
    SERVO_1,
    SERVO_2
}servo_id_t;

typedef enum {
    SUCCESS_SERVO = 18,
    ERR_INVALID_SERVO_ID
}servo_status_t;

typedef struct {
    float roll;
    float pitch;
}servo_angle_t;

servo_status_t servo_change_angle (float servo_angle, servo_id_t servo_num);
void SERVO_INIT (void);

#endif