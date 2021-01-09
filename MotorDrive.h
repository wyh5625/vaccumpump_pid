/*
 * MotorDrive.h - class of motor drive that has definition of motor drive 
 * Created by William W.
 * It allow user to quickly define a motor driver and conviently interactive with it
 * and the functions of it are easy to use
 */
#ifndef MotorDrive_h
#define MotorDrive_h


class MotorDrive{
  public:
    MotorDrive(int M1, int M2, int PWM);
    void MoveForward(int S);
    void MoveBackward(int S);
    void Stop();
  private:
    int m1;
    int m2;
    int pwm;
};
#endif
