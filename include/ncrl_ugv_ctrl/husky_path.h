#ifndef HUSKY_PATH_H
#define HUSKY_PATH_H
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <stdio.h>
#include "ncrl_tf.h"
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <queue>
#include <thread>
#include <mutex> // for lock
#include <stdio.h>
#include <iostream>
#include <vector>

struct XYZyaw{
  float x; float y; float z; float yaw;
};

struct PID{
  float KP; float KI; float KD;
  float PR; float IN; float DE;
};

struct trajectory{
  float px; float vx; float ax;
  float py; float vy; float ay;
};

void confine(float &data, float threshold){
  if(data >= threshold || data <= -threshold){
    if(data >= threshold)
      data = threshold;
    else
      data = -threshold;
  }
  else{
    data = data;
  }
}

void pid_compute(PID &pid, float &r, const float &e, const float &e_l, float c){
  pid.PR = e;
  pid.IN += e;
  pid.DE = e - e_l;
  confine(pid.IN, c);
  r = (pid.PR * pid.KP + pid.IN * pid.KI + pid.DE * pid.KD);
}

char getch(){
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

#endif // HUSKY_PATH_H
