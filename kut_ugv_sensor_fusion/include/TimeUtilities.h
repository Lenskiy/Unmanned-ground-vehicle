/*
 * TimeUtilities.h
 *
 *  Created on: Apr 15, 2013
 *      Author: Daniel Oñoro Rubio
 */

#ifndef TIMEUTILITIES_H_
#define TIMEUTILITIES_H_

#include <sys/time.h>

//MACRO
#define TIME_THIS(X,Y) \
{ \
    struct timeval t_ini, t_fin; \
    gettimeofday(&t_ini, NULL); \
    X; \
    gettimeofday(&t_fin, NULL); \
    Y=timeval_diff(&t_fin, &t_ini); \
}

//Structs
double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) -
    (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

class TimeUtilities
{
public:
  TimeUtilities():work_begin(0),work_fps(0){};
  ~TimeUtilities(){};

  inline void workBegin() {
          work_begin = getTickCount();
  }

  inline double workEnd() {
          int64 delta = getTickCount() - hog_work_begin;
          double freq = getTickFrequency();
          work_fps = freq / delta;
          return work_fps;
  }

  inline string toString() const {
          stringstream ss;
          ss << work_fps;
          return ss.str();
  }

private:
  int64 work_begin,work_fps;
};

#endif /* TIMEUTILITIES_H_ */
