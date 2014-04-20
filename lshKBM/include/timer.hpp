#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <iostream>
#include <sys/time.h>

using namespace std;

class Timer
{
public:
  Timer()
  {
    gettimeofday(&t0, NULL);
  };

  void tic(void)
  {
    gettimeofday(&t0, NULL);
  };

  float toc(void)
  {
    gettimeofday(&t1, NULL);
    return getDtMs();
  };

  float lastDt(void) const
  {
    return dt;
  };

  Timer& operator=(const Timer& t)
  {
    if(this != &t){
      dt=t.lastDt();
    }
    return *this;
  };

private:
  float dt;
  timeval t0, t1;

  float getDtMs(void)
  {
    dt = (t1.tv_sec - t0.tv_sec) * 1000.0; // sec to ms
    dt += (t1.tv_usec - t0.tv_usec) / 1000.0; // us to ms
    return dt;
  };
};

inline ostream& operator<<(ostream &out, const Timer& t)
{
  out << t.lastDt() << "ms";
  return out;
};

#endif /* TIMER_HPP_ */
