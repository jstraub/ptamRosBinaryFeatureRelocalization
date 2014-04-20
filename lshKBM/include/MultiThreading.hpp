/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef MULTITHREADING_HPP_
#define MULTITHREADING_HPP_

#include <boost/thread/thread.hpp>

#include <stdio.h>
#include <vector>
#include <time.h>
#include <list>

using namespace std;

//#define MULTITHREADING_OUTPUT

template<class Result>
class SingleThread
{
public:
  SingleThread()
  : mIsRunning(false),
    mpThread(NULL)
  {};
  virtual ~SingleThread()
  {
    if(mpThread!=NULL) mpThread->join();
    delete mpThread;
    mpThread=NULL;
  };

  // starts thread
  void run() {
    assert(mpThread==NULL);
    mIsRunning=true;
    mpThread=new boost::thread(boost::bind(&SingleThread<Result>::doWork, this));
  };

  virtual bool isRunning() {
    boost::mutex::scoped_lock l(mRunningMutex);
    return mIsRunning;
  };

  virtual const Result& getResults()
  {
    boost::mutex::scoped_lock l(mResMutex); // lock before retrieving values
    return mRes;
  }

protected:

  Result mRes; // all resulting statistics
  boost::mutex mResMutex;

  virtual void doWork_impl() = 0;

private:

  boost::mutex mRunningMutex;
  bool mIsRunning;
  boost::thread* mpThread;

  void doWork()
  {// work in here
    doWork_impl();
#ifdef MULTITHREADING_OUTPUT
    cout<<"Thread stopped"<<endl;
#endif
    boost::mutex::scoped_lock l(mRunningMutex);
    mIsRunning=false;
  };
};

// Specialization of SingleThread which does not return a result
template<>
class SingleThread<void>
{
public:
  SingleThread()
  : mIsRunning(false),
    mpThread(NULL)
  {};
  virtual ~SingleThread()
  {
    if(mpThread!=NULL) mpThread->join();
    delete mpThread;
    mpThread=NULL;
  };

  // starts thread
  void run() {
    assert(mpThread==NULL);
    mIsRunning=true;
    mpThread=new boost::thread(boost::bind(&SingleThread<void>::doWork, this));
  };

  virtual bool isRunning() {
    boost::mutex::scoped_lock l(mRunningMutex);
    return mIsRunning;
  };

protected:

  virtual void doWork_impl() = 0;

private:

  boost::mutex mRunningMutex;
  bool mIsRunning;
  boost::thread* mpThread;

  void doWork()
  {// work in here
    doWork_impl();
#ifdef MULTITHREADING_OUTPUT
    cout<<"Thread stopped"<<endl;
#endif
    boost::mutex::scoped_lock l(mRunningMutex);
    mIsRunning=false;
  };
};

template<class Result>
class MultiThreads
{
public:
  MultiThreads(vector<SingleThread<Result>* >& threads, uint32_t maxThreads)
  : mMaxThreads(maxThreads), mThreads(threads), mAllRes(threads.size())
  {

  };
  ~MultiThreads()
  {};

  void work()
  {
    list<uint32_t> runningThreads;

    char buff[12]; sprintf(buff,"%6.2lf",double(0.0));
    cout<<"Progress: "<<string(buff);

    for(uint32_t tId=0; tId<mThreads.size(); ++tId)
    {
#ifdef MULTITHREADING_OUTPUT
    bool haveDisplayed=false;
#endif
      while(runningThreads.size()>=mMaxThreads)
      {
#ifdef MULTITHREADING_OUTPUT

        {// count number of running threads
          if(!haveDisplayed){
            cout<<"Threads running: ";
            for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
            {
              cout<<*it<<" ";
            }
            cout<<"=> "<<runningThreads.size()<<endl;
            haveDisplayed=true;
          }
        }
#endif
        for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
        {
          assert(mThreads[*it]!=NULL);
          if(!mThreads[*it]->isRunning())
          {
            // collect results
            assert(*it<mAllRes.size());
            mAllRes[*it]=mThreads[*it]->getResults();
            delete mThreads[*it]; mThreads[*it]=NULL;
            it=runningThreads.erase(it);
#ifdef MULTITHREADING_OUTPUT
            cout<<"deleted thread"<<endl;
#endif
          }
        }
        usleep(1000);
      }
#ifdef MULTITHREADING_OUTPUT
      time_t rawtime; time ( &rawtime );
      cout<<" ------------ Starting thread "<<tId<<" @ "<<asctime(localtime(&rawtime)) <<" ------------ " <<endl;
#endif
      mThreads[tId]->run();
      //assert(mThreads[tId]->isRunning());
      runningThreads.push_back(tId);
#ifdef MULTITHREADING_OUTPUT
      cout<<" ------------ Threads running:" <<runningThreads.size()<<" finished+running: "<<mThreads.size() <<" allinall todo: "<<mThreads.size()<<endl;
#endif
      if(tId % uint32_t(ceil(double(mThreads.size())/10000.0)) == 0)
      {
        char buff[10]; sprintf(buff,"%6.2lf",double(*(runningThreads.begin()))*100.0/double(mThreads.size()));
        cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
      }
    }

#ifdef MULTITHREADING_OUTPUT
    cout<<" ------------ Collecting remaining threads!"<<endl;
#endif

    // catch all remaining still running threads
    while(runningThreads.size()>0)
    {
      for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
      {
        assert(mThreads[*it]!=NULL);
        if(!mThreads[*it]->isRunning())
        {
          // collect results
          assert(*it<mAllRes.size());
          mAllRes[*it]=mThreads[*it]->getResults();
          delete mThreads[*it]; mThreads[*it]=NULL;
          it=runningThreads.erase(it);
#ifdef MULTITHREADING_OUTPUT
          cout<<"deleted thread"<<endl;
#endif
        }
      }
      usleep(1000);
      sprintf(buff,"%6.2lf",double(mThreads.size()-runningThreads.size())*100.0/double(mThreads.size()));
      cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
    }
    sprintf(buff,"%6.2lf",double(100.0));
    cout<<"\b\b\b\b\b\b"<<string(buff)<<endl;
  };

  const vector<Result>& getResults() const
  {
    return mAllRes;
  };

protected:

private:

  const uint32_t mMaxThreads;
  vector<SingleThread<Result>* >& mThreads;
  vector<Result> mAllRes;
};

// Specialization for SingleThread<void> which does not return a result
template<>
class MultiThreads<void>
{
public:
  MultiThreads(vector<SingleThread<void>* >& threads, uint32_t maxThreads)
  : mMaxThreads(maxThreads), mThreads(threads)
  {

  };
  ~MultiThreads()
  {};

  void work()
  {
    list<uint32_t> runningThreads;
    char buff[10];

    sprintf(buff,"%6.2lf",double(0.0));
    cout<<"MultiThreads::work: Progress: "<<string(buff);

    for(uint32_t tId=0; tId<mThreads.size(); ++tId)
    {
#ifdef MULTITHREADING_OUTPUT
    bool haveDisplayed=false;
#endif
      while(runningThreads.size()>=mMaxThreads)
      {
#ifdef MULTITHREADING_OUTPUT

        {// count number of running threads
          if(!haveDisplayed){
            cout<<"Threads running: ";
            for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
            {
              cout<<*it<<" ";
            }
            cout<<"=> "<<runningThreads.size()<<endl;
            haveDisplayed=true;
          }
        }
#endif
        for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
        {
          assert(mThreads[*it]!=NULL);
          if(!mThreads[*it]->isRunning())
          {
            delete mThreads[*it]; mThreads[*it]=NULL;
            it=runningThreads.erase(it);
#ifdef MULTITHREADING_OUTPUT
            cout<<"deleted thread"<<endl;
#endif
          }
        }
        usleep(1000);
      }
#ifdef MULTITHREADING_OUTPUT
      time_t rawtime; time ( &rawtime );
      cout<<" ------------ Starting thread "<<tId<<" @ "<<asctime(localtime(&rawtime)) <<" ------------ " <<endl;
#endif
      mThreads[tId]->run();
      //assert(mThreads[tId]->isRunning());
      runningThreads.push_back(tId);
#ifdef MULTITHREADING_OUTPUT
      cout<<" ------------ Threads running:" <<runningThreads.size()<<" finished+running: "<<mThreads.size() <<" allinall todo: "<<mThreads.size()<<endl;
#endif
      if(tId % uint32_t(ceil(double(mThreads.size())/10000.0)) == 0)
      {
        sprintf(buff,"%6.2lf",double(*(runningThreads.begin()))*100.0/double(mThreads.size()));
        cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
      }
    }

#ifdef MULTITHREADING_OUTPUT
    cout<<" ------------ Collecting remaining threads!"<<endl;
#endif

    // catch all remaining still running threads
    while(runningThreads.size()>0)
    {
//      cout<<"runningThreads.size()="<<runningThreads.size()<<endl;
//      for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
//      {
//        cout<<*it<<" -> "<<mThreads[*it]<<" "<<(mThreads[*it]->isRunning()?1:0)<<"; ";
//      }; cout<<endl;

      for(list<uint32_t>::iterator it=runningThreads.begin(); it!=runningThreads.end(); it++)
      {
        assert(mThreads[*it]!=NULL);
        if(!mThreads[*it]->isRunning())
        {
          delete mThreads[*it]; mThreads[*it]=NULL;
          it=runningThreads.erase(it);
#ifdef MULTITHREADING_OUTPUT
          cout<<"deleted thread"<<endl;
#endif
        }
      }
      usleep(1000);
      sprintf(buff,"%6.2lf",double(mThreads.size()-runningThreads.size())*100.0/double(mThreads.size()));
      cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
    }
    sprintf(buff,"%6.2lf",double(100.0));
    cout<<"\b\b\b\b\b\b"<<string(buff)<<endl;
  };

protected:

private:

  const uint32_t mMaxThreads;
  vector<SingleThread<void>* >& mThreads;
};

#endif /* MULTITHREADING_HPP_ */
