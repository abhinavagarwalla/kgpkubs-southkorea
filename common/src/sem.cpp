#ifdef WIN32
# define WIN32_LEAN_AND_MEAN
# include <winsock2.h>
#endif

#ifndef _STDEXCEPT_
# include <stdexcept>
#endif

#include <errno.h>
#include <cstdio>
#include <cstring>
#include "sem.h"
#include "logger.h"

#define NOT_USED(x)      ((void)(x))
 
using namespace std;

namespace Util
{
#ifdef WIN32
  Sem::Sem(const char* const   inName,
                       const unsigned long inCount,
                       const unsigned long inMaxCount)
    : theSemaphoreHandle(0),
      theMaxCount(inMaxCount)
  {
    if (theMaxCount == 0 || theMaxCount > 0x7FFFFFFF)
    {
      theMaxCount = 0x7FFFFFFF;
    }

    theSemaphoreHandle = CreateSemaphore(NULL,
                                         inCount > theMaxCount ? theMaxCount : inCount,
                                         theMaxCount,
                                         inName);

    if (theSemaphoreHandle == NULL)
    {
      DWORD rc = GetLastError();

      if (rc == ERROR_ALREADY_EXISTS)
      {
        // semaphore is creted try to open it
        theSemaphoreHandle = OpenSemaphore(SEMAPHORE_ALL_ACCESS | SYNCHRONIZE, FALSE, inName);

        if (theSemaphoreHandle == 0)
        {
          Logger::abort("Semaphore: failed to open.");
        }
      }
      else
      {
        Logger::abort("Semaphore: failed to create.");
      }
    }
  }
#else
  Sem::Sem(sem_t* sem, unsigned int inInitialCount) :
    sem(sem)
  {
    if (sem_init(sem, 1, inInitialCount) != 0)
    {
      Logger::abort("Could not create semaphore: %s", strerror(errno));
    }
  }
#endif // !WIN32

  Sem::~Sem ()
  {
#ifdef WIN32

    if (!CloseHandle(theSemaphoreHandle))
    {
      fprintf(stderr, "Semaphore: failed to close.");
    }

#else
    if (sem_destroy(sem) != 0)
    {
      Logger::abort("Could not destroy semaphore: %s", strerror(errno));
    }
    #endif // !WIN32
  }

  void Sem::signal(const unsigned long inPostCount)
  {
#ifdef WIN32
    unsigned long postCount = inPostCount ? inPostCount : 1;

    if (!ReleaseSemaphore(theSemaphoreHandle, postCount, NULL))
    {
      unsigned long rc = GetLastError();

      if (rc != ERROR_TOO_MANY_POSTS)
      {
        fprintf(stderr, "Semaphore: failed to post.");
      }
    }

#else
    NOT_USED(inPostCount);
    if (sem_post(sem) != 0)
    {
      Logger::abort("Semaphore Post failed: %s", strerror(errno));
    }
#endif // !WIN32
  }

  void Sem::wait(const unsigned long inTimeout)
  {
#ifdef WIN32
    DWORD aTimeout = inTimeout == 0 ? INFINITE : inTimeout;
    DWORD rc = WaitForSingleObject(theSemaphoreHandle, aTimeout);

    if (rc == WAIT_FAILED || (inTimeout == 0 && rc == WAIT_TIMEOUT))
    {
      fprintf(stderr, "Semaphore: wait failed.");
    }

#else
    NOT_USED(inTimeout);
    if (sem_wait(sem) != 0)
    {
      Logger::abort("Semaphore wait failed: %s", strerror(errno));
    }
#endif // !WIN32
  }
} // namespace Util
