#ifdef WIN32
# define WIN32_LEAN_AND_MEAN
# include <winsock2.h>
#endif // WIN32

#include <cstdio>
#include <cstdlib>
#include <string>
#include <stdexcept>
#include "logger.h"
#include "shmem.h"

using namespace std;

namespace Util
{
#ifdef WIN32
  Shmem::Shmem(const char* smName, unsigned long size)
  {
    strcpy(this->smName, smName);

    hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE,    // use paging file
                                 NULL,                    // default security
                                 PAGE_READWRITE,          // read/write access
                                 0,                       // maximum object size (high-order DWORD)
                                 size,                    // maximum object size (low-order DWORD)
                                 smName);                 // name of mapping object

    if (hMapFile == NULL)
    {
      fprintf(stderr, "Could not create file mapping object (%s).\n", GetLastError());
    }

    sprintf(spName, "%s__", smName);
    mutex = new Sem(spName, 1);
  }
#else
  Shmem::Shmem(key_t key, size_t size)
  {
    // Adding extra space for storing the semaphore
    if ((shmid = shmget(key, size + sizeof(sem_t), IPC_CREAT | SHM_R | SHM_W)) < 0)
    {
      Logger::abort("shmget failed: %s", strerror(errno));
    }

    pBuf = shmat(shmid, NULL, 0);
    if (pBuf == (void *) - 1)
    {
      Logger::abort("shmat failed: %s", strerror(errno));
    }

    sem = (sem_t *)((char *)pBuf + size);   // Semaphore is stored at the end of the shared memory block created

    mutex = new Sem(sem, 1);
  }
#endif // !WIN32

  Shmem::~Shmem()
  {
#ifdef WIN32
    UnmapViewOfFile(pBuf);

    if (!CloseHandle((HANDLE) hMapFile))
    {
      fprintf(stderr, "Shared Memory: failed to close.\n");
    }
#else
    if (shmdt(pBuf) != 0)
    {
      Logger::abort("shmdt failed: %s", strerror(errno));
    }
#endif // !WIN32
  }

  int Shmem::read(void *buffer, unsigned long size)
  {
    mutex->wait();
#ifdef WIN32
    hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS,     // read/write access
                               FALSE,                   // do not inherit the name
                               smName);                 // name of mapping object

    if (hMapFile == NULL)
    {
      cerr << "Could not open file mapping object (" << GetLastError() << ").\n";
      mutex->signal();
      return -1;
    }

    pBuf = (LPTSTR) MapViewOfFile(hMapFile,             // handle to map object
                                  FILE_MAP_ALL_ACCESS,  // read/write permission
                                  0,
                                  0,
                                  size);

    if (pBuf == NULL)
    {
      cerr << "Could not map view of file (" << GetLastError() << ").\n";
      CloseHandle(hMapFile);
      mutex->signal();
      return -1;
    }

    memcpy(buffer, pBuf, size);
    UnmapViewOfFile(pBuf);
#else
    memcpy(buffer, pBuf, size);
#endif // !WIN32
    mutex->signal();
    return 0;
  }

  int Shmem::write(const void *buffer, unsigned long size)
  {
    mutex->wait();
#ifdef WIN32
    pBuf = (LPTSTR) MapViewOfFile(hMapFile,             // handle to map object
                                  FILE_MAP_ALL_ACCESS,  // read/write permission
                                  0,
                                  0,
                                  size);

    if (pBuf == NULL)
    {
      cerr << "Could not map view of file (" << GetLastError() << ").\n";
      CloseHandle(hMapFile);
      mutex->signal();
      return -1;
    }

    CopyMemory((PVOID)pBuf, buffer, (size * sizeof(TCHAR)));
    UnmapViewOfFile(pBuf);
#else
    memcpy(pBuf, buffer, size);
#endif // !WIN32
    mutex->signal();
    return 0;
  }
} // namespace Util
