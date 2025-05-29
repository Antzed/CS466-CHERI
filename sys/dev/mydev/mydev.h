#ifndef _MYDEV_H_
#define _MYDEV_H_

#include <sys/types.h>
#include <sys/ioccom.h>

typedef struct {
	int x;
	int y;
} bar_t;

typedef struct test_struct {
	int * __kerncap ptr_test;
} test_struct_t;

#define MYDEVIOC_READ	_IOR('a', 1, bar_t)
#define MYDEVIOC_WRITE	_IOW('a', 2, bar_t)
#define MYDEVIOC_RDWR	_IOWR('a', 3, bar_t)
#define MYDEVIOC_TEST	_IOWR('a', 4, test_struct_t)

#endif /* _MYDEV_H_ */
