
#include <time.h>
#include <px4_log.h>
// #include <qurt.h>

void *voxl_malloc( unsigned int size);
void voxl_free( void *ptr);

__attribute__((visibility("default"))) void free(void *ptr)
{
	voxl_free(ptr);
	ptr = 0;
}

__attribute__((visibility("default"))) void *malloc(size_t size)
{
	return voxl_malloc(size);
}

__attribute__((visibility("default"))) void *calloc(size_t nmemb, size_t size)
{
	PX4_ERR("Undefined calloc called");
	return (void *) 0;
}

__attribute__((visibility("default"))) void *realloc(void *ptr, size_t size)
{
	PX4_ERR("Undefined realloc called");
	return (void *) 0;
}

__attribute__((visibility("default"))) int nanosleep(const struct timespec *req, struct timespec *rem)
{
	PX4_ERR("Undefined nanosleep called");
	return -1;
}

// These are all undefined in new SDK so they need to be defined here...
// TODO: Why? What are these used for?
int sys_close(unsigned int fd) { return -1; };
int sys_write(unsigned int fd, const char *buf, size_t count) { return -1; }
int sys_remove(const char* filename) { return -1; }
void _exit(int code) { while(1); }
int sys_Mtxdst() { return -1; }
