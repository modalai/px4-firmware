
#include <time.h>

void qurt_free(void *ptr);

__attribute__((visibility("default"))) void free(void *ptr)
{
	qurt_free(ptr);
	// ptr = 0;
}

__attribute__((visibility("default"))) void *malloc(size_t size)
{
	return (void *) 0;
}

__attribute__((visibility("default"))) void *calloc(size_t nmemb, size_t size)
{
	return (void *) 0;
}

__attribute__((visibility("default"))) void *realloc(void *ptr, size_t size)
{
	return (void *) 0;
}

__attribute__((visibility("default"))) int nanosleep(const struct timespec *req, struct timespec *rem)
{
	return -1;
}

uint32_t crc32part(const uint8_t *name, uint32_t len, uint32_t hash) { return 0; }
