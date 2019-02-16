#ifndef DMA_UTILS_H
#define DMA_UTILS_H

extern int verbose;
uint64_t getopt_integer(char *optarg);
ssize_t read_to_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base);
ssize_t write_from_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base);
void timespec_sub(struct timespec *t1, struct timespec *t2);

#endif  // DMA_UTILS_H
