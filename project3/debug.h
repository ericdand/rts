#ifndef NDEBUG
#define DEBUG(M) write_trace(M, __FILE__, __LINE__)
void debug_init();
void write_trace(char *M, char *file, int line);
#else
#define DEBUG(M)  
#endif
		
