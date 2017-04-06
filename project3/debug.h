#ifndef NDEBUG
#define DEBUG(M) write_trace(M, __FILE__, __LINE__)
#define DEBUG_VALUE(N) dbg_print_value(N, __FILE__, __LINE__)
void debug_init();
void write_trace(char *M, char *file, int line);
void dbg_print_value(uint16_t, char*, int);
#else
#define DEBUG(M)  
#define DEBUG_VALUE(N)
#endif
		
