#if defined _WIN32
#ifdef API_EXPORTS
#define DVO_EXPORTS __declspec(dllexport)
#else
#define DVO_EXPORTS 
#endif
#else
#define DVO_EXPORTS
#endif