#include "../include/CArenaAlloc.h"

#ifdef  _WIN32
    //for now idgaf about windows, so just let the malloc be malloc
    //Also the API for virtual memory allocation is different on windows...
    #define __ARENA_MALLOC(sz) malloc(sz)
    #define __ARENA_FREE(ptr) free(ptr)
    #define __ARENA_MALLOCRetIsValid(ret) (ret == NULL) 
#else
    #define __ARENA_MALLOC(sz) mmap(NULL, sz, PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0)
    #define __ARENA_FREE(ptr) munmap(ptr, ptr->cap + sizeof(struct Arena))
    #define __ARENA_MALLOCRetIsValid(ret) (!(ret == MAP_FAILED))
#endif

Arenamem AR_InitArena(u64 sz)
{
    Arenamem arena = (Arenamem)__ARENA_MALLOC(sz + sizeof(struct Arena));
    if(!(__ARENA_MALLOCRetIsValid(arena)))
    {
        return NULL;
    }
    arena->cap = sz;
    arena->pos = 0;
    return arena;
}

void AR_DestroyArena(Arenamem arena)
{
    __ARENA_FREE(arena);
}

void* AR_ArenaPush(Arenamem arena, u64 sz)
{
    if((arena->pos + sz) > arena->cap){
        return NULL;
    }
    u8* base = (u8*) arena + sizeof(struct Arena);
    void* pos = arena->pos + base;
    arena->pos += sz;
    return pos;
}

//returns 0 if trying to pop empty stack
i32 AR_ArenaPop(Arenamem arena, u64 sz)
{
    if(sz > arena->pos) return 0;
    arena->pos -= sz;
    return 1;
}


