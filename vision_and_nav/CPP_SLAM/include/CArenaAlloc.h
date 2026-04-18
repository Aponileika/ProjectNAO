#ifndef __CARENAALLOC_H_
#define __CARENAALLOC_H_
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <sys/mman.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float fp32;
typedef double fp64;

struct Arena
{
    u64 pos;
    u64 cap;
};

typedef struct Arena* Arenamem;

#define KiB (1 << 10)
#define MiB (1 << 20)
#define GiB (1 << 30)

Arenamem AR_InitArena(u64 sz);
void AR_DestroyArena(Arenamem arena);

void* AR_ArenaPush(Arenamem arena, u64 sz);

#define AR_ArenaPushArr(arena, type, cnt) (AR_ArenaPush(arena, sizeof(type)*cnt))
#define AR_ArenaPushStruct(arena, type) (AR_ArenaPushArr(arena, type, 1))

i32 AR_ArenaPop(Arenamem arena, u64 sz);
#define AR_ArenaPopArr(arena, type, cnt) (AR_ArenaPop(arena, sizeof(type)*cnt))
#define AR_ArenaPopStruct(arena, type) (AR_ArenaPopArr(arena, type, 1))

#endif //__CARENAALLOC_H_
