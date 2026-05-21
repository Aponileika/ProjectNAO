#include "ARGS_ParseArgs.hpp"

i32  __ARGS_ParseGroup(FILE* fp);
i32  __ARGS_ParseGroupF(FILE* fp);
i32  __ARGS_ParseDataset(FILE* fp);
i32  __ARGS_ParseRANSAC_E(FILE* fp);
i32  __ARGS_ParseRANSAC_PNP(FILE* fp);
i32  __ARGS_ParseBA(FILE* fp);

#define GROUP_STR "GROUP"
#define GROUPF_STR "GROUPF"
#define DATASET_STR "DATASET"
#define END_STR "END"

void ARGS_ParseArgs()
{
    FILE* fp = fopen(PATH_TO_ARGS, "r");
    if(fp == NULL)
    {
        perror("[ARGS_PARSEARGS] Failed to open arg file, check path PATH_TO_ARGS in ARGS_ParseArgs.hpp");
        return;
    }
    size_t line_max = static_cast<size_t>(LINE_MAX);
    char* line = NULL;
    ssize_t num_read_c;
    while((num_read_c = getline(&line, &line_max, fp)) != EOF)
    {
        std::string curr_line(line);
        i32 res = strcmp(line, GROUP_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseGroup(fp);
            if(res_parse == EOF)break;
        }

        res = strcmp(line, GROUPF_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseGroupF(fp);
            if(res_parse == EOF)break;
        }

        res = strcmp(line, GROUPF_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseGroupF(fp);
            if(res_parse == EOF)break;
        }

        res = strcmp(line, DATASET_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseGroupF(fp);
            if(res_parse == EOF)break;
        }
    }
    free(line);
    fclose(fp);
}

//this is dumb but who cares really
#define PARAM_STR "PARAM"
#define RANSAC_E_STR "RANSAC_E_STR"
#define RANSAC_PNP_STR "RANSAC_PNP_STR"
#define BA_STR "BA_STR"

i32 __ARGS_ParseGroup(FILE* fp)
{
    size_t line_max = static_cast<size_t>(LINE_MAX);
    char* line = NULL;
    ssize_t num_read_c;
    while((num_read_c = getline(&line, &line_max, fp)) != EOF)
    {
        std::string curr_line(line);
        i32 res = strcmp(line, RANSAC_E_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseRANSAC_E(fp);
            if(res_parse == EOF)return EOF;
        }

        res = strcmp(line, RANSAC_PNP_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseRANSAC_PNP(fp);
            if(res_parse == EOF)return EOF;
        }

        res = strcmp(line, BA_STR); 
        if(res == 0)
        {
            i32 res_parse = __ARGS_ParseBA(fp);
            if(res_parse == EOF)return EOF;
        }
    }
    free(line);
    return 1;
}

#define RANSACMETHOD_STR "RANSACMETHOD_STR"
#define PROBECORRECT_STR "PROBECORRECT_STR"
#define RANSACEPIXELT_STR "RANSACEPIXELT_STR"
#define RANSACMAXITERS_STR "RANSACMAXITERS_STR"

typedef enum
{
    METHOD = 0,
    PROBECORRECT = 1,
    PIXELT = 2,
    MAXITER = 3,
}RANSACE_PARAMS;

#define NUM_RANSACE_PARAMS 4

const char* RANSACE_params[NUM_RANSACE_PARAMS] =
{RANSACMETHOD_STR, PROBECORRECT_STR, RANSACEPIXELT_STR, RANSACMAXITERS_STR};

i32 __ARGS_ParseRANSAC_E(FILE* fp)
{
    //Set params...
    size_t line_max = static_cast<size_t>(LINE_MAX);
    char* line = NULL;
    ssize_t num_read_c = getline(&line, &line_max, fp);
    if(num_read_c == EOF)return EOF;
    while(strcmp(line, END_STR) != 0)
    {
        num_read_c = getline(&line, &line_max, fp);
        if(num_read_c == EOF)return EOF;

        i32 res = strcmp(line, END_STR);
        if(res == 0)return 1;

        res = strcmp(line, PARAM_STR);
        if(res == 0)
        {
            //get param type, and value
            num_read_c = getline(&line, &line_max, fp);
            if(num_read_c == EOF)return EOF;
            for(i32 i = 0; i < NUM_RANSACE_PARAMS; i++)
            {
                res = strcmp(line, RANSACE_params[i]);
                if(res == 0)
                {
                }
            }
        }

    }
}
