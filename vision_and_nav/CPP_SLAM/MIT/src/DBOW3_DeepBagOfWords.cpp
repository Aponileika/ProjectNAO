#include "../include/DBOW3_DeepBagOfWords.hpp"

static DBoW3::Vocabulary Vocabulary{};
static bool VocabIsInit = false;

void DBOW3_InitVocabulary(void)
{
    Vocabulary.load(PANTO_VocabFilePath);
    VocabIsInit = true;
}

DBoW3::Vocabulary DBOW3_GetVocabulary(void)
{
    if(!VocabIsInit)
    {
        DBOW3_InitVocabulary();
    }
    return Vocabulary;
}
