#include "../include/DBOW3_DeepBagofWords.hpp"

DBoW3::Vocabulary* DBOW3_GetVocabulary(void)
{
    static DBoW3::Vocabulary Vocabulary;

    static bool Initialized = false;

    if(!Initialized)
    {
        Vocabulary.load(std::string(PANTO_VocabFilePath));
        Initialized = true;
    }

    return &Vocabulary;
}

