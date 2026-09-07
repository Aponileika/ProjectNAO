#include "../include/DBOW3_DeepBagofWords.hpp"

const DBoW3::Vocabulary* DBOW3_GetVocabulary(void)
{
    static DBoW3::Vocabulary Vocabulary;
    // Function-local static initialization is synchronized by C++. Unlike a
    // separate mutable bool, this guard cannot let another thread observe the
    // vocabulary while load() is still modifying it.
    static const bool Initialized = []()
    {
        Vocabulary.load(std::string(PANTO_VocabFilePath));
        return true;
    }();
    (void) Initialized;

    return &Vocabulary;
}
