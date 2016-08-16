#ifndef ORB_VOCABULARY_H
#define ORB_VOCABULARY_H

#include "TemplatedVocabulary.h"
#include "FORB.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor,DBoW2::FORB> ORB_Vocabulary;
typedef DBoW2::FORB::TDescriptor ORB_Descriptor;
#endif // ORB_VOCABULARY_H
