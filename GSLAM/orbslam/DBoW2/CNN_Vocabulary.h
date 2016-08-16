#ifndef CNN_VOCABULARY_H
#define CNN_VOCABULARY_H
#include "TemplatedVocabulary.h"
#include "FCNN.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FCNN::TDescriptor,DBoW2::FCNN> CNN_Vocabulary;
typedef DBoW2::FCNN::TDescriptor CNN_Descriptor;

#endif // CNN_VOCABULARY_H
