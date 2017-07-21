#ifndef GSLAM_LOOPDETECTOR_H
#define GSLAM_LOOPDETECTOR_H

#include "GSLAM.h"

namespace GSLAM {

// The LoopDetector is used to detect loops with Poses or Descriptors information
class LoopDetector: public GObject
{
public:
    struct LoopCandidate
    {
        FramePtr frame;
        double   score;
        friend bool operator<(const LoopCandidate& l,const LoopCandidate& r)
        {
            return l.score<r.score;
        }
    };
    typedef std::vector<LoopCandidate> LoopCandidates;

    virtual std::string type()const{return "LoopDetector";}
    virtual bool insertMapFrame(const FramePtr& frame){return false;}
    virtual bool eraseMapFrame(const FramePtr& frame){return false;}
    virtual bool obtainCandidates(const FramePtr& frame,LoopCandidates& candidates){return false;}
};

}

#endif
