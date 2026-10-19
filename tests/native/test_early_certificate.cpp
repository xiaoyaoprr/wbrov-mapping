#include "EarlyGravityCertificate.h"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
    wbrov::EarlyGravityCertificate base;
    base.nodes=17;base.gravityNodes=17;base.sourceSpan=16;
    base.beforeMeanDeg=.5;base.beforeMaxDeg=1.5;base.alignedMeanMaxDeg=.7;
    base.alignedMaxDeg=1.8;base.maxResidualIncreaseDeg=.3;
    std::string reason;assert(base.eligible(reason));
    auto v=base;v.sourceSpan=14.999;assert(!v.eligible(reason));
    v=base;v.nodes=9;assert(!v.eligible(reason));
    v=base;v.gravityNodes=15;assert(!v.eligible(reason));
    v=base;v.gravityNodes=16;assert(v.eligible(reason));
    v=base;v.beforeMeanDeg=2.001;assert(!v.eligible(reason));
    v=base;v.beforeMaxDeg=5.001;assert(!v.eligible(reason));
    v=base;v.alignedMeanMaxDeg=2.001;assert(!v.eligible(reason));
    v=base;v.alignedMaxDeg=5.001;assert(!v.eligible(reason));
    v=base;v.maxResidualIncreaseDeg=.501;assert(!v.eligible(reason));
    v=base;v.finite=false;assert(!v.eligible(reason));
    v=base;v.sourceSpan=std::numeric_limits<double>::infinity();assert(!v.eligible(reason));
    v=base;v.beforeMeanDeg=std::numeric_limits<double>::quiet_NaN();assert(!v.eligible(reason));
    v=base;v.nodes=200;v.gravityNodes=200;assert(!v.eligible(reason));
    v=base;v.nodes=3;v.gravityNodes=3;v.sourceSpan=.2;
    assert(v.eligible(reason,true));assert(!v.eligible(reason));
    v.gravityNodes=2;assert(!v.eligible(reason,true));
    v.gravityNodes=3;v.sourceSpan=.199;assert(!v.eligible(reason,true));
    v.sourceSpan=.2;v.maxResidualIncreaseDeg=.501;assert(!v.eligible(reason,true));
    v=base;v.nodes=33;v.gravityNodes=33;assert(!v.eligible(reason,true));
    std::cout << "PASS legacy and observed-reanchor certificate predicates; this is not a graph-merge test\n";
}
