    bool earlyGravityCertified = false;
    if(currentIds.size() < 200)
    {
        const char * earlySwitch = std::getenv("WBROV_PRIVATE_EARLY_GRAVITY_CERTIFICATE");
        wbrov::EarlyGravityCertificate certificate;
        certificate.nodes=currentIds.size();
        if(earlySwitch && std::string(earlySwitch)=="1")
        {
            double firstStamp=std::numeric_limits<double>::infinity(),lastStamp=0.0;
            for(int id : currentIds)
            {
                const Signature * signature=_memory->getSignature(id);
                if(!signature || !std::isfinite(signature->getStamp()) || signature->getStamp()<=0.0)
                { certificate.finite=false;continue; }
                firstStamp=std::min(firstStamp,signature->getStamp());
                lastStamp=std::max(lastStamp,signature->getStamp());
            }
            certificate.sourceSpan=lastStamp-firstStamp;
            std::set<int> measuredIds;
            double beforeSum=0.0;
            std::vector<double> alignedSums(anchorAlignments.size(),0.0);
            for(const auto & row : _constraints)
            {
                const Link & link=row.second;
                if(link.type()!=Link::kGravity || link.from()!=link.to() || !currentIds.count(link.from()))continue;
                if(!measuredIds.insert(link.from()).second) {certificate.finite=false;continue;}
                const Transform & pose=initialPoses.at(link.from());
                const double before=safeGravityDirectionError(pose,link.transform())*180.0/M_PI;
                certificate.finite=certificate.finite && std::isfinite(before);
                beforeSum+=before;certificate.beforeMaxDeg=std::max(certificate.beforeMaxDeg,before);
                for(unsigned int i=0;i<anchorAlignments.size();++i)
                {
                    const double after=safeGravityDirectionError(anchorAlignments[i]*pose,link.transform())*180.0/M_PI;
                    certificate.finite=certificate.finite && std::isfinite(after);
                    alignedSums[i]+=after;
                    certificate.alignedMaxDeg=std::max(certificate.alignedMaxDeg,after);
                    certificate.maxResidualIncreaseDeg=std::max(certificate.maxResidualIncreaseDeg,after-before);
                }
            }
            certificate.gravityNodes=measuredIds.size();
            if(certificate.gravityNodes)
            {
                certificate.beforeMeanDeg=beforeSum/certificate.gravityNodes;
                for(double sum : alignedSums)
                    certificate.alignedMeanMaxDeg=std::max(certificate.alignedMeanMaxDeg,sum/certificate.gravityNodes);
            }
            std::string certificateReason;
            earlyGravityCertified=certificate.eligible(certificateReason);
            UWARN("private early certificate eligible=%d nodes=%u gravity=%u span=%.6fs mean=%.6f max=%.6f aligned_mean=%.6f aligned_max=%.6f increase=%.6f reason=%s",
                earlyGravityCertified?1:0,(unsigned int)certificate.nodes,(unsigned int)certificate.gravityNodes,certificate.sourceSpan,
                certificate.beforeMeanDeg,certificate.beforeMaxDeg,certificate.alignedMeanMaxDeg,certificate.alignedMaxDeg,
                certificate.maxResidualIncreaseDeg,certificateReason.c_str());
        }
        if(!earlyGravityCertified)
        {
            reason=uFormat("current reset component needs gravity stabilization (%u/200 nodes)",(unsigned int)currentIds.size());
            return false;
        }
    }
