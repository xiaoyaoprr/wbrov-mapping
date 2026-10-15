        // An early graph is accepted only under the ORIGINAL strict rigidity
        // limits: it never receives the mature distributed-gravity exception.
        // Check absolute residuals after every independent shadow solve too.
        if(earlyGravityCertified)
        {
            double maxEarlyGravity=0.0;
            for(const auto & row : connectedConstraints)
            {
                const Link & link=row.second;
                if(link.type()==Link::kGravity && link.from()==link.to() && currentIds.count(link.from()))
                {
                    const double residual=safeGravityDirectionError(optimized.at(link.from()),link.transform())*180.0/M_PI;
                    if(!std::isfinite(residual)) {reason="early shadow has non-finite gravity";return false;}
                    maxEarlyGravity=std::max(maxEarlyGravity,residual);
                }
            }
            if(gravityResidualCount*10<currentIds.size()*9 || !gravityResidualCount ||
               gravityResidualAfterSumDeg/gravityResidualCount>2.0 || maxEarlyGravity>5.0 ||
               maxGravityResidualIncreaseDeg>std::min(double(limits.maxGravityResidualIncrease),0.5))
            {reason="early shadow failed absolute measured-gravity certificate";return false;}
        }
