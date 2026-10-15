// World gravity expressed in the body frame: R_world_body^T * world_down.
// World yaw must not change this residual; no Euler decomposition is involved.
static double safeGravityDirectionError(const Transform & a, const Transform & b)
{
    const double ax=a.r31(),ay=a.r32(),az=a.r33();
    const double bx=b.r31(),by=b.r32(),bz=b.r33();
    const double norm=std::sqrt((ax*ax+ay*ay+az*az)*(bx*bx+by*by+bz*bz));
    if(!std::isfinite(norm)||norm<=1e-12)return std::numeric_limits<double>::infinity();
    return std::acos(std::max(-1.0,std::min(1.0,(ax*bx+ay*by+az*bz)/norm)));
}
