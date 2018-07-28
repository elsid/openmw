#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_SETTINGS_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_SETTINGS_H

#include <string>

namespace DetourNavigator
{
    struct Settings
    {
        bool mEnableWriteRecastMeshToFile;
        bool mEnableWriteNavMeshToFile;
        bool mEnableRecastMeshFileNameRevision;
        bool mEnableNavMeshFileNameRevision;
        float mCellHeight;
        float mCellSize;
        float mDetailSampleDist;
        float mDetailSampleMaxError;
        float mMaxAgentRadius;
        float mMaxClimb;
        float mMaxSimplificationError;
        float mMaxSlope;
        float mRecastScaleFactor;
        float mTileSize;
        int mBorderSize;
        int mMaxAgents;
        int mMaxEdgeLen;
        int mMaxNavMeshQueryNodes;
        int mMaxVertsPerPoly;
        int mRegionMergeSize;
        int mRegionMinSize;
        std::size_t mMaxPolygonPathSize;
        std::size_t mMaxSmoothPathSize;
        std::size_t mTrianglesPerChunk;
        std::string mRecastMeshPathPrefix;
        std::string mNavMeshPathPrefix;
    };
}

#endif
