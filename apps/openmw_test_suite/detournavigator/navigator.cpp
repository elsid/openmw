#include <components/detournavigator/debug.hpp>
#include <components/detournavigator/navigator.hpp>
#include <components/detournavigator/exceptions.hpp>
#include <components/detournavigator/settingsutils.hpp>
#include <components/detournavigator/obstacleavoidancetype.hpp>
#include <components/detournavigator/queryfiltertype.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <DetourCrowd.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <array>

namespace
{
    using namespace testing;
    using namespace DetourNavigator;

    struct DetourNavigatorNavigatorTest : Test
    {
        Settings mSettings;
        std::unique_ptr<Navigator> mNavigator;
        osg::Vec3f mPlayerPosition;
        osg::Vec3f mAgentHalfExtents;
        osg::Vec3f mStart;
        osg::Vec3f mEnd;
        std::deque<osg::Vec3f> mPath;
        std::back_insert_iterator<std::deque<osg::Vec3f>> mOut;
        std::size_t mAgentId;
        dtCrowdAgentParams mAgentCrowdParams;

        DetourNavigatorNavigatorTest()
            : mPlayerPosition(0, 0, 0)
            , mAgentHalfExtents(29, 29, 66)
            , mStart(-215, 215, 1)
            , mEnd(215, -215, 1)
            , mOut(mPath)
            , mAgentId(0)
        {
            mSettings.mEnableWriteRecastMeshToFile = false;
            mSettings.mEnableWriteNavMeshToFile = false;
            mSettings.mEnableRecastMeshFileNameRevision = false;
            mSettings.mEnableNavMeshFileNameRevision = false;
            mSettings.mBorderSize = 16;
            mSettings.mCellHeight = 0.2f;
            mSettings.mCellSize = 0.2f;
            mSettings.mDetailSampleDist = 6;
            mSettings.mDetailSampleMaxError = 1;
            mSettings.mMaxAgentRadius = 1;
            mSettings.mMaxClimb = 34;
            mSettings.mMaxSimplificationError = 1.3f;
            mSettings.mMaxSlope = 49;
            mSettings.mRecastScaleFactor = 0.013f;
            mSettings.mTileSize = 64;
            mSettings.mMaxAgents = 128;
            mSettings.mMaxEdgeLen = 12;
            mSettings.mMaxNavMeshQueryNodes = 2048;
            mSettings.mMaxVertsPerPoly = 6;
            mSettings.mRegionMergeSize = 20;
            mSettings.mRegionMinSize = 8;
            mSettings.mMaxPolygonPathSize = 1024;
            mSettings.mMaxSmoothPathSize = 1024;
            mSettings.mTrianglesPerChunk = 256;

            mNavigator.reset(new Navigator(mSettings));

            mAgentCrowdParams.radius = getRadius(mAgentHalfExtents, mSettings);
            mAgentCrowdParams.height = getHeight(mAgentHalfExtents, mSettings);
            mAgentCrowdParams.maxSpeed = 2;
            mAgentCrowdParams.maxAcceleration = mAgentCrowdParams.maxSpeed;
            mAgentCrowdParams.collisionQueryRange = mAgentCrowdParams.radius * 50;
            mAgentCrowdParams.pathOptimizationRange = mAgentCrowdParams.radius * 100;
            mAgentCrowdParams.separationWeight = mAgentCrowdParams.radius * 2;
            mAgentCrowdParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS
                | DT_CROWD_OBSTACLE_AVOIDANCE
                | DT_CROWD_SEPARATION
                | DT_CROWD_OPTIMIZE_VIS
                | DT_CROWD_OPTIMIZE_TOPO;
            mAgentCrowdParams.obstacleAvoidanceType = ObstacleAvoidanceType_veryHigh;
            mAgentCrowdParams.queryFilterType = QueryFilterType_allFlags;
            mAgentCrowdParams.userData = nullptr;
        }
    };

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_empty_should_throw_exception)
    {
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut), InvalidArgument);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_existing_agent_with_no_navmesh_should_throw_exception)
    {
        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut), NavigatorException);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_removed_agent_should_throw_exception)
    {
        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->removeAgent(mAgentId, mAgentHalfExtents);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut), InvalidArgument);
    }

    TEST_F(DetourNavigatorNavigatorTest, add_agent_should_count_each_agent)
    {
        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->removeAgent(mAgentId, mAgentHalfExtents);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut), NavigatorException);
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_find_path_should_return_path)
    {
        const std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut);

        EXPECT_EQ(mPath, std::deque<osg::Vec3f>({
            osg::Vec3f(-215, 215, 7.69229984283447265625),
            osg::Vec3f(-187.8035888671875, 187.8035888671875, 7.692303180694580078125),
            osg::Vec3f(-160.607177734375, 160.607177734375, 7.692306995391845703125),
            osg::Vec3f(-133.4107666015625, 133.4107666015625, 2.239284515380859375),
            osg::Vec3f(-106.21435546875, 106.21435546875, -12.33022403717041015625),
            osg::Vec3f(-79.01793670654296875, 79.01793670654296875, -26.899723052978515625),
            osg::Vec3f(-51.821521759033203125, 51.821521759033203125, -41.46923065185546875),
            osg::Vec3f(-24.6251068115234375, 24.6251068115234375, -56.038753509521484375),
            osg::Vec3f(2.57130527496337890625, -2.57130527496337890625, -68.03694915771484375),
            osg::Vec3f(29.7677173614501953125, -29.7677173614501953125, -55.41004180908203125),
            osg::Vec3f(56.96413421630859375, -56.96413421630859375, -42.783130645751953125),
            osg::Vec3f(84.16054534912109375, -84.16054534912109375, -30.1562328338623046875),
            osg::Vec3f(111.35695648193359375, -111.35695648193359375, -13.0287628173828125),
            osg::Vec3f(138.553375244140625, -138.553375244140625, 4.45464611053466796875),
            osg::Vec3f(165.749786376953125, -165.749786376953125, 7.692306041717529296875),
            osg::Vec3f(192.9461822509765625, -192.9461822509765625, 7.692302703857421875),
            osg::Vec3f(215, -215, 7.69229984283447265625),
        }));
    }

    TEST_F(DetourNavigatorNavigatorTest, for_overlapping_heightfields_should_use_higher)
    {
        const std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        const std::array<btScalar, 5 * 5> heightfieldData2 {{
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
        }};
        btHeightfieldTerrainShape shape2(5, 5, heightfieldData2.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape2.setLocalScaling(btVector3(128, 128, 1));

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->addObject(2, shape2, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut);

        EXPECT_EQ(mPath, std::deque<osg::Vec3f>({
            osg::Vec3f(-215, 215, 7.637353420257568359375),
            osg::Vec3f(-187.8035888671875, 187.8035888671875, 3.7521531581878662109375),
            osg::Vec3f(-160.607177734375, 160.607177734375, -0.13304729759693145751953125),
            osg::Vec3f(-133.4107666015625, 133.4107666015625, -4.018249034881591796875),
            osg::Vec3f(-106.21435546875, 106.21435546875, -7.9034481048583984375),
            osg::Vec3f(-79.01793670654296875, 79.01793670654296875, -11.78864955902099609375),
            osg::Vec3f(-51.821521759033203125, 51.821521759033203125, -15.67385101318359375),
            osg::Vec3f(-24.6251068115234375, 24.6251068115234375, -19.559055328369140625),
            osg::Vec3f(2.57130527496337890625, -2.57130527496337890625, -22.7096004486083984375),
            osg::Vec3f(29.7677173614501953125, -29.7677173614501953125, -18.824398040771484375),
            osg::Vec3f(56.96413421630859375, -56.96413421630859375, -14.93919467926025390625),
            osg::Vec3f(84.16054534912109375, -84.16054534912109375, -11.05399417877197265625),
            osg::Vec3f(111.35695648193359375, -111.35695648193359375, -7.168792247772216796875),
            osg::Vec3f(138.553375244140625, -138.553375244140625, -3.2835910320281982421875),
            osg::Vec3f(165.749786376953125, -165.749786376953125, 0.601609706878662109375),
            osg::Vec3f(192.9461822509765625, -192.9461822509765625, 4.48680973052978515625),
            osg::Vec3f(215, -215, 7.637353420257568359375),
        }));
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_around_avoid_shape)
    {
        std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        std::array<btScalar, 5 * 5> heightfieldDataAvoid {{
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
            -25, -25, -25, -25, -25,
        }};
        btHeightfieldTerrainShape shapeAvoid(5, 5, heightfieldDataAvoid.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shapeAvoid.setLocalScaling(btVector3(128, 128, 1));

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, ObjectShapes {shape, &shapeAvoid}, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut);

        EXPECT_EQ(mPath, std::deque<osg::Vec3f>({
            osg::Vec3f(-215, 215, 7.6068267822265625),
            osg::Vec3f(-194.2162628173828125, 182.6375732421875, 2.9882237911224365234375),
            osg::Vec3f(-173.432525634765625, 150.275146484375, -1.63038408756256103515625),
            osg::Vec3f(-152.6487884521484375, 117.91272735595703125, -6.24899196624755859375),
            osg::Vec3f(-131.86505126953125, 85.55031585693359375, -10.86759853363037109375),
            osg::Vec3f(-111.08132171630859375, 53.187892913818359375, -15.4862060546875),
            osg::Vec3f(-90.29758453369140625, 20.8254795074462890625, -20.1048145294189453125),
            osg::Vec3f(-69.5138397216796875, -11.5369319915771484375, -22.1596622467041015625),
            osg::Vec3f(-41.49837493896484375, -37.888851165771484375, -22.529338836669921875),
            osg::Vec3f(-13.48290920257568359375, -64.24077606201171875, -22.8990535736083984375),
            osg::Vec3f(14.5325565338134765625, -90.59268951416015625, -20.039234161376953125),
            osg::Vec3f(47.2125396728515625, -110.87343597412109375, -15.532405853271484375),
            osg::Vec3f(79.89252471923828125, -131.1541748046875, -11.02557373046875),
            osg::Vec3f(112.57250213623046875, -151.4349212646484375, -6.51874065399169921875),
            osg::Vec3f(145.2524871826171875, -171.715667724609375, -2.011909961700439453125),
            osg::Vec3f(177.9324798583984375, -191.9964141845703125, 2.4949223995208740234375),
            osg::Vec3f(210.6124725341796875, -212.2771453857421875, 7.0017547607421875),
            osg::Vec3f(215, -215, 7.60683155059814453125),
        }));
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_water_when_use_swim_flag)
    {
        std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        const Water water(-25, 128 * 4);

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, ObjectShapes {shape, water}, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk | Flag_swim, mOut);

        EXPECT_EQ(mPath, std::deque<osg::Vec3f>({
            osg::Vec3f(-215, 215, 7.69229984283447265625),
            osg::Vec3f(-187.8035888671875, 187.8035888671875, 7.69229984283447265625),
            osg::Vec3f(-160.607177734375, 160.607177734375, 7.69229984283447265625),
            osg::Vec3f(-133.4107666015625, 133.4107666015625, 7.69229984283447265625),
            osg::Vec3f(-106.21435546875, 106.21435546875, 7.69229984283447265625),
            osg::Vec3f(-79.01793670654296875, 79.01793670654296875, 7.69229984283447265625),
            osg::Vec3f(-51.821521759033203125, 51.821521759033203125, 7.69229984283447265625),
            osg::Vec3f(-24.6251068115234375, 24.6251068115234375, 7.69229984283447265625),
            osg::Vec3f(2.57130527496337890625, -2.57130527496337890625, 7.69229984283447265625),
            osg::Vec3f(29.7677173614501953125, -29.7677173614501953125, 7.69229984283447265625),
            osg::Vec3f(56.96413421630859375, -56.96413421630859375, 7.69229984283447265625),
            osg::Vec3f(84.16054534912109375, -84.16054534912109375, 7.69229984283447265625),
            osg::Vec3f(111.35695648193359375, -111.35695648193359375, 7.69229984283447265625),
            osg::Vec3f(138.553375244140625, -138.553375244140625, 7.69229984283447265625),
            osg::Vec3f(165.749786376953125, -165.749786376953125, 7.69229984283447265625),
            osg::Vec3f(192.9461822509765625, -192.9461822509765625, 7.69229984283447265625),
            osg::Vec3f(215, -215, 7.69229984283447265625),
        }));
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_around_water_when_use_only_walk_flag)
    {
        std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        const Water water(-25, 128 * 4);

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, ObjectShapes {shape, water}, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut);

        EXPECT_EQ(mPath, std::deque<osg::Vec3f>({
            osg::Vec3f(-215, 215, 7.69229984283447265625),
            osg::Vec3f(-204.90869140625, 177.8859100341796875, 7.69229984283447265625),
            osg::Vec3f(-194.8173828125, 140.771820068359375, 7.69229984283447265625),
            osg::Vec3f(-184.7260589599609375, 103.65773773193359375, 7.69229984283447265625),
            osg::Vec3f(-174.634735107421875, 66.54364776611328125, 7.69229984283447265625),
            osg::Vec3f(-164.5434417724609375, 29.429561614990234375, 7.69229984283447265625),
            osg::Vec3f(-154.4521331787109375, -7.684524059295654296875, 7.69229984283447265625),
            osg::Vec3f(-144.3608245849609375, -44.798610687255859375, 7.69229984283447265625),
            osg::Vec3f(-134.269500732421875, -81.91269683837890625, 7.69229984283447265625),
            osg::Vec3f(-124.17818450927734375, -119.0267791748046875, 7.69229984283447265625),
            osg::Vec3f(-114.08687591552734375, -156.140869140625, 7.69229984283447265625),
            osg::Vec3f(-76.2261505126953125, -162.9124908447265625, 7.69229984283447265625),
            osg::Vec3f(-38.365413665771484375, -169.684112548828125, 7.69229984283447265625),
            osg::Vec3f(-0.504681646823883056640625, -176.4557342529296875, 7.69229984283447265625),
            osg::Vec3f(37.356048583984375, -183.2273406982421875, 7.69229984283447265625),
            osg::Vec3f(75.2167816162109375, -189.99896240234375, 7.69229984283447265625),
            osg::Vec3f(113.0775146484375, -196.77056884765625, 7.69229984283447265625),
            osg::Vec3f(150.9382476806640625, -203.5421905517578125, 7.69229984283447265625),
            osg::Vec3f(188.7989654541015625, -210.31378173828125, 7.69229984283447265625),
            osg::Vec3f(215, -215, 7.69229984283447265625),
        }));
    }

    TEST_F(DetourNavigatorNavigatorTest, set_agent_target_and_update_crowd_then_get_agent_target_should_return_next_path_point)
    {
        const std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, Flag_walk, mOut);

        mNavigator->updateAgentTarget(mAgentId, mPath[1], Flag_walk);
        mNavigator->updateCrowd(1.0f / 25.0f);

        const auto agentPosition = mNavigator->getAgentPosition(mAgentId);

        EXPECT_EQ(agentPosition, osg::Vec3f(-214.825927734375, 214.825927734375, 7.69229984283447265625));
    }

    TEST_F(DetourNavigatorNavigatorTest, set_agent_target_and_update_crowd_multiple_times_then_agent_should_reach_end)
    {
        const std::array<btScalar, 5 * 5> heightfieldData {{
            0,   0,    0,    0,    0,
            0, -25,  -25,  -25,  -25,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
            0, -25, -100, -100, -100,
        }};
        btHeightfieldTerrainShape shape(5, 5, heightfieldData.data(), 1, 0, 0, 2, PHY_FLOAT, false);
        shape.setLocalScaling(btVector3(128, 128, 1));

        mNavigator->addAgent(mAgentId, mStart, mAgentHalfExtents, mAgentCrowdParams);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->updateAgentTarget(mAgentId, mEnd, Flag_walk);

        auto agentPosition = mNavigator->getAgentPosition(mAgentId);

        for (int i = 0; i < 191; ++i)
        {
            mNavigator->updateCrowd(1.0f / 25.0f);
            agentPosition = mNavigator->getAgentPosition(mAgentId);
        }

        EXPECT_LE((agentPosition - mEnd).length(), 10.0f);
    }
}
