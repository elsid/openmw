#include <components/detournavigator/navigator.hpp>
#include <components/detournavigator/exceptions.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

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

        DetourNavigatorNavigatorTest()
            : mPlayerPosition(0, 0, 0)
            , mAgentHalfExtents(29, 29, 66)
            , mStart(-215, 215, 1)
            , mEnd(215, -215, 1)
            , mOut(mPath)
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
            mSettings.mMaxClimb = 34;
            mSettings.mMaxSimplificationError = 1.3f;
            mSettings.mMaxSlope = 49;
            mSettings.mRecastScaleFactor = 0.013f;
            mSettings.mTileSize = 64;
            mSettings.mMaxEdgeLen = 12;
            mSettings.mMaxNavMeshQueryNodes = 2048;
            mSettings.mMaxVertsPerPoly = 6;
            mSettings.mRegionMergeSize = 20;
            mSettings.mRegionMinSize = 8;
            mSettings.mMaxPolygonPathSize = 1024;
            mSettings.mMaxSmoothPathSize = 1024;
            mSettings.mTrianglesPerChunk = 256;
            mNavigator.reset(new Navigator(mSettings));
        }
    };

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_empty_should_throw_exception)
    {
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut), InvalidArgument);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_existing_agent_with_no_navmesh_should_throw_exception)
    {
        mNavigator->addAgent(mAgentHalfExtents);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut), NavigatorException);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_removed_agent_should_throw_exception)
    {
        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->removeAgent(mAgentHalfExtents);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut), InvalidArgument);
    }

    TEST_F(DetourNavigatorNavigatorTest, add_agent_should_count_each_agent)
    {
        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->removeAgent(mAgentHalfExtents);
        EXPECT_THROW(mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut), NavigatorException);
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

        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut);

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

        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->addObject(1, shape, btTransform::getIdentity());
        mNavigator->addObject(2, shape2, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut);

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

        mNavigator->addAgent(mAgentHalfExtents);
        mNavigator->addObject(1, ObjectShapes {shape, &shapeAvoid}, btTransform::getIdentity());
        mNavigator->update(mPlayerPosition);
        mNavigator->wait();

        mNavigator->findPath(mAgentHalfExtents, mStart, mEnd, mOut);

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
}
