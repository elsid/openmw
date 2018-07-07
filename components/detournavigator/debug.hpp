#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_DEBUG_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_DEBUG_H

#include <DetourStatus.h>

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

#include <osg/Vec2i>
#include <osg/Vec3f>
#include <osg/Vec2i>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

class dtNavMesh;

namespace DetourNavigator
{
    class RecastMesh;

    inline std::ostream& operator <<(std::ostream& stream, const osg::Vec2i& value)
    {
        return stream << '(' << value.x() << ", " << value.y() << ')';
    }

    inline std::ostream& operator <<(std::ostream& stream, const osg::Vec2f& value)
    {
        return stream << '(' << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.x()
                      << ", " << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.y()
                      << ')';
    }

    inline std::ostream& operator <<(std::ostream& stream, const osg::Vec3f& value)
    {
        return stream << '(' << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.x()
                      << ", " << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.y()
                      << ", " << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.z()
                      << ')';
    }

    inline std::ostream& operator <<(std::ostream& stream, const btVector3& value)
    {
        return stream << '(' << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.x()
                      << ", " << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.y()
                      << ", " << std::setprecision(std::numeric_limits<float>::max_exponent10) << value.z()
                      << ')';
    }

    inline std::ostream& operator <<(std::ostream& stream, BroadphaseNativeTypes value)
    {
        switch (value)
        {
#ifndef SHAPE_NAME
#define SHAPE_NAME(name) case name: return stream << #name;
            SHAPE_NAME(BOX_SHAPE_PROXYTYPE)
            SHAPE_NAME(TRIANGLE_SHAPE_PROXYTYPE)
            SHAPE_NAME(TETRAHEDRAL_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONVEX_HULL_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE)
            SHAPE_NAME(CUSTOM_POLYHEDRAL_SHAPE_TYPE)
            SHAPE_NAME(IMPLICIT_CONVEX_SHAPES_START_HERE)
            SHAPE_NAME(SPHERE_SHAPE_PROXYTYPE)
            SHAPE_NAME(MULTI_SPHERE_SHAPE_PROXYTYPE)
            SHAPE_NAME(CAPSULE_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONE_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONVEX_SHAPE_PROXYTYPE)
            SHAPE_NAME(CYLINDER_SHAPE_PROXYTYPE)
            SHAPE_NAME(UNIFORM_SCALING_SHAPE_PROXYTYPE)
            SHAPE_NAME(MINKOWSKI_SUM_SHAPE_PROXYTYPE)
            SHAPE_NAME(MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE)
            SHAPE_NAME(BOX_2D_SHAPE_PROXYTYPE)
            SHAPE_NAME(CONVEX_2D_SHAPE_PROXYTYPE)
            SHAPE_NAME(CUSTOM_CONVEX_SHAPE_TYPE)
            SHAPE_NAME(CONCAVE_SHAPES_START_HERE)
            SHAPE_NAME(TRIANGLE_MESH_SHAPE_PROXYTYPE)
            SHAPE_NAME(SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
            SHAPE_NAME(FAST_CONCAVE_MESH_PROXYTYPE)
            SHAPE_NAME(TERRAIN_SHAPE_PROXYTYPE)
            SHAPE_NAME(GIMPACT_SHAPE_PROXYTYPE)
            SHAPE_NAME(MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE)
            SHAPE_NAME(EMPTY_SHAPE_PROXYTYPE)
            SHAPE_NAME(STATIC_PLANE_PROXYTYPE)
            SHAPE_NAME(CUSTOM_CONCAVE_SHAPE_TYPE)
            SHAPE_NAME(CONCAVE_SHAPES_END_HERE)
            SHAPE_NAME(COMPOUND_SHAPE_PROXYTYPE)
            SHAPE_NAME(SOFTBODY_SHAPE_PROXYTYPE)
            SHAPE_NAME(HFFLUID_SHAPE_PROXYTYPE)
            SHAPE_NAME(HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE)
            SHAPE_NAME(INVALID_SHAPE_PROXYTYPE)
            SHAPE_NAME(MAX_BROADPHASE_COLLISION_TYPES)
#undef SHAPE_NAME
#endif
            default:
                return stream << "undefined(" << int(value) << ")";
        }
    }

    inline std::ostream& operator <<(std::ostream& stream, const std::chrono::steady_clock::time_point& value)
    {
        using float_s = std::chrono::duration<float, std::ratio<1>>;
        return stream << std::fixed << std::setprecision(4)
                      << std::chrono::duration_cast<float_s>(value.time_since_epoch()).count();
    }

    struct Sink
    {
        virtual ~Sink() = default;
        virtual void write(const std::string& text) = 0;
    };

    class FileSink final : public Sink
    {
    public:
        FileSink(std::string path)
            : mPath(std::move(path))
        {
            mFile.exceptions(std::ios::failbit | std::ios::badbit);
        }

        void write(const std::string& text) override
        {
            if (!mFile.is_open())
            {
                mFile.open(mPath);
            }
            mFile << text << std::flush;
        }

    private:
        std::string mPath;
        std::ofstream mFile;
    };

    class StdoutSink final : public Sink
    {
    public:
        void write(const std::string& text) override
        {
            std::cout << text << std::flush;
        }
    };

    class Log
    {
    public:
        void setSink(std::unique_ptr<Sink> sink)
        {
            const std::lock_guard<std::mutex> guard(mMutex);
            mSink = std::move(sink);
        }

        bool isEnabled() const
        {
            const std::lock_guard<std::mutex> guard(mMutex);
            return bool(mSink);
        }

        void write(const std::string& text)
        {
            const std::lock_guard<std::mutex> guard(mMutex);
            if (mSink)
                mSink->write(text);
        }

        static Log& instance()
        {
            static Log value;
            return value;
        }

    private:
        mutable std::mutex mMutex;
        std::unique_ptr<Sink> mSink;
    };

    inline void write(std::ostream& stream)
    {
        stream << '\n';
    }

    template <class Head, class ... Tail>
    void write(std::ostream& stream, const Head& head, const Tail& ... tail)
    {
        stream << head;
        write(stream, tail ...);
    }

    template <class ... Ts>
    void log(Ts&& ... values)
    {
        auto& log = Log::instance();
        if (!log.isEnabled())
            return;
        std::ostringstream stream;
        stream << '[' << std::chrono::steady_clock::now() << "] ";
        write(stream, std::forward<Ts>(values) ...);
        log.write(stream.str());
    }

    void writeToFile(const RecastMesh& recastMesh, const std::string& pathPrefix, const std::string& revision);
    void writeToFile(const dtNavMesh& navMesh, const std::string& pathPrefix, const std::string& revision);
}

#endif
