//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_POINT3D_H
#define ROBO_PLANNER_WS_POINT3D_H

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <unordered_set>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/algorithm/clamp.hpp>

namespace utils
{
    class Point3D
    {

    public:
        Point3D() : data_{0.0, 0.0, 0.0} {}

        Point3D(const float &x, const float &y, const float &z) : data_{x, y, z} {}

        Point3D(const Eigen::Vector3d &point) : data_{point.x(), point.y(), point.z()} {}

        Point3D(const pcl::PointXYZ &point) : data_{point.x, point.y, point.z} {}

        Point3D(const Point3D &other) : data_{other.data_[0], other.data_[1], other.data_[2]} {}

        Point3D &operator=(Point3D const &other)
        {
            data_[0] = other.data_[0];
            data_[1] = other.data_[1];
            data_[2] = other.data_[2];
            return *this;
        }

        Point3D cross(Point3D const &other) const { return cross(*this, other); }

        static Point3D cross(Point3D const &first, Point3D const &second)
        {
            return Point3D(
                (first.data_[1] * second.data_[2]) - (first.data_[2] * second.data_[1]),
                (first.data_[2] * second.data_[0]) - (first.data_[0] * second.data_[2]),
                (first.data_[0] * second.data_[1]) - (first.data_[1] * second.data_[0]));
        }

        float dot(Point3D const &other) const { return dot(*this, other); }

        static float dot(Point3D const &first, Point3D const &second)
        {
            return (first.data_[0] * second.data_[0]) + (first.data_[1] * second.data_[1]) +
                   (first.data_[2] * second.data_[2]);
        }

        float &operator()(size_t idx) { return data_[idx]; }

        float const &operator()(size_t idx) const { return data_[idx]; }

        float &operator[](size_t idx) { return data_[idx]; }

        float const &operator[](size_t idx) const { return data_[idx]; }

        float &x() { return data_[0]; }

        float const &x() const { return data_[0]; }

        float &y() { return data_[1]; }

        float const &y() const { return data_[1]; }

        float &z() { return data_[2]; }

        float const &z() const { return data_[2]; }

        float &roll() { return data_[0]; }

        float const &roll() const { return data_[0]; }

        float &pitch() { return data_[1]; }

        float const &pitch() const { return data_[1]; }

        float &yaw() { return data_[2]; }

        float const &yaw() const { return data_[2]; }

        Point3D operator-() const { return Point3D(-data_[0], -data_[1], -data_[2]); }

        Point3D operator-(Point3D const &other) const
        {
            return Point3D(data_[0] - other.data_[0], data_[1] - other.data_[1],
                           data_[2] - other.data_[2]);
        }

        Point3D operator-(float value) const
        {
            return Point3D(data_[0] - value, data_[1] - value, data_[2] - value);
        }

        Point3D operator+(Point3D const &other) const
        {
            return Point3D(data_[0] + other.data_[0], data_[1] + other.data_[1],
                           data_[2] + other.data_[2]);
        }

        Point3D operator+(float value) const
        {
            return Point3D(data_[0] + value, data_[1] + value, data_[2] + value);
        }

        Point3D operator*(Point3D const &other) const
        {
            return Point3D(data_[0] * other.data_[0], data_[1] * other.data_[1],
                           data_[2] * other.data_[2]);
        }

        Point3D operator*(float value) const
        {
            return Point3D(data_[0] * value, data_[1] * value, data_[2] * value);
        }

        Point3D operator/(Point3D const &other) const
        {
            return Point3D(data_[0] / other.data_[0], data_[1] / other.data_[1],
                           data_[2] / other.data_[2]);
        }

        Point3D operator/(float value) const
        {
            return Point3D(data_[0] / value, data_[1] / value, data_[2] / value);
        }

        void operator-=(Point3D const &other)
        {
            data_[0] -= other.data_[0];
            data_[1] -= other.data_[1];
            data_[2] -= other.data_[2];
        }

        void operator+=(Point3D const &other)
        {
            data_[0] += other.data_[0];
            data_[1] += other.data_[1];
            data_[2] += other.data_[2];
        }

        void operator*=(Point3D const &other)
        {
            data_[0] *= other.data_[0];
            data_[1] *= other.data_[1];
            data_[2] *= other.data_[2];
        }

        void operator/=(Point3D const &other)
        {
            data_[0] /= other.data_[0];
            data_[1] /= other.data_[1];
            data_[2] /= other.data_[2];
        }

        void operator-=(float value)
        {
            data_[0] -= value;
            data_[1] -= value;
            data_[2] -= value;
        }

        void operator+=(float value)
        {
            data_[0] += value;
            data_[1] += value;
            data_[2] += value;
        }

        void operator*=(float value)
        {
            data_[0] *= value;
            data_[1] *= value;
            data_[2] *= value;
        }

        void operator/=(float value)
        {
            data_[0] /= value;
            data_[1] /= value;
            data_[2] /= value;
        }

        bool operator==(Point3D const &other) const
        {
            return data_[0] == other.data_[0] && data_[1] == other.data_[1] &&
                   data_[2] == other.data_[2];
        }

        bool operator!=(Point3D const &other) const
        {
            return data_[0] != other.data_[0] || data_[1] != other.data_[1] ||
                   data_[2] != other.data_[2];
        }

        float norm() const { return std::sqrt(squaredNorm()); }

        float squaredNorm() const
        {
            return (data_[0] * data_[0]) + (data_[1] * data_[1]) + (data_[2] * data_[2]);
        }

        Point3D &normalize()
        {
            *this /= norm();
            return *this;
        }

        Point3D normalized() const
        {
            Point3D temp(*this);
            return temp.normalize();
        }

        float angleTo(Point3D const &other) const
        {
            return std::acos(dot(other) / (norm() * other.norm()));
        }

        float distance(Point3D const &other) const
        {
            float x = data_[0] - other.data_[0];
            float y = data_[1] - other.data_[1];
            float z = data_[2] - other.data_[2];
            return sqrt((x * x) + (y * y) + (z * z));
        }

        float distanceXY(Point3D const &other) const
        {
            float x = data_[0] - other.data_[0];
            float y = data_[1] - other.data_[1];
            return sqrt((x * x) + (y * y));
        }

        size_t size() const { return 3; }

        float min() const { return std::min(std::min(data_[0], data_[1]), data_[2]); }

        float max() const { return std::max(std::max(data_[0], data_[1]), data_[2]); }

        size_t minElementIndex() const
        {
            if (data_[0] <= data_[1])
            {
                return data_[0] <= data_[2] ? 0 : 2;
            }
            else
            {
                return data_[1] <= data_[2] ? 1 : 2;
            }
        }

        size_t maxElementIndex() const
        {
            if (data_[0] >= data_[1])
            {
                return data_[0] >= data_[2] ? 0 : 2;
            }
            else
            {
                return data_[1] >= data_[2] ? 1 : 2;
            }
        }

        Point3D &ceil()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::ceil(data_[i]);
            }
            return *this;
        }

        Point3D ceil() const
        {
            return Point3D(std::ceil(data_[0]), std::ceil(data_[1]), std::ceil(data_[2]));
        }

        Point3D &floor()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::floor(data_[i]);
            }
            return *this;
        }

        Point3D floor() const
        {
            return Point3D(std::floor(data_[0]), std::floor(data_[1]), std::floor(data_[2]));
        }

        Point3D &trunc()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::trunc(data_[i]);
            }
            return *this;
        }

        Point3D trunc() const
        {
            return Point3D(std::trunc(data_[0]), std::trunc(data_[1]), std::trunc(data_[2]));
        }

        Point3D &round()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::round(data_[i]);
            }
            return *this;
        }

        Point3D round() const
        {
            return Point3D(std::round(data_[0]), std::round(data_[1]), std::round(data_[2]));
        }

        Point3D &clamp(Point3D const &min, Point3D const &max)
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = boost::algorithm::clamp(data_[i], min[i], max[i]);
            }
            return *this;
        }

        Point3D clamp(Point3D const &min, Point3D const &max) const
        {
            return clamp(*this, min, max);
        }

        static Point3D clamp(Point3D const &value, Point3D const &min, Point3D const &max)
        {
            return Point3D(boost::algorithm::clamp(value[0], min[0], max[0]),
                           boost::algorithm::clamp(value[1], min[1], max[1]),
                           boost::algorithm::clamp(value[2], min[2], max[2]));
        }

    protected:
        float data_[3];
    };

    struct Point3DHash
    {
        std::size_t operator()(const Point3D &key) const
        {
            int x_int = std::round(key.x() * 1000);
            int y_int = std::round(key.y() * 1000);
            int z_int = std::round(key.z() * 1000);
            std::string x = std::to_string((int)(x_int));
            std::string y = std::to_string((int)(y_int));
            std::string z = std::to_string((int)(z_int));

            using std::hash;
            using std::size_t;

            return ((hash<std::string>()(x) ^ (hash<std::string>()(y) << 1)) >> 1) ^ (hash<std::string>()(z) << 1);
        }
    };

    struct Equal
    {
        bool operator()(const Point3D &lhs, const Point3D &rhs) const
        {
            return std::round(lhs.x() * 1000) == std::round(rhs.x() * 1000) && std::round(lhs.y() * 1000) == std::round(rhs.y() * 1000) && std::round(lhs.z() * 1000) == std::round(rhs.z() * 1000);
        }
    };

    using Point3DSet = std::unordered_set<Point3D, Point3DHash, Equal>;
    template <typename T>
    using Point3DMap = std::unordered_map<Point3D, T, Point3DHash, Equal>;
    using Point3DQueue = std::vector<Point3D>;

}

#endif // ROBO_PLANNER_WS_POINT3D_H
