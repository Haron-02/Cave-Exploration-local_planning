//
// Created by hjl on 2021/11/13.
//

#ifndef TOPO_PLANNER_WS_GRID_MAP_3D_H
#define TOPO_PLANNER_WS_GRID_MAP_3D_H

#include <future>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace perception
{
    enum class Status3D : int8_t
    {
        Free_3D = 0,
        Occupied_3D = 1,
        Empty_3D = 2,
        Unknown_3D = 3
    };

    struct Grid3D
    {
        Status3D status;
        pcl::PointCloud<pcl::PointXYZI> points;

        Grid3D() : status(Status3D::Unknown_3D){};

        explicit Grid3D(Status3D sta) : status(sta){};

        void clear()
        {
            status = Status3D::Unknown_3D;
            points.clear();
        };
    };

    typedef Eigen::Vector3d GridPoint3D;
    typedef Eigen::Vector3i Index3D;
    typedef std::vector<Index3D> Ray3D;

    // Convert coordinate index to hash index
    struct IdxHash3D
    {
        size_t operator()(const Index3D &key) const
        {
            return ((std::hash<int>()(key.x()) ^ (std::hash<int>()(key.y()) << 1) ^ (std::hash<int>()(key.z()) << 2)) >> 2);
        }
    };

    struct EqualIdx3D
    {
        bool operator()(const Index3D &lhs, const Index3D &rhs) const
        {
            return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
        }
    };

    typedef std::unordered_set<Index3D, IdxHash3D, EqualIdx3D> Index3DSet;

    class GridMap3D
    {
    public:
        typedef std::shared_ptr<GridMap3D> Ptr;

        GridMap3D();

        GridMap3D(double &grid_size, int &grid_x_num, int &grid_y_num, int &grid_z_num, Status3D init_status);

        GridMap3D(double &grid_size, double &x_length, double &y_length, double &z_length, Status3D init_status);

        // 拷贝构造函数
        GridMap3D(const GridMap3D &other)
        {
            grid_size_ = other.grid_size_;
            center_point_ = other.center_point_;
            map_grid_num_ = Eigen::Vector3i(other.map_grid_num_);
            map_length_ = Eigen::Vector3d(other.map_length_);
            map_min_bound_ = Eigen::Vector3d(other.map_min_bound_);
            map_max_bound_ = Eigen::Vector3d(other.map_max_bound_);

            // 深拷贝 grids_ 数组
            grids_.resize(map_grid_num_[0]);
            for (int i = 0; i < map_grid_num_[0]; i++)
            {
                grids_[i].resize(map_grid_num_[1]);
                for (int j = 0; j < map_grid_num_[1]; j++)
                {
                    grids_[i][j].resize(map_grid_num_[2]);
                    for (int k = 0; k < map_grid_num_[2]; k++)
                    {
                        grids_[i][j][k] = other.grids_[i][j][k];
                    }
                }
            }
        }

        void initialize(double &grid_size, int &grid_x_num, int &grid_y_num, int &grid_z_num, Status3D init_status);

        void initialize(double &grid_size, double &x_length, double &y_length, double &z_length, Status3D init_status);

        void setMapCenterAndBoundary(GridPoint3D &center_point);

        void clearMap()
        {
            grids_.clear();
        };

        void resetMapStatus(Status3D status);

        Ray3D castRay(GridPoint3D &start, GridPoint3D &end);

        std::vector<Index3D> getAllOccupiedGrids();

        void inflateGridMap3D(const double &inflate_radius, const double &inflate_empty_radius);

        visualization_msgs::MarkerArray
        generateMapMarkers(const std::vector<std::vector<std::vector<Grid3D>>> &grid_map, const geometry_msgs::Pose &current_pose);

        inline bool isCollisionFreeStraight(const GridPoint3D &source, const GridPoint3D &target) const
        {
            if (isInMapRange3D(source) && isInMapRange3D(target))
            {
                if (getStatusInMap3D(source) != Status3D::Free_3D || getStatusInMap3D(target) != Status3D::Free_3D)
                {
                    return false;
                }
                else
                {
                    Index3D start_sub = getIndexInMap3D(source);
                    Index3D end_sub = getIndexInMap3D(target);
                    if (start_sub == end_sub)
                        return true;
                    Index3D diff_sub = end_sub - start_sub;
                    double max_dist = diff_sub.squaredNorm();
                    int step_x = signum(diff_sub.x());
                    int step_y = signum(diff_sub.y());
                    int step_z = signum(diff_sub.z());
                    double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
                    double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
                    double t_max_z = step_z == 0 ? DBL_MAX : intbound(start_sub.z(), diff_sub.z());
                    double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
                    double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
                    double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
                    Index3D cur_sub = start_sub;

                    while (isInMapRange3D(cur_sub) && cur_sub != end_sub &&
                           (cur_sub - start_sub).squaredNorm() <= max_dist)
                    {
                        if (getStatusInMap3D(cur_sub) != Status3D::Free_3D)
                        {
                            return false;
                        }
                        if (t_max_x < t_max_y)
                        {
                            if (t_max_x < t_max_z)
                            {
                                // Update which cube we are now in.
                                cur_sub.x() += step_x;
                                // Adjust tMaxX to the next X-oriented boundary crossing.
                                t_max_x += t_delta_x;
                            }
                            else
                            {
                                cur_sub.z() += step_z;
                                t_max_z += t_delta_z;
                            }
                        }
                        else
                        {
                            if (t_max_y < t_max_z)
                            {
                                cur_sub.y() += step_y;
                                t_max_y += t_delta_y;
                            }
                            else
                            {
                                cur_sub.z() += step_z;
                                t_max_z += t_delta_z;
                            }
                        }
                    }
                    if (!isInMapRange3D(cur_sub))
                        return false;
                    return true;
                }
            }
            else
            {
                return false;
            }
        };

        inline void addPointInGrid(const Index3D &index, const pcl::PointXYZI &point)
        {
            grids_[index.x()][index.y()][index.z()].points.push_back(point);
        }

        inline void addPointInGrid(const int &x, const int &y, const int &z, const pcl::PointXYZI &point)
        {
            grids_[x][y][z].points.push_back(point);
        }

        inline void setFree(const Index3D &index)
        {
            grids_[index.x()][index.y()][index.z()].status = Status3D::Free_3D;
        };

        inline void setFree(const int &x, const int &y, const int &z)
        {
            grids_[x][y][z].status = Status3D::Free_3D;
        };

        inline void setOccupied(const Index3D &index)
        {
            grids_[index.x()][index.y()][index.z()].status = Status3D::Occupied_3D;
        };

        inline void setOccupied(const int &x, const int &y, const int &z)
        {
            grids_[x][y][z].status = Status3D::Occupied_3D;
        };

        inline void setEmpty(const Index3D &index)
        {
            grids_[index.x()][index.y()][index.z()].status = Status3D::Empty_3D;
        };

        inline void setEmpty(const int &x, const int &y, const int &z)
        {
            grids_[x][y][z].status = Status3D::Empty_3D;
        };

        inline void setUnknown(const Index3D &index)
        {
            grids_[index.x()][index.y()][index.z()].status = Status3D::Unknown_3D;
        };

        inline void setUnknown(const int &x, const int &y, const int &z)
        {
            grids_[x][y][z].status = Status3D::Unknown_3D;
        };

        inline Status3D getStatusInMap3D(const Index3D &index) const
        {
            if (isInMapRange3D(index))
                return grids_[index.x()][index.y()][index.z()].status;
            else
                return Status3D::Unknown_3D;
        }

        inline Status3D getStatusInMap3D(const GridPoint3D &point) const
        {
            if (isInMapRange3D(point))
            {
                return grids_[floor((point.x() - map_min_bound_[0]) / grid_size_)]
                             [floor((point.y() - map_min_bound_[1]) / grid_size_)]
                             [floor((point.z() - map_min_bound_[2]) / grid_size_)]
                                 .status;
            }
            else
            {
                return Status3D::Unknown_3D;
            }
        }

        inline bool isInMapRange3D(const GridPoint3D &point) const
        {
            if (point.x() < (map_min_bound_[0] + 1e-4) || point.x() > (map_max_bound_[0] - 1e-4) ||
                point.y() < (map_min_bound_[1] + 1e-4) || point.y() > (map_max_bound_[1] - 1e-4) ||
                point.z() < (map_min_bound_[2] + 1e-4) || point.z() > (map_max_bound_[2] - 1e-4))
            {
                return false;
            }
            else
            {
                return true;
            }
        };

        inline bool isInMapRange3D(const pcl::PointXYZI &point) const
        {
            if (point.x < map_min_bound_[0] + 1e-4 || point.x > map_max_bound_[0] - 1e-4 ||
                point.y < map_min_bound_[1] + 1e-4 || point.y > map_max_bound_[1] - 1e-4 ||
                point.z < map_min_bound_[2] + 1e-4 || point.z > map_max_bound_[2] - 1e-4)
            {
                return false;
            }
            else
            {
                return true;
            }
        };

        inline bool isInMapRange3D(const Index3D &index) const
        {
            if (index.x() >= 0 && index.x() < map_grid_num_[0] &&
                index.y() >= 0 && index.y() < map_grid_num_[1] &&
                index.z() >= 0 && index.z() < map_grid_num_[2])
                return true;
            else
                return false;
        }

        inline Index3D getIndexInMap3D(const GridPoint3D &point) const
        {
            Index3D index;
            index.x() = floor((point.x() - map_min_bound_[0]) / grid_size_);
            index.y() = floor((point.y() - map_min_bound_[1]) / grid_size_);
            index.z() = floor((point.z() - map_min_bound_[2]) / grid_size_);
            return index;
        };

        inline Index3D getIndexInMap3D(const pcl::PointXYZI &point) const
        {
            Index3D index;
            index.x() = floor((point.x - map_min_bound_[0]) / grid_size_);
            index.y() = floor((point.y - map_min_bound_[1]) / grid_size_);
            index.z() = floor((point.z - map_min_bound_[2]) / grid_size_);
            return index;
        };

        inline GridPoint3D getGridCenter(const Index3D &index) const
        {
            GridPoint3D center;
            center.x() = map_min_bound_[0] + grid_size_ / 2 + index.x() * grid_size_;
            center.y() = map_min_bound_[1] + grid_size_ / 2 + index.y() * grid_size_;
            center.z() = map_min_bound_[2] + grid_size_ / 2 + index.z() * grid_size_;
            return center;
        };

        inline GridPoint3D getGridCenter(const int &index_x, const int &index_y, const int &index_z) const
        {
            GridPoint3D center;
            center.x() = map_min_bound_[0] + grid_size_ / 2 + index_x * grid_size_;
            center.y() = map_min_bound_[1] + grid_size_ / 2 + index_y * grid_size_;
            center.z() = map_min_bound_[2] + grid_size_ / 2 + index_z * grid_size_;
            return center;
        };

        inline GridPoint3D getGridCenter(const int &grid_id) const
        {
            GridPoint3D center;
            center.x() = map_min_bound_[0] + grid_size_ / 2 + grid_size_ * (grid_id / (map_grid_num_[1] * map_grid_num_[2]));
            center.y() = map_min_bound_[1] + grid_size_ / 2 + grid_size_ * ((grid_id % (map_grid_num_[1] * map_grid_num_[2])) / map_grid_num_[2]);
            center.z() = map_min_bound_[2] + grid_size_ / 2 + grid_size_ * ((grid_id % (map_grid_num_[1] * map_grid_num_[2])) % map_grid_num_[2]);
            return center;
        }

        inline int index3DToGridId(const Index3D &index) const
        {
            return index.x() * (map_grid_num_[1] * map_grid_num_[2]) + index.y() * map_grid_num_[2] + index.z();
        };

        inline int index3DToGridId(const int &index_x, const int &index_y, const int &index_z) const
        {
            return index_x * (map_grid_num_[1] * map_grid_num_[2]) + index_y * map_grid_num_[2] + index_z;
        }

        inline Index3D gridIdToIndex3D(const int &id) const
        {
            Index3D index;
            index.x() = id / (map_grid_num_[1] * map_grid_num_[2]);
            index.y() = (id % (map_grid_num_[1] * map_grid_num_[2])) / map_grid_num_[2];
            index.z() = (id % (map_grid_num_[1] * map_grid_num_[2])) % map_grid_num_[2];
            return index;
        };

        inline Grid3D getGrid3D(const Index3D &index) const
        {
            if (isInMapRange3D(index))
                return grids_[index.x()][index.y()][index.z()];
            else
                return Grid3D();
        }

        inline bool isNearOccupy(const GridPoint3D &point, const double delta) const
        {
            double t_delta = grid_size_;
            double delta_real = std::fmax(delta, t_delta);
            for (double x = (point.x() - delta_real); x <= (point.x() + delta_real); x += t_delta)
            {
                for (double y = (point.y() - delta_real); y <= (point.y() + delta_real); y += t_delta)
                {
                    for (double z = (point.z() - delta_real); z <= (point.z() + delta_real); z += t_delta)
                    {
                        GridPoint3D point3d(x, y, z);
                        if (getStatusInMap3D(point3d) == Status3D::Occupied_3D)
                            return true;
                    }
                }
            }
            return false;
        };

        inline bool isNearEmpty(const GridPoint3D &point, const double delta) const
        {
            double t_delta = grid_size_;
            double delta_real = std::fmax(delta, t_delta);
            for (double x = (point.x() - delta_real); x <= (point.x() + delta_real); x += t_delta)
            {
                for (double y = (point.y() - delta_real); y <= (point.y() + delta_real); y += t_delta)
                {
                    for (double z = (point.z() - delta_real); z <= (point.z() + delta_real); z += t_delta)
                    {
                        GridPoint3D point3d(x, y, z);
                        if (getStatusInMap3D(point3d) == Status3D::Empty_3D)
                            return true;
                    }
                }
            }
            return false;
        };

        inline bool isNearUnknown(const GridPoint3D &point, const double delta) const
        {
            double t_delta = grid_size_;
            double delta_real = std::fmax(delta, t_delta);
            for (double x = (point.x() - delta_real); x <= (point.x() + delta_real); x += t_delta)
            {
                for (double y = (point.y() - delta_real); y <= (point.y() + delta_real); y += t_delta)
                {
                    for (double z = (point.z() - delta_real); z <= (point.z() + delta_real); z += t_delta)
                    {
                        GridPoint3D point3d(x, y, z);
                        if (getStatusInMap3D(point3d) == Status3D::Unknown_3D)
                            return true;
                    }
                }
            }
            return false;
        };

        inline bool isNearFree(const GridPoint3D &point, const double delta) const
        {
            double t_delta = grid_size_;
            double delta_real = std::fmax(delta, t_delta);
            for (double x = (point.x() - delta_real); x <= (point.x() + delta_real); x += t_delta)
            {
                for (double y = (point.y() - delta_real); y <= (point.y() + delta_real); y += t_delta)
                {
                    for (double z = (point.z() - delta_real); z <= (point.z() + delta_real); z += t_delta)
                    {
                        GridPoint3D point3d(x, y, z);
                        if (getStatusInMap3D(point3d) == Status3D::Free_3D)
                            return true;
                    }
                }
            }
            return false;
        }

        bool getShortestPath(const GridPoint3D &goal, const GridPoint3D &start, std::vector<GridPoint3D> &path3d) const;

        std::vector<GridPoint3D> optimalToStraight(std::vector<GridPoint3D> &path) const;

        inline int signum(const int &x) const
        {
            return x == 0 ? 0 : x < 0 ? -1
                                      : 1;
        }

        inline double mod(const double &value, const double &modulus) const
        {
            return fmod(fmod(value, modulus) + modulus, modulus);
        }

        inline double intbound(const double &s, const double &ds) const
        {
            // Find the smallest positive t such that s+t*ds is an integer.
            if (ds < 0)
            {
                return intbound(-s, -ds);
            }
            else
            {
                // problem is now s+t*ds = 1
                return (1 - mod(s, 1)) / ds;
            }
        }

        inline double minPositionElement(const GridPoint3D &pos) const
        {
            return std::min(std::min(pos[0], pos[1]), pos[2]);
        }

        inline double maxPositionElement(const GridPoint3D &pos) const
        {
            return std::max(std::max(pos[0], pos[1]), pos[2]);
        }

        inline size_t minElementIndex(const GridPoint3D &pos) const
        {
            return pos[0] <= pos[1] && pos[0] <= pos[2] ? 0 : pos[1] <= pos[2] ? 1
                                                                               : 2;
        }

        inline size_t maxElementIndex(const GridPoint3D &pos) const
        {
            return pos[0] >= pos[1] && pos[0] >= pos[2] ? 0 : pos[1] >= pos[2] ? 1
                                                                               : 2;
        }

        inline double getGridSize()
        {
            return this->grid_size_;
        }

        inline Eigen::Vector3d getCenterPoint()
        {
            return this->center_point_;
        }

        inline Eigen::Vector3i getMapGridNum()
        {
            return this->map_grid_num_;
        }

        inline Eigen::Vector3d getMapLength()
        {
            return this->map_length_;
        }

        inline Eigen::Vector3d getMapMinBound()
        {
            return this->map_min_bound_;
        }

        inline Eigen::Vector3d getMapMaxBound()
        {
            return this->map_max_bound_;
        }

        inline std::vector<std::vector<std::vector<Grid3D>>> getMapGrid()
        {
            return this->grids_;
        }

    private:
        double grid_size_;
        GridPoint3D center_point_;
        Eigen::Vector3i map_grid_num_;
        Eigen::Vector3d map_length_;
        Eigen::Vector3d map_min_bound_;
        Eigen::Vector3d map_max_bound_;
        std::vector<std::vector<std::vector<Grid3D>>> grids_;
    };
}

#endif // TOPO_PLANNER_WS_GRID_MAP_3D_H
