//
// Created by hjl on 2021/11/13.
//

#include "perception3d/grid_map_3d.h"

namespace perception
{

    GridMap3D::GridMap3D()
    {
    }

    GridMap3D::GridMap3D(double &grid_size, int &grid_x_num, int &grid_y_num, int &grid_z_num, Status3D init_status)
    {
        initialize(grid_size, grid_x_num, grid_y_num, grid_z_num, init_status);
    }

    GridMap3D::GridMap3D(double &grid_size, double &x_length, double &y_length, double &z_length, Status3D init_status)
    {
        initialize(grid_size, x_length, y_length, z_length, init_status);
    }

    void GridMap3D::initialize(double &grid_size, int &grid_x_num, int &grid_y_num, int &grid_z_num, Status3D init_status)
    {
        grid_size_ = grid_size;
        map_grid_num_[0] = grid_x_num;
        map_grid_num_[1] = grid_y_num;
        map_grid_num_[2] = grid_z_num;

        map_length_[0] = grid_size * grid_x_num;
        map_length_[1] = grid_size * grid_y_num;
        map_length_[2] = grid_size * grid_z_num;

        grids_ = std::vector<std::vector<std::vector<Grid3D>>>(
            map_grid_num_[0],
            std::vector<std::vector<Grid3D>>(
                map_grid_num_[1],
                std::vector<Grid3D>(map_grid_num_[2], Grid3D(init_status))));
    }

    void GridMap3D::initialize(double &grid_size, double &x_length, double &y_length, double &z_length, Status3D init_status)
    {
        grid_size_ = grid_size;
        map_length_[0] = x_length;
        map_length_[1] = y_length;
        map_length_[2] = z_length;

        map_grid_num_[0] = std::ceil(x_length / grid_size);
        map_grid_num_[1] = std::ceil(y_length / grid_size);
        map_grid_num_[2] = std::ceil(z_length / grid_size);

        grids_ = std::vector<std::vector<std::vector<Grid3D>>>(
            map_grid_num_[0],
            std::vector<std::vector<Grid3D>>(
                map_grid_num_[1],
                std::vector<Grid3D>(map_grid_num_[2], Grid3D(init_status))));
    }

    void GridMap3D::setMapCenterAndBoundary(GridPoint3D &center_point)
    {
        center_point_ = center_point;
        map_min_bound_[0] = center_point.x() - map_length_[0] / 2;
        map_max_bound_[0] = center_point.x() + map_length_[0] / 2;
        map_min_bound_[1] = center_point.y() - map_length_[1] / 2;
        map_max_bound_[1] = center_point.y() + map_length_[1] / 2;
        map_min_bound_[2] = center_point.z() - map_length_[2] / 2;
        map_max_bound_[2] = center_point.z() + map_length_[2] / 2;
    }

    void GridMap3D::resetMapStatus(Status3D status)
    {
        for (auto &items : grids_)
        {
            for (auto &item : items)
            {
                for (auto &grid : item)
                {
                    grid.status = status;
                }
            }
        }
    }

    Ray3D GridMap3D::castRay(GridPoint3D &start, GridPoint3D &end)
    {
        Ray3D ray;
        Index3D start_sub = getIndexInMap3D(start);
        Index3D end_sub = getIndexInMap3D(end);
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
        double dist;
        Index3D cur_sub = start_sub;

        while (isInMapRange3D(cur_sub))
        {
            ray.push_back(cur_sub);
            dist = (cur_sub - start_sub).squaredNorm();
            if (cur_sub == end_sub || dist > max_dist || getStatusInMap3D(cur_sub) != Status3D::Free_3D)
            {
                return ray;
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
        return ray;
    }

    std::vector<Index3D> GridMap3D::getAllOccupiedGrids()
    {
        std::vector<Index3D> occupied_grids;
        for (int i = 0; i < map_grid_num_[0]; ++i)
        {
            for (int j = 0; j < map_grid_num_[1]; ++j)
            {
                for (int k = 0; k < map_grid_num_[2]; ++k)
                    if (grids_[i][j][k].status == Status3D::Occupied_3D)
                    {
                        occupied_grids.emplace_back(i, j, k);
                    }
            }
        }
        return occupied_grids;
    }

    void GridMap3D::inflateGridMap3D(const double &inflate_radius, const double &inflate_empty_radius)
    {
        auto inflate_grids = grids_;
        int increment = static_cast<int>(std::ceil(abs(inflate_radius / grid_size_)));
        int increment_empty = static_cast<int>(std::ceil(abs(inflate_empty_radius / grid_size_)));
        for (int I = 0; I < map_grid_num_[0]; ++I)
        {
            for (int J = 0; J < map_grid_num_[1]; ++J)
            {
                for (int K = 0; K < map_grid_num_[2]; ++K)
                {
                    if (grids_[I][J][K].status == Status3D::Occupied_3D)
                    {
                        for (int i = -increment; i <= increment; ++i)
                        {
                            for (int j = -increment; j <= increment; ++j)
                            {
                                for (int k = -increment; k <= increment; ++k)
                                {
                                    if (I + i >= 0 && I + i < map_grid_num_[0] &&
                                        J + j >= 0 && J + j < map_grid_num_[1] &&
                                        K + k >= 0 && K + k < map_grid_num_[2])
                                    {
                                        inflate_grids[I + i][J + j][K + k].status = Status3D::Occupied_3D;
                                    }
                                }
                            }
                        }
                    }

                    if (grids_[I][J][K].status == Status3D::Empty_3D)
                    {
                        for (int i = -increment_empty; i <= increment_empty; ++i)
                        {
                            for (int j = -increment_empty; j <= increment_empty; ++j)
                            {
                                for (int k = -increment_empty; k <= increment_empty; ++k)
                                {
                                    if (I + i >= 0 && I + i < map_grid_num_[0] &&
                                        J + j >= 0 && J + j < map_grid_num_[1] &&
                                        K + k >= 0 && K + k < map_grid_num_[2])
                                    {
                                        inflate_grids[I + i][J + j][K + k].status = Status3D::Empty_3D;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        grids_ = inflate_grids;
    }

    visualization_msgs::MarkerArray
    GridMap3D::generateMapMarkers(const std::vector<std::vector<std::vector<Grid3D>>> &grid_map,
                                  const geometry_msgs::Pose &current_pose)
    {

        visualization_msgs::Marker free;
        visualization_msgs::Marker occupied;
        visualization_msgs::Marker empty;
        visualization_msgs::Marker unknow;
        visualization_msgs::Marker center_point;

        empty.header.frame_id = occupied.header.frame_id = free.header.frame_id = unknow.header.frame_id = center_point.header.frame_id = "world";
        empty.header.stamp = occupied.header.stamp = free.header.stamp = unknow.header.stamp = center_point.header.stamp = ros::Time::now();
        occupied.ns = "occupied";
        free.ns = "free";
        empty.ns = "empty";
        unknow.ns = "unknown";
        center_point.ns = "center";
        empty.action = occupied.action = free.action = unknow.action = center_point.action = visualization_msgs::Marker::ADD;
        empty.pose.orientation.w = occupied.pose.orientation.w = free.pose.orientation.w = unknow.pose.orientation.w = center_point.pose.orientation.w = 1.0;

        free.id = 0;
        free.type = visualization_msgs::Marker::CUBE_LIST;

        occupied.id = 1;
        occupied.type = visualization_msgs::Marker::CUBE_LIST;

        empty.id = 2;
        empty.type = visualization_msgs::Marker::CUBE_LIST;

        unknow.id = 3;
        unknow.type = visualization_msgs::Marker::CUBE_LIST;

        center_point.id = 4;
        center_point.type = visualization_msgs::Marker::SPHERE_LIST;

        // setting scale
        free.scale.x = grid_size_;
        free.scale.y = grid_size_;
        free.scale.z = 0.01;

        // assigning colors
        free.color.r = 0.5f;
        free.color.g = 1.0f;
        free.color.b = 0.5f;
        free.color.a = 0.5f;

        occupied.scale.x = grid_size_;
        occupied.scale.y = grid_size_;
        occupied.scale.z = 0.01;

        occupied.color.r = 0.1f;
        occupied.color.g = 0.5f;
        occupied.color.b = 0.5f;
        occupied.color.a = 1.0f;

        empty.scale.x = grid_size_;
        empty.scale.y = grid_size_;
        empty.scale.z = 0.01;

        empty.color.r = 0.5f;
        empty.color.g = 0.5f;
        empty.color.b = 0.5f;
        empty.color.a = 0.75f;

        unknow.scale.x = grid_size_;
        unknow.scale.y = grid_size_;
        unknow.scale.z = 0.01;

        unknow.color.r = 1.0f;
        unknow.color.g = 0.1f;
        unknow.color.b = 0.1f;
        unknow.color.a = 0.5f;

        center_point.scale.x = grid_size_;
        center_point.scale.y = grid_size_;
        center_point.scale.z = grid_size_;
        center_point.color.r = 1.0f;
        center_point.color.a = 1.0f;

        geometry_msgs::Point point;
        point.x = center_point_.x();
        point.y = center_point_.y();
        point.z = center_point_.z();
        center_point.points.push_back(point);

        for (int i = 0; i < map_grid_num_[0]; i++)
        {
            for (int j = 0; j < map_grid_num_[1]; j++)
            {
                for (int k = 0; k < map_grid_num_[2]; k++)
                {
                    point.x = map_min_bound_[0] + grid_size_ / 2 + i * grid_size_;
                    point.y = map_min_bound_[1] + grid_size_ / 2 + j * grid_size_;
                    point.z = map_min_bound_[2] + grid_size_ / 2 + k * grid_size_;
                    if (grid_map[i][j][k].status == Status3D::Free_3D)
                    {
                        free.points.push_back(point);
                    }
                    if (grid_map[i][j][k].status == Status3D::Occupied_3D)
                    {
                        occupied.points.push_back(point);
                    }
                    if (grid_map[i][j][k].status == Status3D::Empty_3D)
                    {
                        empty.points.push_back(point);
                    }
                    if (grid_map[i][j][k].status == Status3D::Unknown_3D)
                    {
                        unknow.points.push_back(point);
                    }
                }
            }
        }

        visualization_msgs::MarkerArray grid_map_markers;
        grid_map_markers.markers.resize(5);
        grid_map_markers.markers[0] = free;
        grid_map_markers.markers[1] = occupied;
        grid_map_markers.markers[2] = empty;
        grid_map_markers.markers[3] = unknow;
        grid_map_markers.markers[4] = center_point;

        return grid_map_markers;
    }

    bool GridMap3D::getShortestPath(const GridPoint3D &goal, const GridPoint3D &start, std::vector<GridPoint3D> &path3d) const
    {
        path3d.clear();

        if (isInMapRange3D(start) && isInMapRange3D(goal) &&
            getStatusInMap3D(start) == Status3D::Free_3D && getStatusInMap3D(goal) == Status3D::Free_3D)
        {
            int start_grid_id = index3DToGridId(getIndexInMap3D(start));
            int goal_grid_id = index3DToGridId(getIndexInMap3D(goal));
            if (start_grid_id == goal_grid_id)
            { // starting point is the end point
                // std::cerr << "start point = end point, only return the start point or end point as a path" << std::endl;
                path3d.clear();
                path3d.push_back(start);
                return true;
            }

            enum class status : int8_t
            {
                OPEN,
                CLOSED,
                INIT
            };

            typedef std::pair<double, int> iPair;                                                             // Vertices are represented by their index in the graph.vertices list
            std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;                           // Priority queue of vertices
            std::vector<double> dist(map_grid_num_[0] * map_grid_num_[1] * map_grid_num_[2], INFINITY);       // Vector of distances
            std::vector<double> estimation(map_grid_num_[0] * map_grid_num_[1] * map_grid_num_[2], INFINITY); // Vector of G + H .
            const int INF = 0x3f3f3f3f;                                                                       // integer infinity
            std::vector<int> backpointers(map_grid_num_[0] * map_grid_num_[1] * map_grid_num_[2], INF);       // Vector of backpointers
            std::vector<status> in_pq(map_grid_num_[0] * map_grid_num_[1] * map_grid_num_[2], status::INIT);

            // ******* A star **********
            // Add the start vertex
            dist[start_grid_id] = 0;
            estimation[start_grid_id] = (start - goal).norm();
            pq.push(std::make_pair(estimation[start_grid_id], start_grid_id));
            in_pq[start_grid_id] = status::OPEN;

            std::cout << "A* search.." << std::endl;
            int u;
            int v;
            while (!pq.empty())
            { // Loop until priority queue is empty
                u = pq.top().second;
                pq.pop(); // Pop the minimum distance vertex
                in_pq[u] = status::CLOSED;
                if (u == goal_grid_id)
                { // Early termination
                    break;
                }
                for (int i = -1; i <= 1; ++i)
                { // Get all adjacent vertices
                    for (int j = -1; j <= 1; ++j)
                    {
                        for (int k = -1; k <= 1; ++k)
                        {
                            if (!(i == 0 && j == 0 && k == 0))
                            {
                                Index3D id_temp = gridIdToIndex3D(u) + Index3D(i, j, k);
                                if (isInMapRange3D(id_temp))
                                {
                                    if (grids_[id_temp.x()][id_temp.y()][id_temp.z()].status == Status3D::Free_3D)
                                    {
                                        // Get vertex label and weight of current adjacent edge of u
                                        v = index3DToGridId(id_temp);
                                        if (in_pq[v] == status::CLOSED)
                                        {
                                            continue;
                                        }

                                        // If there is a shorter path to v through u
                                        if (dist[v] > dist[u] + grid_size_ * sqrt(i * i + j * j + k * k))
                                        {
                                            // Updating distance of v
                                            dist[v] = dist[u] + grid_size_ * sqrt(i * i + j * j + k * k);
                                            // Diagonal-heuristic
                                            GridPoint3D diff = (getGridCenter(v) - getGridCenter(goal_grid_id)).cwiseAbs();
                                            double min_dist = minPositionElement(diff);
                                            double max_dist = maxPositionElement(diff);
                                            double med_dist = diff.sum() - min_dist - max_dist;
                                            double heuristic = max_dist + (sqrt(2.0) - 1) * med_dist + (sqrt(3.0) - sqrt(2.0)) * min_dist;
                                            // Updating estimation of v
                                            estimation[v] = dist[v] + heuristic;
                                            backpointers[v] = u;
                                            if (in_pq[v] == status::INIT)
                                            {
                                                pq.push(std::make_pair(estimation[v], v));
                                                in_pq[v] = status::OPEN;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Backtrack to find path
            std::vector<int> path;
            std::vector<int> reverse_path;
            int current = goal_grid_id;
            if (backpointers[current] == INF)
            { // no path found
                std::cerr << "no path found" << std::endl;
                path.clear();
                return false;
            }
            else
            {
                // path found
                while (current != INF)
                {
                    reverse_path.push_back(current);
                    current = backpointers[current];
                }

                // Reverse the path (constructing it this way since vector is more efficient
                // at push_back than insert[0])
                path.clear();
                for (int i = reverse_path.size() - 1; i >= 0; --i)
                {
                    path.push_back(reverse_path[i]);
                }

                path3d.clear();
                path3d.push_back(start);
                for (int i = 1; i < path.size() - 1; ++i)
                {
                    path3d.push_back(getGridCenter(path[i]));
                }
                path3d.push_back(goal);
                return true;
            }
        }
        else
        {
            path3d.clear();
            std::cout << "start or end point is out of map range" << std::endl;
            return false;
        }
    }

    std::vector<GridPoint3D> GridMap3D::optimalToStraight(std::vector<GridPoint3D> &path) const
    {
        if (path.size() > 2)
        {
            // The farthest point that can be straightened is the control point.
            std::vector<GridPoint3D> pruned_path;
            std::vector<int> control_point_ids;
            int inner_idx = 0;
            int control_point_id = inner_idx;
            control_point_ids.push_back(control_point_id);
            while (inner_idx < path.size() - 1)
            {
                control_point_id = inner_idx;
                for (int i = inner_idx + 1; i < path.size(); ++i)
                {
                    if (isCollisionFreeStraight(path[inner_idx], path[i]))
                    { // Use the straight line between two points
                        control_point_id = i;
                    }
                }
                if (control_point_id == inner_idx)
                {
                    control_point_id = inner_idx + 1; // next control point must be the next point of inner_idx
                }
                control_point_ids.push_back(control_point_id);
                inner_idx = control_point_id;
            }

            for (int i = 0; i < control_point_ids.size(); ++i)
            {
                pruned_path.push_back(path[control_point_ids[i]]);
            }
            return pruned_path;
        }
        else
        {
            return path;
        }
    }
}
