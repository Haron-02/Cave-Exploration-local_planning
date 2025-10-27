//
// Created by hjl on 2022/6/13.
//

#ifndef __IKDTREE_PLAN_GRAPH__
#define __IKDTREE_PLAN_GRAPH__

#include <iostream>
#include <memory>
#include <unordered_map>
#include <shared_mutex>

#include "utils/point3d.h"
#include "utils/ikd_Tree.h"

namespace graph
{
    using namespace std;
    using PointType = ikdTree_PointType;
    using PointVector = KD_TREE<PointType>::PointVector;

    class IkdTreePlanGraph
    {
    public:
        typedef shared_ptr<IkdTreePlanGraph> Ptr;

        mutable std::shared_timed_mutex graph_mutex_;
        mutable std::shared_timed_mutex point_cloud_mutex_;
        mutable std::shared_timed_mutex delete_point_cloud_mutex_;

        explicit IkdTreePlanGraph();

        void resetGraph();

        // add and remove elements;
        int addVertex(const utils::Point3D &point);

        bool addOneWayEdge(const int &a_id, const int &b_id, const double &distance);

        bool addTwoWayEdge(const int &a_id, const int &b_id);

        void deleteVertex(const utils::Point3D &point, bool update_kdtree = true);

        bool isPoint3DExisted(const utils::Point3D &point) const;

        // extraction
        utils::Point3D getVertex(const int &id) const;

        bool getPointId(const utils::Point3D &point, int &point_id);

        int getLinkNums(const int &point_id) const;

        const std::vector<utils::Point3D> getAllVertices() const;

        const std::unordered_map<int, std::unordered_map<int, double>> getAllEdges() const;

        void buildKdtree();

        void addPointsToKdtree();

        int getNearestVertexId(const utils::Point3D &point);

        std::vector<int> getNeighborVertexsIDs(const utils::Point3D &point,
                                               const double &range, std::vector<utils::Point3D> &neighbor_vertexs);

        bool getShortestPath(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids,
                             utils::Point3DQueue &shortest_path) const;

    private:
        bool A_star_search(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids) const;

        bool vertexIndexInRange(const int &index) const
        {
            return index >= 0 && index < vertex_index_;
        }

        bool is_tree_built_;
        PointVector point_cloud_;
        PointVector delete_point_cloud_;
        KD_TREE<PointType>::Ptr graph_ikdtree_;

        int vertex_index_;
        utils::Point3DMap<int> points_ids_;                // store the ids of points.
        std::unordered_map<int, utils::Point3D> vertices_; // vertex  positions
        std::unordered_map<int, std::unordered_map<int, double>> edges_;
    };
}

#endif // __IKDTREE_PLAN_GRAPH__
