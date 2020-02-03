#ifndef GEOMETRY_COMPRESSOR_HPP_
#define GEOMETRY_COMPRESSOR_HPP_

#include "extractor/segment_data_container.hpp"
#include "road_classification.hpp"

#include "util/typedefs.hpp"

#include <unordered_map>

#include <string>
#include <vector>

namespace osrm
{
namespace extractor
{

class CompressedEdgeContainer
{
  public:
    struct OnewayCompressedEdge
    {
      public:
        NodeID node_id;           // refers to an internal node-based-node
        SegmentWeight weight;     // the weight of the edge leading to this node
        SegmentDuration duration; // the duration of the edge leading to this node
        SegmentDistance distance; // the distance of the edge leading to this node
        RoadPriorityClass::Enum road_class; // the road class of the edge
    };

    using OnewayEdgeBucket = std::vector<OnewayCompressedEdge>;

    CompressedEdgeContainer();
    void CompressEdge(const EdgeID surviving_edge_id,
                      const EdgeID removed_edge_id,
                      const NodeID via_node_id,
                      const NodeID target_node,
                      const EdgeWeight weight1,
                      const EdgeWeight weight2,
                      const EdgeDuration duration1,
                      const EdgeDuration duration2,
                      const EdgeDistance distance1,
                      const EdgeDistance distance2,
                      const RoadPriorityClass::Enum road_class_1,
                      const RoadPriorityClass::Enum road_class_2,
                      // node-penalties can be added before/or after the traversal of an edge which
                      // depends on whether we traverse the link forwards or backwards.
                      const EdgeWeight node_weight_penalty = INVALID_EDGE_WEIGHT,
                      const EdgeDuration node_duration_penalty = MAXIMAL_EDGE_DURATION);

    void AddUncompressedEdge(const EdgeID edge_id,
                             const NodeID target_node,
                             const SegmentWeight weight,
                             const SegmentWeight duration,
                             const SegmentDistance distance,
                             const RoadPriorityClass::Enum road_class);

    void InitializeBothwayVector();
    unsigned ZipEdges(const unsigned f_edge_pos, const unsigned r_edge_pos);

    bool HasEntryForID(const EdgeID edge_id) const;
    bool HasZippedEntryForForwardID(const EdgeID edge_id) const;
    bool HasZippedEntryForReverseID(const EdgeID edge_id) const;
    void PrintStatistics() const;
    unsigned GetPositionForID(const EdgeID edge_id) const;
    unsigned GetZippedPositionForForwardID(const EdgeID edge_id) const;
    unsigned GetZippedPositionForReverseID(const EdgeID edge_id) const;
    const OnewayEdgeBucket &GetBucketReference(const EdgeID edge_id) const;
    bool IsTrivial(const EdgeID edge_id) const;
    NodeID GetFirstEdgeTargetID(const EdgeID edge_id) const;
    NodeID GetLastEdgeTargetID(const EdgeID edge_id) const;
    NodeID GetLastEdgeSourceID(const EdgeID edge_id) const;

    // Invalidates the internal storage
    std::unique_ptr<SegmentDataContainer> ToSegmentData();

  private:
    SegmentWeight ClipWeight(const SegmentWeight weight);
    SegmentDuration ClipDuration(const SegmentDuration duration);
    SegmentDistance ExpandDistance(const EdgeDistance distance_);

    int free_list_maximum = 0;
    std::atomic_size_t clipped_weights{0};
    std::atomic_size_t clipped_durations{0};
    std::atomic_size_t clipped_distances{0};

    void IncreaseFreeList();
    std::vector<OnewayEdgeBucket> m_compressed_oneway_geometries;
    std::vector<unsigned> m_free_list;
    std::unordered_map<EdgeID, unsigned> m_edge_id_to_list_index_map;
    std::unordered_map<EdgeID, unsigned> m_forward_edge_id_to_zipped_index_map;
    std::unordered_map<EdgeID, unsigned> m_reverse_edge_id_to_zipped_index_map;
    std::unique_ptr<SegmentDataContainer> segment_data;
};
}
}

#endif // GEOMETRY_COMPRESSOR_HPP_
