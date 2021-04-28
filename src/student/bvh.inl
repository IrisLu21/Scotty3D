
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <limits>
namespace PT {

const size_t num_buckets = 10;

template<typename Primitive>
std::vector<BBox> BVH<Primitive>::createBuckets(float min, float max, std::vector<Primitive*>* bucket, float bucket_size, int axis) {
    std::vector<BBox> boxes(num_buckets); 
    for (size_t i = 0; i < primitives.size(); i++) {
        Primitive &prim = primitives[i];
        Vec3 center = prim.bbox().center();
        float point;
        if (axis == 0) point = center.x;
        else if (axis == 1) point = center.y;
        else point = center.z;
        size_t j = (size_t) std::floor((point - min) / bucket_size);
        if (j == num_buckets) j = num_buckets - 1;
        bucket[j].push_back(&prim);
        boxes[j].enclose(prim.bbox());
    }
    return boxes;
}

template<typename Primitive>
std::tuple<float, size_t> BVH<Primitive>::minPartition(float bucket_size, std::vector<Primitive*>* bucket, std::vector<BBox> boxes) {
    float min_cost = std::numeric_limits<float>::max();
    size_t min_partition = 0;
    for (size_t i = 0; i < num_buckets; i++) {
        std::vector<Primitive*> A_prim;
        BBox A_box;
        for (size_t j = 0; j < i; j++) {
            for (size_t k = 0; k < bucket[j].size(); k++) {
                A_prim.push_back(bucket[j][k]);
            }
            A_box.enclose(boxes[j]);
        }
        std::vector<Primitive*> B_prim;
        BBox B_box;
        for (size_t j = i; j < num_buckets; j++) {
            for (size_t k = 0; k < bucket[j].size(); k++) {
                B_prim.push_back(bucket[j][k]);
            }
            B_box.enclose(boxes[j]);
        }
        float SA = A_box.surface_area();
        float SB = B_box.surface_area();
        float SN = SA + SB;
        size_t NA = A_prim.size();
        size_t NB = B_prim.size();
        float cost = (SA / SN) * NA + (SB / SN) * NB;
        if (cost < min_cost) {
            min_cost = cost;
            min_partition = i;
        }
    }
    return std::make_tuple(min_cost, min_partition);
}

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // check for leaf
    if (primitives.size() <= max_leaf_size) {

        BBox box;
        for(const Primitive& prim : primitives) box.enclose(prim.bbox());

        new_node(box, 0, primitives.size(), 0, 0);
        root_idx = nodes.size() - 1;
    } else {
        // generate bounding box for all primitives in prims
        BBox bbox;
        for(const Primitive& prim : primitives) bbox.enclose(prim.bbox());

        Vec3 min = bbox.min;
        Vec3 max = bbox.max;
        float x_min = min.x;
        float y_min = min.y;
        float z_min = min.z;
        float x_max = max.x;
        float y_max = max.y;
        float z_max = max.z;

        // check x axis and place in buckets
        float x_range = x_max - x_min;
        float x_bucket_size = x_range / num_buckets;
        std::vector<Primitive*> x_buckets[num_buckets];
        std::vector<BBox> x_bbox = createBuckets(x_min, x_max, x_buckets, x_bucket_size, 0);
        
        // find x partition w/ lowest cost
        std::tuple<float, size_t> x_result = minPartition(x_bucket_size, x_buckets, x_bbox);
        float x_min_cost = std::get<0>(x_result);
        size_t x_partition = std::get<1>(x_result);

        // check y axis
        float y_range = y_max - y_min;
        float y_bucket_size = y_range / num_buckets;
        std::vector<Primitive*> y_buckets[num_buckets];
        std::vector<BBox> y_bbox = createBuckets(y_min, y_max, y_buckets, y_bucket_size, 1);
        
        // find y partition w/ lowest cost
        std::tuple<float, size_t> y_result = minPartition(y_bucket_size, y_buckets, y_bbox);
        float y_min_cost = std::get<0>(y_result);
        size_t y_partition = std::get<1>(y_result);

        // check z axis
        float z_range = z_max - z_min;
        float z_bucket_size = z_range / num_buckets;
        std::vector<Primitive*> z_buckets[num_buckets];
        std::vector<BBox> z_bbox = createBuckets(z_min, z_max, z_buckets, z_bucket_size, 2);
        
        // find z partition w/ lowest cost
        std::tuple<float, size_t> z_result = minPartition(z_bucket_size, z_buckets, z_bbox);
        float z_min_cost = std::get<0>(z_result);
        size_t z_partition = std::get<1>(z_result);

        std::vector<Primitive*>* bucket;
        size_t partition;
        
        if (x_min_cost <= y_min_cost && x_min_cost <= z_min_cost) { // use x partition
            bucket = x_buckets;
            partition = x_partition;
        } else if (y_min_cost <= x_min_cost && y_min_cost <= z_min_cost) { // use y partition
            bucket = y_buckets;
            partition = y_partition;
        } else { // use z partition
            bucket = z_buckets;
            partition = z_partition;
        }

        std::vector<Primitive> partitionA;
        std::vector<Primitive> partitionB;
        for (size_t i = 0; i < partition; i++) {
            for (size_t j = 0; j < bucket[i].size(); j++) {
                partitionA.push_back(std::move(*bucket[i][j]));
            }
        }
        for (size_t i = partition; i < num_buckets; i++) {
            for (size_t j = 0; j < bucket[i].size(); j++) {
                partitionB.push_back(std::move(*bucket[i][j]));
            }
        }

        BVH A(std::move(partitionA), max_leaf_size);
        BVH B(std::move(partitionB), max_leaf_size);
        
        size_t A_prims = A.primitives.size();
        primitives = std::move(A.primitives);
        for (size_t i = 0; i < B.primitives.size(); i++) {
            primitives.push_back(std::move(B.primitives[i]));
        }
        nodes = A.nodes;
        for (size_t i = 0; i < B.nodes.size(); i++) {
            Node node = B.nodes[i];
            new_node(node.bbox, node.start+A_prims, node.size, node.l+A.nodes.size(), node.r+A.nodes.size());
        }
        new_node(bbox, 0, primitives.size(), A.root_idx, B.root_idx + A.nodes.size()); // root node
        root_idx = A.nodes.size() + B.nodes.size();
    }
}

template<typename Primitive> Trace BVH<Primitive>::hit_node(const Ray& ray, Node node) const {
    Trace ret;
    
    if (node.bbox.hit(ray, ray.dist_bounds)) {
        if (node.is_leaf()) {
            for (size_t i = node.start; i < node.start + node.size; i++) {
                const Primitive &prim = primitives[i];
                Trace hit = prim.hit(ray);
                ret = Trace::min(ret, hit);
            }
        } else {
            Trace l_hit = hit_node(ray, nodes[node.l]);
            Trace r_hit = hit_node(ray, nodes[node.r]);
            ret = Trace::min(l_hit, r_hit);
        }
    } 
    return ret;
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Node root = nodes[nodes.size() - 1];
    
    Trace ret = hit_node(ray, root);
    // for(const Primitive& prim : primitives) {
    //     Trace hit = prim.hit(ray);
    //     ret = Trace::min(ret, hit);
    // }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {
    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r && !node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
