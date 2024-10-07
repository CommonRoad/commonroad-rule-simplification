#pragma once

#include <eigen3/Eigen/Dense>

#include <tuple>

namespace knowledge_extraction::ego_behavior::sets {
template <int N> struct Box {
    const Eigen::Vector<double, N> center;
    const Eigen::Vector<double, N> radius;

    Box(const Eigen::Vector<double, N> &center, const Eigen::Vector<double, N> &radius)
        : center(center), radius(radius) {
        assert((radius.array() >= 0).all());
    }

    static Box<N> from_bounds(const Eigen::Vector<double, N> &min, const Eigen::Vector<double, N> &max) {
        auto radius = (max - min) / 2;
        return Box{min + radius, radius};
    }

    std::pair<Eigen::Vector<double, N>, Eigen::Vector<double, N>> bounds() const {
        return {center - radius, center + radius};
    }

    Box<N> intersect(const Box<N> &other) const {
        auto [own_min, own_max] = bounds();
        auto [other_min, other_max] = other.bounds();
        auto min = own_min.cwiseMax(other_min);
        auto max = own_max.cwiseMin(other_max);
        if ((min.array() <= max.array()).all()) {
            return from_bounds(min, max);
        } else {
            throw std::runtime_error("Intersection leads to empty set, which cannot be represented.");
        }
    }

    Box<N> sum(const Box<N> &other) const { return Box{center + other.center, radius + other.radius}; }

    Box<N> shrink(const double &delta) const { return Box{center, (radius.array() - (delta / 2)).cwiseMax(0)}; }

    template <int M> Box<M> linear_map_positive(const Eigen::Matrix<double, M, N> &matrix) const {
        assert((matrix.array() >= 0).all());
        return Box<M>{matrix * center, matrix * radius};
    }

    bool operator==(const Box<N> &other) const { return center == other.center && radius == other.radius; }
};

using Box2D = Box<2>;
using Box4D = Box<4>;

template <int N> inline std::ostream &operator<<(std::ostream &os, const Box<N> &box) {
    os << "Box{ center:\n" << box.center << "\nradius:\n" << box.radius << "}";
    return os;
}
} // namespace knowledge_extraction::ego_behavior::sets
