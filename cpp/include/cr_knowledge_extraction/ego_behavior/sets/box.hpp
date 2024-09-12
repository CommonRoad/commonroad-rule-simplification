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

    Box<N> sum(const Box<N> &other) { return Box{center + other.center, radius + other.radius}; }

    Box<N> shrink(const double &delta) { return Box{center, (radius.array() - (delta / 2)).cwiseMax(0)}; }

    template <int M> Box<M> linear_map_positive(const Eigen::Matrix<double, M, N> &matrix) {
        assert((matrix.array() >= 0).all());
        return Box<M>{matrix * center, matrix * radius};
    }

    bool operator==(const Box<N> &other) const { return center == other.center && radius == other.radius; }
};

template <int N> inline std::ostream &operator<<(std::ostream &os, const Box<N> &box) {
    os << "Box{ center:\n" << box.center << "\nradius:\n" << box.radius << "}";
    return os;
}
} // namespace knowledge_extraction::ego_behavior::sets
