#pragma once

#include <Eigen/Dense>

#include <tuple>

namespace knowledge_extraction::ego_behavior::sets {
template <int N> struct Box {
    const Eigen::Vector<double, N> center;
    const Eigen::Vector<double, N> radius;

    /**
     * Create a box from its center and radius.
     *
     * @param center The center point of the box.
     * @param radius The radius of the box in each dimension.
     */
    Box(const Eigen::Vector<double, N> &center, const Eigen::Vector<double, N> &radius)
        : center(center), radius(radius) {
        assert((radius.array() >= 0).all());
    }

    /**
     * Create a box from its bounds, i.e., the lower left and upper right corner (generalized to N dimensions).
     *
     * @param min The lower left corner of the box.
     * @param max The upper right corner of the box.
     * @return The box defined by the given bounds.
     */
    static Box from_bounds(const Eigen::Vector<double, N> &min, const Eigen::Vector<double, N> &max) {
        auto radius = (max - min) / 2;
        return Box{min + radius, radius};
    }

    /**
     * Get the bounds of the box, i.e., the lower left and upper right corner (generalized to N dimensions).
     *
     * @return The bounds of the box.
     */
    std::pair<Eigen::Vector<double, N>, Eigen::Vector<double, N>> bounds() const {
        return {center - radius, center + radius};
    }

    /**
     * Check whether two boxes intersect.
     *
     * @param other The other box.
     * @return True iff the boxes intersect.
     */
    Box intersect(const Box &other) const {
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

    /**
     * Compute the Minkowski sum of two boxes.
     *
     * @param other The other summand.
     * @return The box representing the Minkowski sum.
     */
    Box sum(const Box &other) const { return Box{center + other.center, radius + other.radius}; }

    /**
     * Shrink a box by a given amount in each dimension.
     *
     * @param delta The amount to shrink the box by in each dimension.
     * @return The shrunken box.
     */
    Box shrink(const double &delta) const { return Box{center, (radius.array() - (delta / 2)).cwiseMax(0)}; }

    /**
     * Compute the linear map of the box by a given matrix.
     *
     * The matrix must not contain negative entries.
     * Note that this operation results in an overapproximation.
     *
     * @tparam M Output dimension of the linear map.
     * @param matrix The non-negative matrix to use for the linear map.
     * @return The box overapproximating the result of the linear map.
     */
    template <int M> Box<M> linear_map_positive(const Eigen::Matrix<double, M, N> &matrix) const {
        assert((matrix.array() >= 0).all());
        return Box<M>{matrix * center, matrix * radius};
    }

    bool operator==(const Box &other) const = default;
};

/**
 * Alias for a two-dimensional box.
 */
using Box2D = Box<2>;

/**
 * Alias for a four-dimensional box.
 */
using Box4D = Box<4>;

template <int N> std::ostream &operator<<(std::ostream &os, const Box<N> &box) {
    os << "Box{ center:\n" << box.center << "\nradius:\n" << box.radius << "}";
    return os;
}
} // namespace knowledge_extraction::ego_behavior::sets
