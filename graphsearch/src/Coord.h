#ifndef COORD_H
#define COORD_H

namespace conop3 {

struct Coord {
    int i, j, k, l;
    bool operator==(const Coord &other) const {
        return i == other.i && j == other.j && k == other.k && l == other.l;
    }
};

} // namespace conops3

#include <functional>
namespace std {
    template <>
    struct hash<conop3::Coord> {
        std::size_t operator()(const conop3::Coord &c) const noexcept {
            std::size_t h1 = std::hash<int>{}(c.i);
            std::size_t h2 = std::hash<int>{}(c.j);
            std::size_t h3 = std::hash<int>{}(c.k);
            std::size_t h4 = std::hash<int>{}(c.l);
            return (((h1 ^ (h2 << 1)) >> 1) ^ (h3 << 1)) ^ (h4 << 1);
        }
    };
}

#endif // COORD_H
