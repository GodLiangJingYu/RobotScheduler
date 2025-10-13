#pragma once

#include <functional>  // 为std::hash添加

class Position {
public:
    int x, y;  // 或其他类型的坐标

    // 确保这些函数存在且不是被删除的
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    Position(const Position&) = default;
    ~Position() = default;  // 析构函数

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Position& other) const {
        return !(*this == other);
    }

    // 添加在这里
    bool operator<(const Position& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

// 为Position类添加哈希函数支持
namespace std {
    template <>
    struct hash<Position> {
        std::size_t operator()(const Position& p) const {
            // 组合 x 和 y 的哈希值
            std::size_t h1 = std::hash<int>{}(p.x);
            std::size_t h2 = std::hash<int>{}(p.y);
            return h1 ^ (h2 << 1); // 或使用其他方式组合哈希值
        }
    };
}