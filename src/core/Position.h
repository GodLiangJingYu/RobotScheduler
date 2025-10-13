#pragma once

#include <functional>  // 为std::hash添加

class Position {
public:
    int x, y;

    Position() : x(0), y(0) {}
    Position(int x, int y) : x(x), y(y) {}

    // 添加相等运算符
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

// 为Position类添加哈希函数支持
namespace std {
    template <>
    struct hash<Position> {
        std::size_t operator()(const Position& pos) const {
            // 简单的哈希函数，将x和y组合
            return static_cast<std::size_t>(pos.x * 31 + pos.y);
        }
    };
}