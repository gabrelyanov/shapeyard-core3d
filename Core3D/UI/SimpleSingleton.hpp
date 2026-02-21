//
// Created by Dmitry Sukhorukov on 25.11.2024.
//

#ifndef SIMPLESINGLETON_HPP
#define SIMPLESINGLETON_HPP

template<typename T>
class SimpleSingleton {
public:
    static T &Instance() {
        static T instance;
        return instance;
    }

protected:
    SimpleSingleton() = default;
    ~SimpleSingleton() = default;

public:
    SimpleSingleton(SimpleSingleton const &) = delete;
    SimpleSingleton &operator=(SimpleSingleton const &) = delete;
};

#endif //SIMPLESINGLETON_HPP
