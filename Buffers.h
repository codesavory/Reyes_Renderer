#pragma once
#include "Primitives.h"

template <class T>
class Buffer {
public:
    int w;
    int h;
    int xsamples, ysamples;
    std::vector<T> buff;

    Buffer(int width, int heigth, int xsamples, int ysamples) {
        this->w = width;
        this->h = heigth;
        this->xsamples = xsamples;
        this->ysamples = ysamples;
        buff.resize(width * xsamples * heigth * ysamples);
        //buff.resize(width * heigth);
    }

    T operator() (int x, int y, int m, int n) const {
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        //int id = y * w + x;
        return buff[id];
    }

    T& operator() (int x, int y, int m, int n) {
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        //int id = y * w + x;
        return buff[id];
    }
};


class FrameBuffer : public Buffer<Color> {
public:
    FrameBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples) {}

    Color collapse(int x, int y) {

        Eigen::Vector3f pix_color = Eigen::Vector3f::Zero();

        for (int m = 0; m < xsamples; ++m) {
            for (int n = 0; n < ysamples; ++n) {
                Color sc = this->operator()(x, y, m, n);
                pix_color += Eigen::Vector3f(sc.r(), sc.g(), sc.b());
            }
        }

        pix_color /= (xsamples * ysamples);
        Color c(pix_color[0], pix_color[1], pix_color[2]);
        return c;
    }
};



class ZBuffer : public Buffer<std::list<float>> {
public:
    ZBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples)
    {}

    std::list<float> operator() (int x, int y, int m, int n) const {
        std::list<float> buff_at_loc = Buffer<std::list<float>>::operator()(x, y, m, n);
        if (buff_at_loc.empty()) {
            buff_at_loc.push_front(std::numeric_limits<float>::infinity());
            return buff_at_loc;
        }
        else {
            return buff_at_loc;
        }
    }

    std::list<float>& operator() (int x, int y, int m, int n) {
        std::list<float>& buff_at_loc = Buffer<std::list<float>>::operator()(x, y, m, n);
        if (buff_at_loc.empty()) {
            buff_at_loc.push_front(std::numeric_limits<float>::infinity());
            return buff_at_loc;
        }
        else {
            return buff_at_loc;
        }
    }

};