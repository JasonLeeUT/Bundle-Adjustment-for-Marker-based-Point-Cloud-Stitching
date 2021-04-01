//
// Created by cherichy on 2019/3/3.
//

#ifndef BI_CALIB_LINES_H
#define BI_CALIB_LINES_H

inline float distance(const cv::Point2f &p1, const cv::Point2f &p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

inline void twoPointLine(const cv::Point2f &p1, const cv::Point2f &p2, float l[3]) {
    l[0] = p1.y - p2.y;
    l[1] = p2.x - p1.x;
    l[2] = p1.x * p2.y - p2.x * p1.y;
}

inline float distance_p2l(const cv::Point2f &p, const float l[3]) {
    return std::abs(p.x * l[0] + p.y * l[1] + l[2]) / std::hypot(l[0], l[1]);
}

inline cv::Point2f crossPoint(const float l1[3], const float l2[3]) {
    float x = l1[1] * l2[2] - l1[2] * l2[1];
    float y = l1[2] * l2[0] - l1[0] * l2[2];
    float H = l1[0] * l2[1] - l1[1] * l2[0];
    return cv::Point2f(x / H, y / H);
}

#endif //BI_CALIB_LINES_H
