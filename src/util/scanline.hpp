#pragma once

#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>

namespace world_exe::util {
class ScanLine {
public:
    /**
     * @brief 获取多边形轮廓内部的所有像素点
     *
     * 该函数使用扫描线算法填充多边形，返回轮廓内部的所有像素点坐标。
     *
     * @param image 输入图像，用于确定图像边界
     * @param contour 多边形轮廓点的集合
     * @return std::vector<cv::Point> 返回多边形内部所有像素点的坐标集合
     *
     * @note 如果轮廓为空，则返回空向量
     */
    static inline std::vector<cv::Point> get_points(
        const cv::Mat& image, const std::vector<cv::Point>& contour) {
        if (contour.empty()) return {};

        std::vector<cv::Point2l> points;
        points.reserve(contour.size());
        for (const auto& point : contour)
            points.emplace_back(point);

        // 收集多边形的所有边
        auto edges = collect_poly_edges(points);

        // 使用边表填充算法获取内部点
        return fill_edge_collection(image, edges);
    }

private:
    struct PolyEdge {
        PolyEdge()
            : y0(0)             // 边的起始Y坐标
            , y1(0)             // 边的结束Y坐标
            , x(0)              // 当前X坐标（定点数）
            , dx(0)             // X方向的增量（定点数）
            , next(nullptr) { } // 指向下一条边的指针

        int y0, y1;     // 边的Y坐标范围
        int64 x, dx;    // X坐标和X增量（使用定点数提高精度）
        PolyEdge* next; // 链表指针
    };

    /**
     * @brief 定点数相关常量定义
     *
     * XY_SHIFT: 定点数位移量，用于提高坐标计算精度
     * XY_ONE: 定点数的1.0表示
     * DRAWING_STORAGE_BLOCK: 绘制存储块大小
     */
    enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1 << 12) - 256 };

    /**
     * @details 算法流程：
     * 1. 计算边界框并进行边界检查
     * 2. 按Y坐标对边进行排序
     * 3. 逐行扫描，维护活动边表
     * 4. 在每条扫描线上，计算交点并填充像素
     */
    static inline std::vector<cv::Point> fill_edge_collection(
        const cv::Mat& img, std::vector<PolyEdge>& edges) {
        PolyEdge tmp;
        int i, y, total = (int)edges.size();
        cv::Size size = img.size();
        PolyEdge* e;
        int y_max = INT_MIN, y_min = std::numeric_limits<int>::max();
        int64 x_max = -1, x_min = 0x7FFFFFFFFFFFFFFF;

        if (total < 2) return {};

        // 计算边界框
        for (i = 0; i < total; i++) {
            PolyEdge& e1 = edges[i];
            assert(e1.y0 < e1.y1);
            // 计算边末端的X坐标（不一定是顶点的X坐标）
            int64 x1 = e1.x + (e1.y1 - e1.y0) * e1.dx;
            y_min    = std::min(y_min, e1.y0);
            y_max    = std::max(y_max, e1.y1);
            x_min    = std::min(x_min, e1.x);
            x_max    = std::max(x_max, e1.x);
            x_min    = std::min(x_min, x1);
            x_max    = std::max(x_max, x1);
        }

        // 边界检查：如果多边形完全在图像外部，直接返回
        if (y_max < 0 || y_min >= size.height || x_max < 0
            || x_min >= ((int64)size.width << XY_SHIFT))
            return {};

        // 按Y坐标排序边，Y相同时按X坐标排序，X也相同时按斜率排序
        std::sort(edges.begin(), edges.end(), [](const PolyEdge& e1, const PolyEdge& e2) {
            return e1.y0 - e2.y0 ? e1.y0 < e2.y0 : e1.x - e2.x ? e1.x < e2.x : e1.dx < e2.dx;
        });

        tmp.y0 = std::numeric_limits<int>::max();
        edges.push_back(tmp); // 添加哨兵边，此后不再向edges添加元素，可以安全使用指针
        i        = 0;
        tmp.next = nullptr;
        e        = &edges[i];
        y_max    = MIN(y_max, size.height);

        std::vector<cv::Point> points;
        // 逐行扫描
        for (y = e->y0; y < y_max; y++) {
            PolyEdge *last, *prelast, *keep_prelast;
            int sort_flag = 0;
            int draw      = 0;     // 绘制标志，奇偶规则
            int clipline  = y < 0; // 当前行是否在图像外部

            // 维护活动边表
            prelast = &tmp;
            last    = tmp.next;
            while (last || e->y0 == y) {
                if (last && last->y1 == y) {
                    // 如果边到达下端点，从活动边表中移除
                    prelast->next = last->next;
                    last          = last->next;
                    continue;
                }
                keep_prelast = prelast;
                if (last && (e->y0 > y || last->x < e->x)) {
                    // 移动到活动边表中的下一条边
                    prelast = last;
                    last    = last->next;
                } else if (i < total) {
                    // 如果到达边的上端点，将新边插入活动边表
                    prelast->next = e;
                    e->next       = last;
                    prelast       = e;
                    e             = &edges[++i];
                } else break;

                if (draw) {
                    if (!clipline) {
                        // 将定点数X坐标转换为图像坐标
                        int x1, x2;

                        if (keep_prelast->x > prelast->x) {
                            x1 = (int)((prelast->x + XY_ONE - 1) >> XY_SHIFT);
                            x2 = (int)(keep_prelast->x >> XY_SHIFT);
                        } else {
                            x1 = (int)((keep_prelast->x + XY_ONE - 1) >> XY_SHIFT);
                            x2 = (int)(prelast->x >> XY_SHIFT);
                        }

                        // 裁剪并绘制线段
                        if (x1 < size.width && x2 >= 0) {
                            if (x1 < 0) x1 = 0;
                            if (x2 >= size.width) x2 = size.width - 1;
                            for (int i = x1; i <= x2; i++)
                                points.emplace_back(i, y);
                        }
                    }
                    // 更新边的X坐标
                    keep_prelast->x += keep_prelast->dx;
                    prelast->x += prelast->dx;
                }
                draw ^= 1; // 切换绘制状态（奇偶规则）
            }

            // 对活动边表中的边按X坐标排序（使用冒泡排序）
            keep_prelast = 0;

            do {
                prelast = &tmp;
                last    = tmp.next;

                while (last != keep_prelast && last->next != 0) {
                    PolyEdge* te = last->next;

                    // 交换边的位置
                    if (last->x > te->x) {
                        prelast->next = te;
                        last->next    = te->next;
                        te->next      = last;
                        prelast       = te;
                        sort_flag     = 1;
                    } else {
                        prelast = last;
                        last    = te;
                    }
                }
                keep_prelast = prelast;
            } while (sort_flag && keep_prelast != tmp.next && keep_prelast != &tmp);
        }
        return points;
    }

    static inline std::vector<PolyEdge> collect_poly_edges(const std::vector<cv::Point2l>& points) {
        const auto points_size = points.size();
        cv::Point2l pt0        = points[points_size - 1], pt1;
        pt0.x                  = (pt0.x) << XY_SHIFT; // 转换为定点数

        std::vector<PolyEdge> edges;
        edges.reserve(edges.size() + points_size);

        // 遍历所有相邻顶点对，构建边
        for (std::size_t i = 0; i < points_size; i++, pt0 = pt1) {
            cv::Point2l t0, t1;
            pt1   = points[i];
            pt1.x = (pt1.x) << XY_SHIFT; // 转换为定点数

            t0.x = pt0.x;
            t1.x = pt1.x;
            t0.y = pt0.y << XY_SHIFT;
            t1.y = pt1.y << XY_SHIFT;

            // 跳过水平边
            if (pt0.y == pt1.y) continue;

            PolyEdge edge;
            // 确保边的方向统一：y0 < y1
            if (pt0.y < pt1.y) {
                edge.y0 = static_cast<int>(pt0.y);
                edge.y1 = static_cast<int>(pt1.y);
                edge.x  = pt0.x;
            } else {
                edge.y0 = static_cast<int>(pt1.y);
                edge.y1 = static_cast<int>(pt0.y);
                edge.x  = pt1.x;
            }
            // 计算X方向的增量（斜率的倒数）
            edge.dx = (pt1.x - pt0.x) / (pt1.y - pt0.y);
            edges.emplace_back(edge);
        }
        return edges;
    }
};
} // namespace world_exe::util
