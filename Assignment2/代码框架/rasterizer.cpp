// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    const Eigen::Vector3f* v = _v;
    Eigen::Vector3f ab = v[1] - v[0], ca = v[0] - v[2], bc = v[2] - v[1];
    Eigen::Vector3f p(x,y,0);
    Eigen::Vector3f ap = p-v[0],bp = p-v[1], cp = p-v[2];

    Eigen::Vector3f ansa = ab.cross(ap), ansb = bc.cross(bp), ansc = ca.cross(cp);
    if (ansa.z() >0 && ansb.z() > 0 && ansc.z() >0) {
        return true;
    }
    if (ansa.z() <0 && ansb.z() < 0 && ansc.z() < 0) {
        return true;
    }
    if (ansa.z() == 0 || ansb.z() == 0 || ansc.z() == 0) {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            //std::cout<<vert.z()<<std::endl;
            vert.z() = vert.z() * f1 + f2;

            //std::cout<<vert.z()<<std::endl;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;
    float minxf=std::numeric_limits<float>::infinity();
    float minyf = minxf;
    float maxxf=0,maxyf=0;
    for (int i=0; i<3; i++) {
        if (t.v[i].x() < minxf) {
            minxf = t.v[i].x();
        }
        if (t.v[i].x() > maxxf) {
            maxxf = t.v[i].x();
        }
        if (t.v[i].y() < minyf) {
            minyf = t.v[i].y();
        }
        if (t.v[i].y() > maxyf) {
            maxyf = t.v[i].y();
        }
    }
    //int minx = minxf, miny = minyf;
    //int maxx = int(maxxf) + 1, maxy = int(maxyf) + 1;
    int minx = floor(minxf), miny = floor(minyf);
    int maxx = ceil(maxxf), maxy = ceil(maxyf);

    //std::cout<<minx<<","<<miny<<","<<maxx<<","<<maxy<<std::endl;
    for (int i=minx; i<=maxx; i++) {
        for (int k=miny; k<=maxy; k++) {
            bool IsMSAA = true;
            if (IsMSAA == false) {
                float x = i+0.5, y = k+0.5;
                if (insideTriangle(x,y,t.v)) {
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if (-z_interpolated <= depth_buf[get_index(i,k)]) {
                        depth_buf[get_index(i,k)] = -z_interpolated;
                        set_pixel(Eigen::Vector3f(i,k,1.0),t.getColor());
                    }
                }
            } else {
                int InsideCount = 0;
                for (int m=0; m<=1; m++) {
                    for (int n=0; n<=1; n++) {
                        float x = (i+0.25) + m * 0.5;
                        float y = (k+0.25) + n * 0.5;
                        int idx = (2*i + m) + ((2*k+n + 1)-1) * (2*width);
                        // std::cout<<idx<<std::endl;
                        if (insideTriangle(x,y,t.v)) {
                            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            if (-z_interpolated <= msaa_depth_buf[idx]) {
                                msaa_depth_buf[idx] = -z_interpolated;
                                InsideCount++;
                            }
                        }
                    }
                }
                if (InsideCount > 0) {
                    float intensity = (float)InsideCount / 4.0;
                    mix_pixel(Eigen::Vector3f(i,k,1.0),t.getColor() * intensity);
                }
            }
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());

        // msaa
        std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }


}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    // msaa
    msaa_depth_buf.resize(2*w * 2*h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}
// msaa
void rst::rasterizer::mix_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] += color;

}
// clang-format on